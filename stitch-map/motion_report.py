#!/usr/bin/env python3
"""Plot common-time camera motion, IMU averaging and drift diagnostics.

All metric visual estimates are model references. Gyro bias/timing are fit
on a prefix; no later visual positions correct the free IMU trajectory.
Visual-reset predictions use only preceding visual poses and are scored
BEFORE the next camera observation. Averaging means a five-sample temporal
mean of each sensor axis, never an average of unrelated XYZ coordinates.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from recording_camera import Recording, COLOR, GYRO, ACCEL
from motion_channels import (angle, estimate_static_window, inertial_path,
    rotations_at, velocity_from_visual_displacement)
from inertial_initialization import integrals
from window_motion import flow_pairs


def temporal_mean(values, window=5):
    values = np.asarray(values, float)
    if window < 1 or window % 2 != 1:
        raise ValueError('temporal mean window must be positive and odd')
    pad = window//2
    padded = np.pad(values, ((pad, pad), (0, 0)), mode='edge')
    return np.column_stack([np.convolve(padded[:, k], np.ones(window)/window, mode='valid')
                            for k in range(values.shape[1])])


def relative_motion(a, b):
    """Camera a coordinates to camera b coordinates; same convention as PnP."""
    ra, pa = a
    rb, pb = b
    return rb.T@ra, rb.T@(pa-pb)


def rotation_vectors_for_plot(rotations):
    """Principal angle-axis values with line breaks at coordinate branch cuts.

    A globally continuous three-vector orientation does not exist. Extending
    angle-axis branches toward 360 degrees also amplifies axis uncertainty
    near identity. Keep principal values and show the discontinuity honestly.
    """
    result = []
    previous = None
    for rotation in rotations:
        if not np.isfinite(rotation).all():
            result.append(np.full(3, np.nan))
            previous = None
            continue
        vector = cv2.Rodrigues(rotation)[0].ravel()
        branch_change = previous is not None and np.linalg.norm(vector-previous) > np.pi
        result.append(np.full(3, np.nan) if branch_change else vector)
        previous = vector
    return np.degrees(result)


def xyz_figure(series, times, recording_name):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), layout='constrained')
    for name, (positions, rotations) in series.items():
        rv = rotation_vectors_for_plot(rotations)
        for k, axis in enumerate('XYZ'):
            axes[0, k].plot(times, positions[:, k], label=name, linewidth=1.5)
            axes[1, k].plot(times, rv[:, k], label=name, linewidth=1.5)
            axes[0, k].set(title=f'Translation {axis}', ylabel='Position (m)', xlabel='Time from common origin (s)')
            axes[1, k].set(title=f'Rotation vector {axis}', ylabel='Angle (degrees)', xlabel='Time from common origin (s)')
    axes[0, 0].legend(fontsize=7)
    fig.suptitle(f'{recording_name}: synchronized XYZ camera motion\nPrincipal angle-axis; lines break at branch cuts or missing data; visual model references')
    return fig


def safe(values):
    array = np.asarray(values)
    return np.where(np.isfinite(array), array, None).tolist()


def errors(positions, rotations, ref_positions, ref_rotations, mask):
    p = np.linalg.norm(positions-ref_positions, axis=1)
    r = np.array([angle(a, b) if np.isfinite(b).all() else np.nan
                 for a, b in zip(rotations, ref_rotations)])
    selected = mask & np.isfinite(p) & np.isfinite(r)
    return {'position_rmse_m': float(np.sqrt(np.mean(p[selected]**2))),
            'rotation_median_deg': float(np.median(r[selected])),
            'position_disagreement_m': safe(p), 'rotation_disagreement_deg': safe(r),
            'evaluation_positions': int(selected.sum())}


def run(bag, channels_path, gyro_path, sfm_path, fit_path, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    channels, timing, sfm, fit = [json.loads(Path(p).read_text())
        for p in (channels_path, gyro_path, sfm_path, fit_path)]
    calibration = timing['selected_hypothesis']
    recording = Recording(bag)
    pdf = PdfPages(outdir/'motion_figures.pdf')
    try:
        gts, gyro = recording.imu(GYRO)
        ats, accel = recording.imu(ACCEL)
        shift = int(round(calibration['shift_ms']*1e6))
        cgts, cats = gts-shift, ats-shift
        lower, upper = max(gts[0], ats[0], cgts[0], cats[0]), min(gts[-1], ats[-1], cgts[-1], cats[-1])
        selected = [(i, np.asarray(p), c) for i, p, c in zip(channels['rgbd_pose_indices'],
            channels['rgbd_poses_to_component'], channels['rgbd_component_ids'])
            if lower <= recording.color_times[i] <= upper]
        indices = [i for i, _, _ in selected]
        queries = recording.color_times[indices]
        t = (queries-queries[0])/1e9
        origin_pose = next(p for _, p, c in selected if c == 0)
        if selected[0][2] != 0:
            raise ValueError('common interval starts in a disconnected visual component')
        world_to_origin = np.linalg.inv(origin_pose)
        visual = [(world_to_origin@p if c == 0 else np.full((4, 4), np.nan)) for _, p, c in selected]
        ref_p = np.array([p[:3, 3] for p in visual])
        ref_r = np.array([p[:3, :3] for p in visual])
        split_ns = int(recording.color_times[0]+.2*(recording.color_times[-1]-recording.color_times[0]))
        test = queries > split_ns
        paths, inits = {}, {}
        for name, gt, w, at, f, bias, visual_init in [
            ('Raw IMU, zero v0', gts, gyro, ats, accel, np.zeros(3), False),
            ('Calibrated IMU + visual v0', cgts, gyro, cats, accel, np.asarray(calibration['bias']), True),
            ('Five-sample mean IMU + visual v0', cgts, temporal_mean(gyro), cats,
                temporal_mean(accel), np.asarray(calibration['bias']), True),
        ]:
            window = estimate_static_window(gt, w, at, f)
            path, init = inertial_path(gt, w, at, f, queries, recording.color_from_gyro[0],
                recording.color_from_accel[0], recording.color_from_gyro[1], bias,
                gravity_window=window)
            # The first relative RGB-D edge is a calibration observation.
            # Its displacement is never counted as a held-out result.
            edge = next((row for row in channels['comparisons']['raw_gyro']
                if row['source_frame'] == indices[0] and row['target_frame'] in indices
                and 'T_rgbd_source_to_target' in row
                and recording.color_times[row['target_frame']] <= split_ns), None)
            if visual_init and edge is not None:
                endpoint = edge['target_frame']
                endtime = int(recording.color_times[endpoint])
                displacement = np.linalg.inv(np.asarray(edge['T_rgbd_source_to_target']))[:3, 3]
                velocity = velocity_from_visual_displacement(displacement, path[endtime][1],
                    (endtime-int(queries[0]))/1e9)
                path = {q: (r, p+velocity*((q-int(queries[0]))/1e9)) for q, (r, p) in path.items()}
                init.update(initial_velocity_m_s=velocity.tolist(),
                    visual_initialization_frames=[indices[0], endpoint])
            paths[name], inits[name] = path, init
        metrics, series = {}, {}
        for name, path in paths.items():
            rp = np.array([path[int(q)][1] for q in queries])
            rr = np.array([path[int(q)][0] for q in queries])
            metrics[name] = errors(rp, rr, ref_p, ref_r, test)
            series[name] = (rp, rr)
        sfm_poses = {i: np.asarray(p) for i, p in zip(sfm['registered_frames'], sfm['poses_camera_to_world'])}
        sfm_origin = np.linalg.inv(sfm_poses[indices[0]])
        sfm_local = np.array([sfm_origin@sfm_poses[i] if i in sfm_poses else np.full((4, 4), np.nan) for i in indices])
        if sfm['metric_scale'].get('usable'):
            series['Visual SfM + initial depth scale'] = (sfm_local[:, :3, 3], sfm_local[:, :3, :3])
        series['Visual RGB-D reference'] = (ref_p, ref_r)

        def save(fig, name):
            fig.savefig(outdir/(name+'.png'), dpi=160)
            fig.savefig(outdir/(name+'.svg'))
            pdf.savefig(fig)
            plt.close(fig)

        plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': .2})
        fig = xyz_figure(series, t, Path(bag).stem)
        save(fig, 'motion_xyz')

        fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout='constrained')
        for name, metric in metrics.items():
            axes[0].plot(t, metric['position_disagreement_m'], label=name)
            axes[1].plot(t, metric['rotation_disagreement_deg'], label=name)
        for ax in axes:
            ax.axvspan(0, max(0., (split_ns-int(queries[0]))/1e9), alpha=.12, color='gray', label='Calibration prefix')
            ax.set_xlabel('Time from common origin (s)')
        axes[0].set(title='Position disagreement with RGB-D', ylabel='Euclidean difference (m)')
        axes[1].set(title='Orientation disagreement with RGB-D', ylabel='Geodesic angle (degrees)')
        axes[0].legend(fontsize=7)
        fig.suptitle(f'{Path(bag).stem}: free propagation drift before and after initialization')
        save(fig, 'drift_comparison')

        # Fig. 6 replacement: exact same endpoints, scale, coordinate frame
        # and transform direction for both visual and inertial channels.
        comparisons = []
        visual_by_frame = {i: p for i, p in zip(indices, visual)}
        for row in channels['comparisons']['raw_gyro']:
            a, b = row['source_frame'], row['target_frame']
            if a not in indices or b not in indices or 'T_rgbd_source_to_target' not in row:
                continue
            entry = {'source_frame': a, 'target_frame': b,
                'start_time_s': float((recording.color_times[a]-queries[0])/1e9),
                'duration_s': float((recording.color_times[b]-recording.color_times[a])/1e9),
                'inlier_rate': row['rgbd']['inlier_rate'], 'channels': {}}
            rgbd = np.asarray(row['T_rgbd_source_to_target'])
            entry['channels']['Visual RGB-D reference'] = {'translation_m': rgbd[:3, 3].tolist(),
                'rotation_vector_deg': np.degrees(cv2.Rodrigues(rgbd[:3, :3])[0].ravel()).tolist()}
            for name, path in paths.items():
                r, p = relative_motion(path[int(recording.color_times[a])], path[int(recording.color_times[b])])
                entry['channels'][name] = {'translation_m': p.tolist(),
                    'rotation_vector_deg': np.degrees(cv2.Rodrigues(r)[0].ravel()).tolist()}
            if a in sfm_poses and b in sfm_poses and sfm['metric_scale'].get('usable'):
                pose = np.linalg.inv(sfm_poses[b])@sfm_poses[a]
                entry['channels']['Visual SfM + initial depth scale'] = {'translation_m': pose[:3, 3].tolist(),
                    'rotation_vector_deg': np.degrees(cv2.Rodrigues(pose[:3, :3])[0].ravel()).tolist()}
            comparisons.append(entry)
        fig, axes = plt.subplots(2, 3, figsize=(14, 8), layout='constrained', sharey='row')
        for name in series:
            rows = [r for r in comparisons if name in r['channels']]
            for k, axis in enumerate('XYZ'):
                axes[0, k].plot([r['start_time_s'] for r in rows],
                    [r['channels'][name]['translation_m'][k] for r in rows], label=name)
                axes[1, k].plot([r['start_time_s'] for r in rows],
                    [r['channels'][name]['rotation_vector_deg'][k] for r in rows], label=name)
                axes[0, k].set(title=f'Relative translation {axis}', ylabel='Translation (m)')
                axes[1, k].set(title=f'Relative rotation {axis}', ylabel='Angle (degrees)', xlabel='Source-frame time (s)')
        axes[0, 0].legend(fontsize=7)
        fig.suptitle(f'{Path(bag).stem}: corrected Fig. 6 comparison\nSource-to-target camera transforms on identical recorded intervals')
        save(fig, 'fig6_common_intervals')

        # Test drift resets using PREVIOUS camera measurements only.
        reset_errors, cv_errors, reset_times, reset_rotation_errors = [], [], [], []
        gravity = np.asarray(inits['Calibrated IMU + visual v0']['gravity_vector'])
        bias = np.asarray(calibration['bias'])
        lever = recording.color_from_gyro[1]
        for k in range(2, len(indices)):
            if not np.isfinite(ref_p[k-2:k+1]).all() or queries[k-1] < split_ns:
                continue
            q0, q1 = queries[k-1:k+1]
            dt = (q1-q0)/1e9
            visual_v = (ref_p[k-1]-ref_p[k-2])/((queries[k-1]-queries[k-2])/1e9)
            omega = np.array([np.interp(q0, cgts, gyro[:, j]-bias[j]) for j in range(3)]) @ recording.color_from_gyro[0].T
            imu_v = visual_v+ref_r[k-1]@np.cross(omega, lever)
            raw, _ = integrals(cgts, gyro, cats, accel, np.array([q0, q1]),
                recording.color_from_gyro[0], recording.color_from_accel[0], ref_r[k-1], bias)[int(q1)]
            delta_r = rotations_at(cgts, gyro, np.array([q1]), recording.color_from_gyro[0], bias, q0)[int(q1)]
            end_r = ref_r[k-1]@delta_r
            prediction = ref_p[k-1]+imu_v*dt+raw+.5*gravity*dt*dt+ref_r[k-1]@lever-end_r@lever
            baseline = ref_p[k-1]+visual_v*dt
            reset_errors.append(float(np.linalg.norm(prediction-ref_p[k])))
            cv_errors.append(float(np.linalg.norm(baseline-ref_p[k])))
            reset_rotation_errors.append(angle(end_r, ref_r[k]))
            reset_times.append(float(t[k]))
        reset_rmse = float(np.sqrt(np.mean(np.square(reset_errors)))) if reset_errors else None
        cv_rmse = float(np.sqrt(np.mean(np.square(cv_errors)))) if cv_errors else None
        reset = {'evaluation_windows': len(reset_errors), 'imu_prediction_rmse_m': reset_rmse,
            'constant_velocity_rmse_m': cv_rmse,
            'beats_constant_velocity': bool(reset_errors and reset_rmse < cv_rmse),
            'endpoint_visual_position_used_in_prediction': False,
            'visual_resets_are_not_independent_imu_accuracy': True,
            'prediction_times_s': reset_times, 'imu_error_m': reset_errors, 'cv_error_m': cv_errors}
        fig, ax = plt.subplots(figsize=(12, 5), layout='constrained')
        ax.plot(reset_times, reset_errors, label='IMU prediction after previous visual reset')
        ax.plot(reset_times, cv_errors, label='Constant velocity from same previous visual poses')
        ax.set(title=f'{Path(bag).stem}: drift compensation checked before the next visual update',
            xlabel='Time from common origin (s)', ylabel='Prediction disagreement with next RGB-D pose (m)')
        ax.legend()
        save(fig, 'visual_reset_validation')

        semantics = []
        for start in range(0, len(recording.color_times)-1, 25):
            end = min(start+25, len(recording.color_times)-1)
            frames = [recording.image(COLOR, i) for i in range(start, end+1)]
            grays = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in frames]
            _, _, _, stats = next(flow_pairs(grays, window=25, semantic_images=frames,
                reject_full_frame_horizon=False))
            stats.update(start_frame=start, end_frame=end)
            semantics.append(stats)
        semantic_total = sum(s['semantic_rejected'] for s in semantics)
        summary = {'recording': Path(bag).name, 'common_camera_frames': len(indices),
            'duration_s': float(t[-1]), 'gyro_shift_ms': calibration['shift_ms'],
            'heldout_gyro_vs_rgbd_median_deg': timing['heldout_calibrated_median_deg'],
            'raw_position_rmse_m': metrics['Raw IMU, zero v0']['position_rmse_m'],
            'initialized_position_rmse_m': metrics['Calibrated IMU + visual v0']['position_rmse_m'],
            'mean_imu_position_rmse_m': metrics['Five-sample mean IMU + visual v0']['position_rmse_m'],
            'constrained_position_rmse_m': fit['heldout_initialized_position_rmse_m'],
            'constrained_baseline_rmse_m': fit['heldout_constant_velocity_rmse_m'],
            'inertial_translation_fit_usable': fit['initialization_usable'],
            'visual_reset_prediction_rmse_m': reset_rmse, 'visual_reset_baseline_rmse_m': cv_rmse,
            'visual_reset_beats_baseline': reset['beats_constant_velocity'],
            'semantic_windows': len(semantics), 'semantic_additional_rejections': semantic_total,
            'pnp_inlier_rate_median': float(np.median([r['inlier_rate'] for r in comparisons])),
            'metric_ground_truth_available': False}
        result = {'summary': summary, 'initializations': inits, 'free_propagation_metrics': metrics,
            'visual_reset_check': reset, 'frame_indices': indices, 'times_s': t.tolist(),
            'camera_positions_m': {name: safe(p) for name, (p, _) in series.items()},
            'camera_rotations': {name: safe(r) for name, (_, r) in series.items()},
            'interval_comparisons': comparisons, 'semantic_windows': semantics,
            'definitions': {'frame': 'color camera at common integration origin',
                'relative_transform': 'source camera coordinates to target camera coordinates',
                'translation_units': 'metres; SfM scale from initial registered depth',
                'rotation': 'principal Rodrigues rotation-vector components in degrees; plot line breaks at branch cuts; not Euler angles',
                'imu_average': 'centered five-sample temporal arithmetic mean on each sensor axis; offline',
                'gyro_and_accel_share_clock_shift': True,
                'static_window_selection': 'offline low-motion candidate; stationary motion not verified',
                'free_translation_drift_correction': 'gyro calibration plus one initial visual displacement only',
                'visual_reset': 'previous pose/orientation and preceding visual velocity; current endpoint reserved for scoring'},
            'inputs': {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                for p in (channels_path, gyro_path, sfm_path, fit_path)},
            'recording': {'path': str(Path(bag).resolve()), 'size': Path(bag).stat().st_size,
                'mtime_ns': Path(bag).stat().st_mtime_ns}}
        (outdir/'motion_report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        with (outdir/'interval_motion.csv').open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['source_frame', 'target_frame', 'start_s', 'duration_s', 'channel',
                             'tx_m', 'ty_m', 'tz_m', 'rx_deg', 'ry_deg', 'rz_deg'])
            for row in comparisons:
                for name, value in row['channels'].items():
                    writer.writerow([row['source_frame'], row['target_frame'], row['start_time_s'],
                        row['duration_s'], name, *value['translation_m'], *value['rotation_vector_deg']])
        print(json.dumps(summary), flush=True)
        return result
    finally:
        pdf.close()
        recording.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'channels', 'gyro-fit', 'sfm', 'fit', 'outdir'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    run(a.bag, a.channels, a.gyro_fit, a.sfm, a.fit, a.outdir)
