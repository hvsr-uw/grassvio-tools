#!/usr/bin/env python3
"""Measure archived controls without assuming that their camera was stationary.

Keep unavailable RGB-D pairs explicit. Compare temporal and colour-cue
filtering on spatially held-out tracks; no physical wind labels are inferred.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'stitch-map'))
from recording_camera import Recording, DepthPairingError, COLOR, GYRO, ACCEL, rays
from motion_channels import angle, estimate_pnp, sample_cloud, estimate_static_window, inertial_path
from gyro_calibration import samples_between, integrate_rotation
from geometry import fit_homography, apply_transform
from window_motion import track_window, temporal_filter
from heldout_diagnostics import evaluate_heldout_window
from semantic_cues import vegetation_likelihood, sample_likelihood, semantic_motion_keep


def compare_filters(history, image, threshold_px=1.5):
    """Fit each selection policy only on training tiles, then score other tiles."""
    def semantic_scorer(tracks, models, threshold):
        keep, scores = temporal_filter(tracks, models, threshold)
        vegetation = sample_likelihood(vegetation_likelihood(image), tracks[0])
        semantic_keep, _ = semantic_motion_keep(vegetation, scores,
            vegetation_threshold=.35, residual_threshold_px=threshold*.75)
        return keep & semantic_keep, scores

    temporal = evaluate_heldout_window(history, None, threshold_px,
        temporal_filter, fit_homography, apply_transform)
    semantic = evaluate_heldout_window(history, None, threshold_px,
        semantic_scorer, fit_homography, apply_transform)
    keep, scores = temporal_filter(history, None, threshold_px)
    vegetation = sample_likelihood(vegetation_likelihood(image), history[0])
    semantic_keep, _ = semantic_motion_keep(vegetation, scores,
        vegetation_threshold=.35, residual_threshold_px=threshold_px*.75)
    return keep, {
        'complete_tracks': int(history.shape[1]),
        'temporal_kept': int(keep.sum()),
        'semantic_extra_rejections': int(np.count_nonzero(keep & ~semantic_keep)),
        'vegetation_candidate_tracks': int(np.count_nonzero(vegetation >= .35)),
        'median_observed_track_motion_px': (float(np.median(np.linalg.norm(history[-1]-history[0], axis=1)))
            if history.shape[1] else None),
        'temporal': temporal, 'temporal_and_colour': semantic,
        'independent_camera_prior_available': False, 'wind_cause_verified': False}


def measure_rgbd(recording, source_frame, history, keep):
    try:
        cloud, offset = recording.cloud_for_color(source_frame)
    except DepthPairingError:
        return None, {'status': 'unavailable_depth_pair_over_20_ms'}
    transform, stats = estimate_pnp(sample_cloud(cloud, history[0, keep]),
        rays(history[-1, keep], recording.color),
        (recording.color['fx']+recording.color['fy'])*.5)
    return transform, dict(stats, rgb_depth_offset_ms=offset)


def inspect(bag, outdir, window):
    outdir.mkdir(parents=True, exist_ok=True)
    recording = Recording(bag)
    try:
        gts, gyro = recording.imu(GYRO)
        ats, accel = recording.imu(ACCEL)
        static = estimate_static_window(gts, gyro, ats, accel)
        pairing = []
        for index in range(len(recording.color_times)):
            offset = float(np.min(np.abs(recording.depth_times-recording.color_times[index]))/1e6)
            pairing.append(offset)
        frames = [recording.image(COLOR, i) for i in range(len(recording.color_times))]
        grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
        measured = []
        # Every interval remains in the report, even if its depth or IMU is missing.
        for start in range(0, len(frames)-1, window):
            end = min(start+window, len(frames)-1)
            history, detected = track_window(grays[start:end+1])
            keep, filters = compare_filters(history, frames[start])
            transform, rgbd = measure_rgbd(recording, start, history, keep)
            row = {'source_frame': start, 'target_frame': end, 'features_detected': detected,
                'filters': filters, 'rgbd': rgbd}
            if transform is not None:
                row['T_rgbd_source_to_target'] = transform.tolist()
                row['rgbd_rotation_deg'] = angle(np.eye(3), transform[:3, :3])
                row['rgbd_translation_norm_m'] = float(np.linalg.norm(transform[:3, 3]))
            a, b = recording.color_times[[start, end]]
            if a >= gts[0] and b <= gts[-1]:
                rotation = integrate_rotation(samples_between(gts, gyro, a, b), np.zeros(3),
                    recording.color_from_gyro[0])
                row['gyro_rotation_deg'] = angle(np.eye(3), rotation)
                if transform is not None:
                    row['gyro_vs_rgbd_deg'] = angle(rotation, transform[:3, :3])
            else:
                row['gyro_status'] = 'outside_sensor_coverage'
            measured.append(row)
            print(f'{bag.name}: {start}->{end}: {rgbd["status"]}', flush=True)

        indices = np.flatnonzero((recording.color_times >= max(gts[0], ats[0])) &
            (recording.color_times <= min(gts[-1], ats[-1])))
        queries = recording.color_times[indices]
        path, initialization = inertial_path(gts, gyro, ats, accel, queries,
            recording.color_from_gyro[0], recording.color_from_accel[0], recording.color_from_gyro[1],
            np.zeros(3), gravity_window=static)
        for row in measured:
            a, b = recording.color_times[[row['source_frame'], row['target_frame']]]
            if int(a) in path and int(b) in path:
                ra, pa = path[int(a)]
                rb, pb = path[int(b)]
                row['raw_imu_carried_translation_m'] = (rb.T@(pa-pb)).tolist()
                if 'T_rgbd_source_to_target' in row:
                    row['raw_imu_vs_rgbd_translation_m'] = float(np.linalg.norm(
                        rb.T@(pa-pb)-np.array(row['T_rgbd_source_to_target'])[:3, 3]))

        def median(values):
            values = [v for v in values if v is not None]
            return float(np.median(values)) if values else None
        def heldout(policy, key):
            return median([r['filters'][policy][key] for r in measured])
        summary = {'recording': bag.name, 'window_frames': window,
            'frames': len(frames), 'duration_s': float((recording.color_times[-1]-recording.color_times[0])/1e9),
            'windows': len(measured), 'rgbd_windows': sum('T_rgbd_source_to_target' in r for r in measured),
            'depth_pairs_over_20_ms': sum(v > 20 for v in pairing),
            'first_depth_offset_ms': pairing[0], 'max_depth_offset_ms': max(pairing),
            'quiet_window_mean_gyro_rad_s': static['mean_gyro_norm'],
            'quiet_window_is_low_motion_candidate': static['low_motion_candidate'],
            'rgbd_rotation_median_deg': median([r.get('rgbd_rotation_deg') for r in measured]),
            'rgbd_translation_median_m': median([r.get('rgbd_translation_norm_m') for r in measured]),
            'gyro_rotation_median_deg': median([r.get('gyro_rotation_deg') for r in measured]),
            'gyro_vs_rgbd_median_deg': median([r.get('gyro_vs_rgbd_deg') for r in measured]),
            'unfiltered_heldout_median_px': heldout('temporal', 'heldout_unfiltered_median_error_px'),
            'temporal_heldout_median_px': heldout('temporal', 'heldout_median_error_px'),
            'colour_heldout_median_px': heldout('temporal_and_colour', 'heldout_median_error_px'),
            'semantic_extra_rejections': sum(r['filters']['semantic_extra_rejections'] for r in measured),
            'vegetation_candidate_tracks': sum(r['filters']['vegetation_candidate_tracks'] for r in measured),
            'camera_stationary_independently_verified': False, 'wind_cause_verified': False,
            'metric_ground_truth_available': False}
        with bag.open('rb') as handle:
            digest = hashlib.file_digest(handle, 'sha256').hexdigest()
        report = {'summary': summary, 'sensor_initialization': initialization,
            'assumptions': {'depth_pairing_limit_ms': 20., 'imu_initial_velocity': 'assumed_zero_not_verified',
                'gyro_bias_rad_s': [0., 0., 0.], 'clock_shift_ms': 0.,
                'gravity': 'offline quiet-window heuristic, not verified static calibration',
                'metric_reference': 'RGB-D fitted to temporally selected tracks, not ground truth',
                'filter_evaluation': 'each policy fits training tiles, then scores identical held-out tiles',
                'heldout_score': 'all held-out tracks, including moving objects; image consistency only',
                'fig6_original_provenance_established': False},
            'source': {'recording': bag.name, 'size_bytes': bag.stat().st_size,
                'sha256': digest},
            'depth_pair_offsets_ms': pairing, 'windows': measured}
        (outdir/'controls.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        return report
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--names', nargs='+', required=True)
    parser.add_argument('--window', type=int, default=25)
    args = parser.parse_args()
    if args.window < 1:
        parser.error('--window must be positive')
    bags = [args.input_dir/name for name in args.names]
    if any(not b.is_file() for b in bags):
        parser.error('one or more requested recordings are missing')
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    reports = [inspect(b, args.output_dir/b.stem, args.window) for b in bags]
    rows = [r['summary'] for r in reports]
    (args.output_dir/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')
    with (args.output_dir/'summary.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(rows, indent=2), flush=True)


if __name__ == '__main__':
    main()
