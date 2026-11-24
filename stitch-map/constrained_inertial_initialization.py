#!/usr/bin/env python3
"""Test a gravity-magnitude-constrained prefix fit without hiding observability.

The fixed gravity magnitude and accelerometer-bias prior are assumptions.
The data-only Jacobian excludes that prior when checking rank. No later
camera position changes the fitted parameters or fixes propagated drift.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from recording_camera import Recording, GYRO, ACCEL
from inertial_initialization import integrals, solve_initialization
from validation_cache import fingerprint, completed


GRAVITY = 9.80665


def gravity_vector(angles):
    azimuth, elevation = angles
    return GRAVITY*np.array([np.cos(elevation)*np.sin(azimuth), np.sin(elevation),
                             np.cos(elevation)*np.cos(azimuth)])


def positions(parameters, elapsed, raw, bias_integrals):
    return (raw + elapsed[:, None]*parameters[:3]
            - np.einsum('nij,j->ni', bias_integrals, parameters[3:6])
            + .5*elapsed[:, None]**2*gravity_vector(parameters[6:]))


def fit_prefix(elapsed, raw, bias_integrals, reference):
    linear, _ = solve_initialization(elapsed, raw, bias_integrals, reference)
    gravity = linear[6:9]
    if np.linalg.norm(gravity) < 1.:
        gravity = np.array([0., GRAVITY, .01])
    initial = np.r_[np.clip(linear[:6], -.8, .8), np.arctan2(gravity[0], gravity[2]),
                    np.arcsin(np.clip(gravity[1]/np.linalg.norm(gravity), -.999999, .999999))]

    def data_residual(parameters):
        return ((positions(parameters, elapsed, raw, bias_integrals)-reference)/.02).ravel()

    def residual(parameters):
        return np.r_[data_residual(parameters), parameters[3:6]/.25]

    lower = [-3.]*3+[-1.]*3+[-np.pi, -np.pi/2+.0001]
    upper = [3.]*3+[1.]*3+[np.pi, np.pi/2-.0001]
    initial = np.clip(initial, np.array(lower)+1e-8, np.array(upper)-1e-8)
    solved = least_squares(residual, initial, bounds=(lower, upper), loss='soft_l1', max_nfev=200)
    # Numerical data-only Jacobian in explicit, reported parameter units.
    scale = np.array([1.]*3+[.1]*3+[.1]*2)
    base = data_residual(solved.x)
    jacobian = np.column_stack([(data_residual(solved.x+np.eye(8)[k]*scale[k]*1e-5)-base)/1e-5
                                for k in range(8)])
    singular = np.linalg.svd(jacobian, compute_uv=False)
    rank = int(np.sum(singular > singular[0]*1e-4)) if len(singular) else 0
    condition = float(singular[0]/singular[-1]) if len(singular) == 8 and singular[-1] > 1e-12 else None
    return solved.x, {'optimizer_converged': bool(solved.success), 'data_rank': rank, 'unknowns': 8,
        'data_scaled_condition_number': condition, 'data_singular_values': singular.tolist(),
        'rank_relative_tolerance': 1e-4, 'jacobian_parameter_scales': scale.tolist(),
        'bias_prior_sigma_m_s2': .25, 'position_residual_scale_m': .02,
        'velocity_m_s': solved.x[:3].tolist(), 'accel_bias_m_s2': solved.x[3:6].tolist(),
        'gravity_m_s2': gravity_vector(solved.x[6:]).tolist(), 'gravity_magnitude_assumed_m_s2': GRAVITY,
        'physically_plausible': bool(np.linalg.norm(solved.x[3:6]) < 1.),
        'observable_from_data': rank == 8 and condition is not None and condition < 1e4}


def run(bag, channels_path, gyro_fit_path, outpath):
    source_fingerprint = fingerprint(bag, [channels_path, gyro_fit_path],
        ['constrained_inertial_initialization', 'inertial_initialization', 'recording_camera'])
    cached = completed(outpath, source_fingerprint)
    if cached is not None:
        print(f'{Path(bag).name}: cached constrained initialization', flush=True)
        return cached
    channel = json.loads(Path(channels_path).read_text())
    calibration = json.loads(Path(gyro_fit_path).read_text())['selected_hypothesis']
    recording = Recording(bag)
    try:
        gts, gyro = recording.imu(GYRO)
        ats, accel = recording.imu(ACCEL)
        shift = calibration['shift_ms']*1e6
        gts, ats = gts-shift, ats-shift
        selected = [(i, np.asarray(p)) for i, c, p in zip(channel['rgbd_pose_indices'], channel['rgbd_component_ids'],
                    channel['rgbd_poses_to_component']) if c == 0 and max(gts[0], ats[0]) <= recording.color_times[i] <= min(gts[-1], ats[-1])]
        indices = [i for i, _ in selected]
        poses = [p for _, p in selected]
        queries = recording.color_times[indices]
        values = integrals(gts, gyro, ats, accel, queries, recording.color_from_gyro[0], recording.color_from_accel[0],
                           poses[0][:3, :3], np.array(calibration['bias']))
        raw = np.array([values[int(t)][0] for t in queries])
        bias_integrals = np.array([values[int(t)][1] for t in queries])
        elapsed = (queries-queries[0])/1e9
        lever = recording.color_from_gyro[1]
        reference = np.array([p[:3, 3]+p[:3, :3]@lever for p in poses])
        reference -= reference[0]
        split = recording.color_times[0]+.2*(recording.color_times[-1]-recording.color_times[0])
        train = (queries <= split) & (elapsed > 0)
        test = queries > split
        if train.sum() < 2 or test.sum() < 3:
            raise ValueError('insufficient prefix/later positions')
        parameters, diagnostics = fit_prefix(elapsed[train], raw[train], bias_integrals[train], reference[train])
        prediction = positions(parameters, elapsed, raw, bias_integrals)
        velocity = np.linalg.lstsq(elapsed[train, None], reference[train], rcond=None)[0][0]
        baseline = elapsed[:, None]*velocity
        def rmse(value):
            return float(np.sqrt(np.mean(np.sum((value[test]-reference[test])**2, axis=1))))
        error, baseline_error = rmse(prediction), rmse(baseline)
        usable = (diagnostics['observable_from_data'] and diagnostics['optimizer_converged']
                  and diagnostics['physically_plausible'] and error < baseline_error)
        result = {'recording': Path(bag).name, 'source_fingerprint': source_fingerprint,
                  'training_fraction': .2, 'training_positions': int(train.sum()),
                  'validation_positions': int(test.sum()), 'fit': diagnostics,
                  'heldout_initialized_position_rmse_m': error, 'heldout_constant_velocity_rmse_m': baseline_error,
                  'initialization_usable': bool(usable), 'metric_ground_truth_available': False,
                  'shared_accel_and_gyro_clock_shift_assumed': True,
                  'later_camera_positions_used_to_fit': False, 'rejected_fit_applied_to_trajectory': False}
        Path(outpath).parent.mkdir(parents=True, exist_ok=True)
        Path(outpath).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(result), flush=True)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--channels', type=Path, required=True)
    parser.add_argument('--gyro-fit', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.channels, args.gyro_fit, args.out)


if __name__ == '__main__':
    main()
