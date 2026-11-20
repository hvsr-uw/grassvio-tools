#!/usr/bin/env python3
"""Widen the prefix-only gyro timing search without selecting on later errors."""
import argparse
import json
from pathlib import Path

import numpy as np

from gyro_calibration import samples_between, integrate_rotation, fit_bias
from motion_channels import angle
from recording_camera import Recording, GYRO
from validation_cache import fingerprint, completed


def search_timing(training, sample, rotation_cg, coarse_shifts=None):
    shifts = np.arange(-200., 201., 25.) if coarse_shifts is None else np.asarray(coarse_shifts)
    references = [np.asarray(row['T_rgbd_source_to_target'])[:3, :3] for row in training]
    hypotheses = {}

    def evaluate(shift):
        shift = float(shift)
        if shift not in hypotheses:
            bias, error = fit_bias([sample(row, shift) for row in training], references, rotation_cg)
            hypotheses[shift] = {'shift_ms': shift, 'bias': bias.tolist(), 'training_rmse_rad': error}
        return hypotheses[shift]

    coarse = min((evaluate(shift) for shift in shifts), key=lambda row: row['training_rmse_rad'])
    for shift in np.arange(max(shifts.min(), coarse['shift_ms']-25),
                           min(shifts.max(), coarse['shift_ms']+25)+.1, 2.5):
        evaluate(shift)
    best = min(hypotheses.values(), key=lambda row: row['training_rmse_rad'])
    return best, sorted(hypotheses.values(), key=lambda row: row['shift_ms'])


def run(bag, channels_path, outpath):
    source_fingerprint = fingerprint(bag, [channels_path],
        ['gyro_timing_refinement', 'gyro_calibration', 'motion_channels', 'recording_camera'])
    cached = completed(outpath, source_fingerprint)
    if cached is not None:
        print(f'{Path(bag).name}: cached timing refinement', flush=True)
        return cached
    channel = json.loads(Path(channels_path).read_text())
    recording = Recording(bag)
    try:
        times, gyro = recording.imu(GYRO)
        rotation_cg = recording.color_from_gyro[0]
        first, last = recording.color_times[[0, -1]]
        split = first + .2*(last-first)
        margin = 200_000_000
        candidates = [row for row in channel['comparisons']['raw_gyro']
                      if 'T_rgbd_source_to_target' in row
                      and recording.color_times[row['source_frame']] >= times[0]+margin
                      and recording.color_times[row['target_frame']] <= times[-1]-margin]
        training = [row for row in candidates if recording.color_times[row['target_frame']] <= split]
        validation = [row for row in candidates if recording.color_times[row['source_frame']] >= split]
        if len(training) < 2 or len(validation) < 3:
            raise ValueError('insufficient disjoint prefix/later rotation windows')

        def sample(row, shift):
            a, b = recording.color_times[[row['source_frame'], row['target_frame']]] + shift*1e6
            return samples_between(times, gyro, a, b)

        best, hypotheses = search_timing(training, sample, rotation_cg)
        scores = []
        for row in validation:
            reference = np.asarray(row['T_rgbd_source_to_target'])[:3, :3]
            def error(shift, bias):
                return angle(integrate_rotation(sample(row, shift), bias, rotation_cg), reference)
            scores.append({'source_frame': row['source_frame'], 'target_frame': row['target_frame'],
                           'raw_error_deg': error(0., np.zeros(3)),
                           'calibrated_error_deg': error(best['shift_ms'], np.array(best['bias']))})
        result = {'recording': Path(bag).name, 'source_fingerprint': source_fingerprint, 'training_fraction': .2,
                  'training_windows': len(training), 'validation_windows': len(validation),
                  'search_range_ms': [-200., 200.], 'fine_step_ms': 2.5,
                  'selected_hypothesis': best, 'training_hypotheses': hypotheses,
                  'selected_at_search_boundary': abs(best['shift_ms']) == 200.,
                  'clock_shift_definition': 'gyro query time equals camera time plus shift',
                  'timing_is_hypothesis_not_hardware_calibration': True,
                  'metric_ground_truth_available': False,
                  'heldout_raw_median_deg': float(np.median([r['raw_error_deg'] for r in scores])),
                  'heldout_calibrated_median_deg': float(np.median([r['calibrated_error_deg'] for r in scores])),
                  'heldout_calibrated_p90_deg': float(np.percentile([r['calibrated_error_deg'] for r in scores], 90)),
                  'validation': scores}
        Path(outpath).parent.mkdir(parents=True, exist_ok=True)
        Path(outpath).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('training_hypotheses', 'validation')}), flush=True)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--channels', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.channels, args.out)


if __name__ == '__main__':
    main()
