#!/usr/bin/env python3
"""Compare later SfM, RGB-D and gyro rotations without inventing IMU translation."""
import argparse
import json
from pathlib import Path

import numpy as np

from gyro_calibration import samples_between, integrate_rotation
from motion_channels import angle
from recording_camera import Recording, GYRO
from validation_cache import fingerprint, completed


def run(bag, sfm_path, channels_path, gyro_path, outpath):
    source_fingerprint = fingerprint(bag, [sfm_path, channels_path, gyro_path],
        ['multiview_channel_check', 'gyro_calibration', 'motion_channels', 'recording_camera'])
    cached = completed(outpath, source_fingerprint)
    if cached is not None:
        print(f'{Path(bag).name}: cached multiview comparison', flush=True)
        return cached
    sfm = json.loads(Path(sfm_path).read_text())
    channels = json.loads(Path(channels_path).read_text())
    calibration = json.loads(Path(gyro_path).read_text())['selected_hypothesis']
    poses = {i: np.asarray(p) for i, p in zip(sfm['registered_frames'], sfm['poses_camera_to_world'])}
    recording = Recording(bag)
    try:
        times, gyro = recording.imu(GYRO)
        split = recording.color_times[0]+.2*(recording.color_times[-1]-recording.color_times[0])
        records = []
        for row in channels['comparisons']['raw_gyro']:
            start, end = row['source_frame'], row['target_frame']
            if start not in poses or end not in poses:
                continue
            a, b = recording.color_times[[start, end]]+calibration['shift_ms']*1e6
            # Preserve exactly the prefix/later split used in timing calibration.
            if recording.color_times[start] < split or a < times[0] or b > times[-1]:
                continue
            sfm_relative = np.linalg.inv(poses[end])@poses[start]
            inertial = integrate_rotation(samples_between(times, gyro, a, b), np.array(calibration['bias']),
                                          recording.color_from_gyro[0])
            result = {'source_frame': start, 'target_frame': end,
                      'gyro_vs_sfm_rotation_deg': angle(inertial, sfm_relative[:3, :3])}
            if 'T_rgbd_source_to_target' in row:
                rgbd = np.asarray(row['T_rgbd_source_to_target'])
                result['rgbd_vs_sfm_rotation_deg'] = angle(rgbd[:3, :3], sfm_relative[:3, :3])
                if sfm['metric_scale'].get('usable'):
                    result['rgbd_vs_sfm_translation_vector_error_m'] = float(np.linalg.norm(rgbd[:3, 3]-sfm_relative[:3, 3]))
                    if min(np.linalg.norm(rgbd[:3, 3]), np.linalg.norm(sfm_relative[:3, 3])) > .005:
                        cosine = np.dot(rgbd[:3, 3], sfm_relative[:3, 3])/(np.linalg.norm(rgbd[:3, 3])*np.linalg.norm(sfm_relative[:3, 3]))
                        result['rgbd_vs_sfm_translation_direction_deg'] = float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))
            records.append(result)
        keys = ['gyro_vs_sfm_rotation_deg', 'rgbd_vs_sfm_rotation_deg',
                'rgbd_vs_sfm_translation_vector_error_m', 'rgbd_vs_sfm_translation_direction_deg']
        summary = {}
        for key in keys:
            values = [r[key] for r in records if key in r]
            summary[key] = {'count': len(values), 'median': float(np.median(values)) if values else None,
                            'p90': float(np.percentile(values, 90)) if values else None}
        result = {'recording': Path(bag).name, 'source_fingerprint': source_fingerprint,
                  'summary': summary, 'comparisons': records, 'metric_ground_truth_available': False,
                  'sfm_scale_source': 'first_frame_registered_depth_only',
                  'independent_imu_translation_validated': False, 'three_channel_agreement_established': False,
                  'sfm_camera_fit_uses_all_registered_images': True}
        Path(outpath).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(summary), flush=True)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--sfm', type=Path, required=True)
    parser.add_argument('--channels', type=Path, required=True)
    parser.add_argument('--gyro-fit', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.sfm, args.channels, args.gyro_fit, args.out)


if __name__ == '__main__':
    main()
