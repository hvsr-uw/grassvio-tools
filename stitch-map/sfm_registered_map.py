#!/usr/bin/env python3
"""Render measured depth with explicitly labelled, initial-depth-scaled SfM poses."""
import argparse
import json
from pathlib import Path

import numpy as np

from registered_map import render
from validation_cache import fingerprint, completed


def poses_for_map(sfm):
    if sfm['pose_units'] != 'metres_from_initial_depth' or not sfm['metric_scale'].get('usable'):
        raise ValueError('SfM map needs a supported initial-depth metric scale')
    if len(sfm['registered_frames']) != len(sfm['poses_camera_to_world']):
        raise ValueError('SfM pose/frame count mismatch')
    poses = dict(zip(sfm['registered_frames'], sfm['poses_camera_to_world']))
    if 0 not in poses or not np.allclose(poses[0], np.eye(4), atol=1e-8):
        raise ValueError('SfM map needs the first camera as its fixed world frame')
    return {'camera_pose_source': 'image_only_multiview_SfM_initial_depth_scale',
            'rgbd_pose_indices': sfm['selected_frames'],
            'rgbd_component_ids': [0 if i in poses else -1 for i in sfm['selected_frames']],
            'rgbd_poses_to_component': [poses.get(i, np.eye(4).tolist()) for i in sfm['selected_frames']],
            'depth_registration': 'recorded_reference_to_stream_extrinsics_and_sdk_distortion',
            'metric_ground_truth_available': False}


def run(bag, sfm_path, outdir):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    source_fingerprint = fingerprint(bag, [sfm_path], ['sfm_registered_map', 'registered_map', 'recording_camera'])
    cached = completed(outdir/'registered_map.json', source_fingerprint, [outdir/'registered_map.png'])
    if cached is not None:
        print(f'{Path(bag).name}: cached SfM depth map', flush=True)
        return cached
    sfm = json.loads(Path(sfm_path).read_text())
    channels = poses_for_map(sfm)
    inputs = outdir/'poses.json'
    inputs.write_text(json.dumps(channels, indent=2)+'\n')
    result = render(bag, inputs, outdir/'registered_map.png')
    result['source_fingerprint'] = source_fingerprint
    (outdir/'registered_map.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--sfm', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.sfm, args.outdir)


if __name__ == '__main__':
    main()
