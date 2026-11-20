#!/usr/bin/env python3
"""Retry unsupported SfM intervals with five-frame support and closer keyframes.

This is a different temporal-window setting, not evidence that the original
25-frame setting handled the fast motion. Pose fitting still uses no depth.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from multiview_sfm import augment_flow, build_tracks, reconstruct
from recording_camera import Recording, COLOR, rays
from relocalization import filtered_sequence_features
from motion_channels import sample_cloud
from validation_cache import fingerprint, completed


def densify_gaps(selected, missing, step=5):
    if step < 1:
        raise ValueError('positive frame step required')
    selected = sorted(set(selected))
    dense = set(selected)
    for i, frame in enumerate(selected):
        if frame in missing:
            dense.update(range(selected[max(i-1, 0)], selected[min(i+1, len(selected)-1)]+1, step))
    return sorted(dense)


def run(bag, coarse_path, outdir):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    source_fingerprint = fingerprint(bag, [coarse_path], ['adaptive_sfm', 'multiview_sfm', 'recording_camera',
        'window_motion', 'relocalization', 'geometry', 'sfm_keyframes', 'motion_channels', 'heldout_diagnostics', 'stitch_map'])
    cached = completed(outdir/'reconstruction.json', source_fingerprint, [outdir/'structure.npz'])
    if cached is not None:
        print(f'{Path(bag).name}: cached adaptive SfM', flush=True)
        return cached
    coarse = json.loads(Path(coarse_path).read_text())
    indices = densify_gaps(coarse['selected_frames'], set(coarse['unregistered_frames']))
    recording = Recording(bag)
    try:
        features = []
        cache = outdir/'features.npz'
        if cache.exists():
            with np.load(cache) as saved:
                if str(saved['fingerprint']) == source_fingerprint:
                    features = [{'pixels': saved[f'p{i}'], 'xy': saved[f'x{i}'], 'descriptors': saved[f'd{i}']}
                                for i in range(len(indices))]
        if not features:
            arrays = {'fingerprint': np.array(source_fingerprint)}
            for i, index in enumerate(indices):
                window = (range(index, index+6) if index+5 < len(recording.color_times)
                          else range(index, max(-1, index-6), -1))
                grays = [cv2.cvtColor(recording.image(COLOR, f), cv2.COLOR_BGR2GRAY) for f in window]
                keys, desc = filtered_sequence_features(grays)
                pixels = np.array([key.pt for key in keys]).reshape(-1, 2)
                xy = rays(pixels, recording.color)[:, :2]
                if desc is None:
                    desc = np.empty((0, 128), np.float32)
                features.append({'pixels': pixels, 'xy': xy, 'descriptors': desc})
                arrays.update({f'p{i}': pixels, f'x{i}': xy, f'd{i}': desc})
                print(f'five-frame support {index}: {len(pixels)} sites', flush=True)
            np.savez_compressed(cache, **arrays)
        links = augment_flow(recording, indices, features, outdir)
        focal = float((recording.color['fx']+recording.color['fy'])/2)
        graph_path = outdir/'feature_tracks.json'
        graph = json.loads(graph_path.read_text()) if graph_path.exists() else {}
        if graph.get('fingerprint') == source_fingerprint:
            tracks = [{int(i): k for i, k in track.items()} for track in graph['tracks']]
            edges = graph['edges']
        else:
            tracks, edges = build_tracks(features, focal, links)
            graph_path.write_text(json.dumps({'fingerprint': source_fingerprint, 'tracks': tracks, 'edges': edges}))
        point_budget = max(3600, 100*len(indices))
        poses, points, diagnostics = reconstruct(features, tracks, focal, max_points=point_budget)
        scale = None
        scale_info = {'status': 'insufficient_initial_depth'}
        if points:
            ids = [tid for tid in points if 0 in tracks[tid]]
            pixels = np.array([features[0]['pixels'][tracks[tid][0]] for tid in ids])
            cloud, _ = recording.cloud_for_color(0)
            measured = sample_cloud(cloud, pixels)
            triangulated = np.array([points[tid] for tid in ids])
            good = np.isfinite(measured).all(axis=1) & (triangulated[:, 2] > .01)
            ratios = measured[good, 2]/triangulated[good, 2]
            if len(ratios) >= 30:
                scale = float(np.median(ratios))
                mad = float(np.median(np.abs(ratios-scale))/scale)
                scale_info = {'status': 'estimated', 'points': len(ratios), 'scale_m_per_unit': scale,
                              'relative_median_absolute_deviation': mad,
                              'source': 'first_frame_registered_depth_only', 'usable': mad < .15}
        cameras = sorted(poses)
        inverse = [np.linalg.inv(poses[i]) for i in cameras]
        if scale is not None:
            for pose in inverse:
                pose[:3, 3] *= scale
        result = {'recording': Path(bag).name, 'source_fingerprint': source_fingerprint,
            'selected_frames': indices, 'original_selected_frames': coarse['selected_frames'],
            'coarse_unregistered_frames': coarse['unregistered_frames'], 'feature_support_window_frames': 5,
            'sampling': 'add_five_frame_keyframes_around_unsupported_intervals',
            'registered_frames': [indices[i] for i in cameras],
            'unregistered_frames': [frame for i, frame in enumerate(indices) if i not in poses],
            'poses_camera_to_world': [p.tolist() for p in inverse],
            'pose_units': 'metres_from_initial_depth' if scale is not None else 'arbitrary_visual_units',
            'metric_scale': scale_info, 'diagnostics': diagnostics, 'feature_tracks': len(tracks),
            'matching_edges': len(edges), 'depth_used_in_pose_or_bundle_fit': False,
            'landmark_cap': point_budget,
            'imu_used_in_reconstruction': False, 'metric_ground_truth_available': False}
        np.savez_compressed(outdir/'structure.npz', points=np.array(list(points.values()))*(scale or 1.),
                            track_ids=np.array(list(points)))
        (outdir/'reconstruction.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('poses_camera_to_world', 'diagnostics')}), flush=True)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--coarse', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.coarse, args.outdir)


if __name__ == '__main__':
    main()
