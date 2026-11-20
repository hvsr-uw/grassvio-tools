#!/usr/bin/env python3
"""Incremental image-only reconstruction, then first-frame depth scale calibration.

Triangulation, PnP and bundle adjustment use no depth or IMU. Separate
landmark tracks check fixed poses. Metric scale uses only initial depth;
the resulting trajectory remains an estimate rather than ground truth.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from recording_camera import Recording, COLOR, rays
from relocalization import filtered_sequence_features
from sfm_keyframes import assess_two_view
from motion_channels import sample_cloud, angle
from window_motion import track_window, temporal_filter
from validation_cache import fingerprint, completed


def reciprocal_indices(a, b):
    if a is None or b is None or min(len(a), len(b)) < 2:
        return []
    matcher = cv2.BFMatcher()
    reverse = {m.queryIdx: m.trainIdx for m in matcher.match(b, a)}
    return [(p[0].queryIdx, p[0].trainIdx) for p in matcher.knnMatch(a, b, k=2)
            if len(p) == 2 and p[0].distance < .75*p[1].distance
            and reverse.get(p[0].trainIdx) == p[0].queryIdx]


class TrackUnion:
    """Reject unions that give one landmark two features in the same camera."""
    def __init__(self):
        self.parent = {}
        self.members = {}

    def find(self, key):
        if key not in self.parent:
            self.parent[key] = key
            self.members[key] = {key[0]: key[1]}
        if self.parent[key] != key:
            self.parent[key] = self.find(self.parent[key])
        return self.parent[key]

    def merge(self, a, b):
        a, b = self.find(a), self.find(b)
        if a == b:
            return True
        if self.members[a].keys() & self.members[b].keys():
            return False
        if len(self.members[a]) < len(self.members[b]):
            a, b = b, a
        self.parent[b] = a
        self.members[a].update(self.members.pop(b))
        return True


def build_tracks(features, focal, flow_links=()):
    union = TrackUnion()
    for i, j, source, target in flow_links:
        for a, b in zip(source, target):
            union.merge((i, int(a)), (j, int(b)))
    edge_stats = []
    for i in range(len(features)):
        for j in range(i+1, len(features)):
            matches = reciprocal_indices(features[i]['descriptors'], features[j]['descriptors'])
            if len(matches) < 30:
                continue
            ia, ib = np.array(matches).T
            a, b = features[i]['xy'][ia], features[j]['xy'][ib]
            cv2.setRNGSeed(0)
            _, mask = cv2.findEssentialMat(a, b, np.eye(3), method=cv2.RANSAC,
                                           prob=.999, threshold=1.5/focal)
            if mask is None or mask.sum() < 30:
                continue
            accepted = 0
            for ka, kb in np.array(matches)[mask.ravel() != 0]:
                accepted += union.merge((i, int(ka)), (j, int(kb)))
            edge_stats.append({'a': i, 'b': j, 'matches': len(matches), 'admitted': accepted})
        print(f'feature matching {i+1}/{len(features)}', flush=True)
    tracks = [observations for observations in union.members.values() if len(observations) >= 2]
    tracks.sort(key=lambda t: (-len(t), tuple(sorted(t.items()))))
    return tracks, edge_stats


def triangulate(track, features, poses, focal, min_angle_deg=1.5):
    views = [(i, k) for i, k in sorted(track.items()) if i in poses]
    if len(views) < 2:
        return None
    directions = []
    for i, k in views:
        ray = poses[i][:3, :3].T @ np.r_[features[i]['xy'][k], 1.]
        directions.append(ray/np.linalg.norm(ray))
    separation = np.degrees(np.arccos(np.clip(np.array(directions) @ np.array(directions).T, -1, 1)))
    a, b = np.unravel_index(np.argmax(separation), separation.shape)
    if separation[a, b] < min_angle_deg:
        return None
    i, ki = views[a]
    j, kj = views[b]
    h = cv2.triangulatePoints(poses[i][:3], poses[j][:3],
                              features[i]['xy'][ki].reshape(2, 1), features[j]['xy'][kj].reshape(2, 1))[:, 0]
    if abs(h[3]) < 1e-10:
        return None
    point = h[:3]/h[3]
    errors = []
    for i, k in views:
        p = poses[i][:3, :3] @ point + poses[i][:3, 3]
        if p[2] <= .01:
            return None
        errors.append(np.linalg.norm(p[:2]/p[2] - features[i]['xy'][k])*focal)
    return point if np.median(errors) < 2.5 and max(errors) < 8. else None


def bundle_adjust(poses, points, tracks, features, seed, focal, max_nfev=60):
    if not points:
        return poses, points, {'status': 'no_points'}
    cameras = sorted(poses)
    landmarks = sorted(points)
    camera_id = {v: k for k, v in enumerate(cameras)}
    params = np.array([np.r_[cv2.Rodrigues(poses[i][:3, :3])[0].ravel(), poses[i][:3, 3]] for i in cameras])
    observations, ci, pi = [], [], []
    for p, tid in enumerate(landmarks):
        for i, k in tracks[tid].items():
            if i in poses:
                observations.append(features[i]['xy'][k])
                ci.append(camera_id[i])
                pi.append(p)
    ci, pi, observed = np.array(ci), np.array(pi), np.asarray(observations)
    original = np.r_[params.ravel(), np.array([points[t] for t in landmarks]).ravel()]
    free = np.ones(len(original), bool)
    free[:6] = False  # The first camera is the fixed world coordinate frame.
    seed_row = camera_id[seed]
    free[seed_row*6+3+np.argmax(np.abs(params[seed_row, 3:]))] = False  # Scale gauge.
    sparsity = lil_matrix((len(observed)*2, len(original)), dtype=int)
    for k in range(6):
        sparsity[2*np.arange(len(ci)), ci*6+k] = 1
        sparsity[2*np.arange(len(ci))+1, ci*6+k] = 1
    for k in range(3):
        sparsity[2*np.arange(len(pi)), len(params)*6+pi*3+k] = 1
        sparsity[2*np.arange(len(pi))+1, len(params)*6+pi*3+k] = 1

    def residual(value):
        packed = original.copy()
        packed[free] = value
        camera = packed[:len(params)*6].reshape(-1, 6)
        xyz = packed[len(params)*6:].reshape(-1, 3)
        projected = Rotation.from_rotvec(camera[ci, :3]).apply(xyz[pi]) + camera[ci, 3:]
        z = np.maximum(projected[:, 2:], .001)
        return ((projected[:, :2]/z-observed)*focal).ravel()

    before = residual(original[free])
    solved = least_squares(residual, original[free], jac_sparsity=sparsity.tocsr()[:, free],
                           loss='soft_l1', f_scale=1., x_scale='jac', ftol=1e-5, max_nfev=max_nfev)
    after = residual(solved.x)
    packed = original.copy()
    packed[free] = solved.x
    cameras_out = packed[:len(params)*6].reshape(-1, 6)
    poses_out = {}
    for i, row in zip(cameras, cameras_out):
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(row[:3])[0]
        pose[:3, 3] = row[3:]
        poses_out[i] = pose
    points_out = dict(zip(landmarks, packed[len(params)*6:].reshape(-1, 3)))
    return poses_out, points_out, {'status': str(solved.message), 'converged': bool(solved.success),
        'function_evaluations': solved.nfev, 'observations': len(observed),
        'before_median_px': float(np.median(np.linalg.norm(before.reshape(-1, 2), axis=1))),
        'after_median_px': float(np.median(np.linalg.norm(after.reshape(-1, 2), axis=1)))}


def reconstruct(features, tracks, focal, max_points=3600):
    train = [i for i in range(len(tracks)) if i % 5 != 0]
    seed_candidates = []
    for j in range(1, len(features)):
        shared = [i for i in train if 0 in tracks[i] and j in tracks[i]]
        if len(shared) < 40:
            continue
        a = np.array([features[0]['xy'][tracks[i][0]] for i in shared])*focal
        b = np.array([features[j]['xy'][tracks[i][j]] for i in shared])*focal
        cv2.setRNGSeed(0)
        checked = assess_two_view(a, b, np.diag([focal, focal, 1.]), min_parallax_deg=2.)
        if checked['eligible']:
            score = checked['cheirality_points']*min(checked['median_parallax_deg'], 10.)
            seed_candidates.append((score, j, checked))
    if not seed_candidates:
        return {}, {}, {'status': 'no_supported_initial_baseline'}
    _, seed, checked = max(seed_candidates, key=lambda value: value[0])
    seed_pose = np.eye(4)
    seed_pose[:3, :3] = checked['R_source_to_target']
    seed_pose[:3, 3] = checked['t_unit_source_to_target']
    poses = {0: np.eye(4), seed: seed_pose}
    points = {}
    adjustments = []
    for _ in range(len(features)):
        # Reserve capacity for new parts of the scene as registration advances.
        # Otherwise a still startup can consume the entire landmark budget.
        budget = min(max_points, max(300, 90*len(poses)))
        coverage = {i: sum(i in tracks[t] for t in points) for i in range(len(features)) if i not in poses}
        priority = sorted((tid for tid in train if tid not in points),
                          key=lambda tid: -sum(1./(1+coverage[i]) for i in tracks[tid] if i in coverage))
        for tid in priority:
            if len(points) < budget:
                point = triangulate(tracks[tid], features, poses, focal)
                if point is not None:
                    points[tid] = point
            else:
                break
        candidates = []
        for camera in range(len(features)):
            if camera not in poses:
                known = [tid for tid in points if camera in tracks[tid]]
                candidates.append((len(known), camera, known))
        added = False
        for count, camera, known in sorted(candidates, reverse=True):
            if count < 30:
                break
            xyz = np.array([points[t] for t in known])
            xy = np.array([features[camera]['xy'][tracks[t][camera]] for t in known])
            cv2.setRNGSeed(0)
            ok, rv, tv, inliers = cv2.solvePnPRansac(xyz, xy, np.eye(3), None,
                iterationsCount=1000, reprojectionError=2.5/focal, confidence=.999, flags=cv2.SOLVEPNP_EPNP)
            if not ok or inliers is None or len(inliers) < max(25, .4*count):
                continue
            good = inliers.ravel()
            ok, rv, tv = cv2.solvePnP(xyz[good], xy[good], np.eye(3), None, rv, tv, True)
            if not ok:
                continue
            pose = np.eye(4)
            pose[:3, :3], pose[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
            if np.mean((xyz[good]@pose[:3, :3].T+pose[:3, 3])[:, 2] > 0) < .98:
                continue
            poses[camera] = pose
            print(f'SfM registered {camera}: {len(poses)}/{len(features)} cameras, {len(points)} points', flush=True)
            added = True
            break
        if len(poses) % 5 == 0 or not added:
            poses, points, info = bundle_adjust(poses, points, tracks, features, seed, focal)
            adjustments.append(info)
            # Remove points inconsistent with the refined cameras before further PnP.
            points = {tid: point for tid, point in points.items()
                      if triangulate(tracks[tid], features, poses, focal) is not None}
            print('bundle adjustment', json.dumps(info), flush=True)
        if not added:
            break
    poses, points, info = bundle_adjust(poses, points, tracks, features, seed, focal, max_nfev=100)
    adjustments.append(info)
    holdout_errors = []
    holdout_points = 0
    for tid in range(0, len(tracks), 5):
        visible = {i: k for i, k in sorted(tracks[tid].items()) if i in poses}
        if len(visible) < 3:
            continue
        ordered = list(visible.items())
        point = None
        for split in range(1, len(ordered)-1):
            pair = dict([ordered[0], ordered[split]])
            point = triangulate(pair, features, poses, focal)
            if point is not None:
                break
        if point is None:
            continue
        holdout_points += 1
        for i, k in ordered[split+1:]:
            p = poses[i][:3, :3]@point+poses[i][:3, 3]
            error = np.linalg.norm(p[:2]/p[2]-features[i]['xy'][k])*focal if p[2] > .01 else 1e6
            holdout_errors.append(float(error))
    return poses, points, {'status': 'reconstructed', 'seed_camera': seed, 'seed_geometry': checked,
        'bundle_adjustments': adjustments, 'training_landmarks': len(points),
        'validation_landmarks': holdout_points, 'validation_observations': len(holdout_errors),
        'validation_median_px': float(np.median(holdout_errors)) if holdout_errors else None,
        'validation_p90_px': float(np.percentile(holdout_errors, 90)) if holdout_errors else None,
        'validation_definition': 'excluded from seed fit, PnP and BA; first view and earliest adequate baseline triangulate, later views score'}


def load_features(recording, indices, folder):
    digest = hashlib.sha256()
    digest.update(inspect.getsource(load_features).encode())
    for name in ('recording_camera', 'window_motion', 'relocalization', 'geometry'):
        digest.update(Path(__file__).with_name(name+'.py').read_bytes())
    digest.update(json.dumps([str(recording.path.resolve()), recording.path.stat().st_size,
        recording.path.stat().st_mtime_ns, indices, cv2.__version__, np.__version__]).encode())
    fingerprint = digest.hexdigest()
    cache = folder/'features.npz'
    if cache.exists():
        with np.load(cache) as saved:
            if str(saved['fingerprint']) == fingerprint:
                return [{'pixels': saved[f'p{i}'], 'xy': saved[f'x{i}'], 'descriptors': saved[f'd{i}']} for i in range(len(indices))]
    features = []
    arrays = {'fingerprint': np.array(fingerprint)}
    for i, index in enumerate(indices):
        window = (range(index, min(index+26, len(recording.color_times))) if index+25 < len(recording.color_times)
                  else range(index, max(-1, index-26), -1))
        grays = [cv2.cvtColor(recording.image(COLOR, frame), cv2.COLOR_BGR2GRAY) for frame in window]
        keys, desc = filtered_sequence_features(grays)
        pixels = np.array([key.pt for key in keys]).reshape(-1, 2)
        xy = rays(pixels, recording.color)[:, :2]
        if desc is None:
            desc = np.empty((0, 128), np.float32)
        features.append({'pixels': pixels, 'xy': xy, 'descriptors': desc})
        arrays.update({f'p{i}': pixels, f'x{i}': xy, f'd{i}': desc})
        print(f'temporal features {i+1}/{len(indices)}: {len(pixels)}', flush=True)
    np.savez_compressed(cache, **arrays)
    return features


def augment_flow(recording, indices, features, folder):
    """Carry the same filtered feature sites into later views before redetection."""
    digest = hashlib.sha256(inspect.getsource(augment_flow).encode())
    for name in ('recording_camera', 'window_motion', 'geometry'):
        digest.update(Path(__file__).with_name(name+'.py').read_bytes())
    digest.update(json.dumps([str(recording.path.resolve()), recording.path.stat().st_size,
        recording.path.stat().st_mtime_ns, indices, cv2.__version__, np.__version__]).encode())
    for feature in features:
        digest.update(feature['pixels'].tobytes())
    fingerprint = digest.hexdigest()
    cache = folder/'flow_features.npz'
    if cache.exists():
        with np.load(cache) as saved:
            if str(saved['fingerprint']) == fingerprint:
                for i, feature in enumerate(features):
                    feature['pixels'] = saved[f'p{i}']
                    feature['xy'] = rays(feature['pixels'], recording.color)[:, :2]
                return [(i, i+1, saved[f's{i}'], saved[f't{i}']) for i in range(len(indices)-1)]
    original_counts = [len(f['pixels']) for f in features]
    links = []
    arrays = {'fingerprint': np.array(fingerprint)}
    for i in range(len(indices)-1):
        pixels = features[i]['pixels']
        carried = np.arange(original_counts[i], len(pixels))[:2500]
        fresh = np.arange(original_counts[i])
        if len(carried) and len(fresh):
            distances = cv2.BFMatcher().match(np.float32(pixels[fresh]), np.float32(pixels[carried]))
            fresh = fresh[np.array([m.distance > 4. for m in distances])]
        selected = np.r_[carried, fresh[:1500]]
        grays = [cv2.cvtColor(recording.image(COLOR, frame), cv2.COLOR_BGR2GRAY)
                 for frame in range(indices[i], indices[i+1]+1)]
        history, _, ids = track_window(grays, initial_points=np.float32(pixels[selected]), return_ids=True)
        keep, _ = temporal_filter(history, residual_threshold_px=1.5)
        source = selected[ids[keep]]
        endpoints = history[-1][keep]
        offset = len(features[i+1]['pixels'])
        target = np.arange(offset, offset+len(endpoints))
        features[i+1]['pixels'] = np.vstack((features[i+1]['pixels'], endpoints))
        features[i+1]['xy'] = rays(features[i+1]['pixels'], recording.color)[:, :2]
        links.append((i, i+1, source, target))
        arrays.update({f's{i}': source, f't{i}': target})
        print(f'carried temporal sites {i}->{i+1}: {len(endpoints)}', flush=True)
    for i, feature in enumerate(features):
        arrays[f'p{i}'] = feature['pixels']
    np.savez_compressed(cache, **arrays)
    return links


def run(bag, atlas_path, channels_path, outdir):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    source_fingerprint = fingerprint(bag, [atlas_path, channels_path],
        ['multiview_sfm', 'recording_camera', 'window_motion', 'relocalization', 'geometry',
         'sfm_keyframes', 'motion_channels', 'heldout_diagnostics', 'stitch_map'])
    cached = completed(outdir/'reconstruction.json', source_fingerprint,
                       [outdir/'structure.npz', outdir/'feature_tracks.json'])
    if cached is not None:
        print(f'{Path(bag).name}: cached SfM reconstruction', flush=True)
        return cached
    indices = json.loads(Path(atlas_path).read_text())['selected_frames']
    channel = json.loads(Path(channels_path).read_text())
    recording = Recording(bag)
    try:
        features = load_features(recording, indices, outdir)
        flow_links = augment_flow(recording, indices, features, outdir)
        focal = float((recording.color['fx']+recording.color['fy'])/2)
        graph_hash = hashlib.sha256()
        for function in (build_tracks, reciprocal_indices, TrackUnion):
            graph_hash.update(inspect.getsource(function).encode())
        graph_hash.update(json.dumps([focal, cv2.__version__, np.__version__]).encode())
        for feature in features:
            graph_hash.update(feature['xy'].tobytes())
            graph_hash.update(feature['descriptors'].tobytes())
        for i, j, source, target in flow_links:
            graph_hash.update(np.array([i, j]).tobytes())
            graph_hash.update(source.tobytes())
            graph_hash.update(target.tobytes())
        graph_path = outdir/'feature_tracks.json'
        graph = json.loads(graph_path.read_text()) if graph_path.exists() else {}
        if graph.get('fingerprint') == graph_hash.hexdigest():
            tracks = [{int(i): k for i, k in track.items()} for track in graph['tracks']]
            edges = graph['edges']
        else:
            tracks, edges = build_tracks(features, focal, flow_links)
            graph_path.write_text(json.dumps({'fingerprint': graph_hash.hexdigest(), 'tracks': tracks, 'edges': edges}))
        poses, points, diagnostics = reconstruct(features, tracks, focal)
        metric_scale = None
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
                metric_scale = float(np.median(ratios))
                mad = float(np.median(np.abs(ratios-metric_scale))/metric_scale)
                scale_info = {'status': 'estimated', 'points': len(ratios), 'scale_m_per_unit': metric_scale,
                              'relative_median_absolute_deviation': mad,
                              'source': 'first_frame_registered_depth_only', 'usable': mad < .15}
        camera_indices = sorted(poses)
        world_from_camera = [np.linalg.inv(poses[i]) for i in camera_indices]
        if metric_scale is not None:
            for pose in world_from_camera:
                pose[:3, 3] *= metric_scale
        comparison = []
        rgbd = {i: np.asarray(p) for i, c, p in zip(channel['rgbd_pose_indices'], channel['rgbd_component_ids'],
                                                   channel['rgbd_poses_to_component']) if c == 0}
        if metric_scale is not None:
            for camera, pose in zip(camera_indices, world_from_camera):
                frame = indices[camera]
                if frame in rgbd:
                    comparison.append({'frame': frame, 'position_disagreement_m': float(np.linalg.norm(pose[:3, 3]-rgbd[frame][:3, 3])),
                                       'rotation_disagreement_deg': angle(pose[:3, :3], rgbd[frame][:3, :3])})
        result = {'recording': Path(bag).name, 'source_fingerprint': source_fingerprint, 'selected_frames': indices,
            'registered_frames': [indices[i] for i in camera_indices],
            'unregistered_frames': [frame for i, frame in enumerate(indices) if i not in poses],
            'poses_camera_to_world': [p.tolist() for p in world_from_camera],
            'pose_units': 'metres_from_initial_depth' if metric_scale is not None else 'arbitrary_visual_units',
            'metric_scale': scale_info, 'diagnostics': diagnostics, 'feature_tracks': len(tracks),
            'matching_edges': len(edges), 'depth_used_in_pose_or_bundle_fit': False,
            'imu_used_in_reconstruction': False, 'metric_ground_truth_available': False,
            'rgbd_comparison': comparison}
        (outdir/'reconstruction.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        np.savez_compressed(outdir/'structure.npz', points=np.array(list(points.values()))*(metric_scale or 1.),
                            track_ids=np.array(list(points)), edges=np.array([[e['a'], e['b']] for e in edges]))
        print(json.dumps({k: v for k, v in result.items() if k not in ('poses_camera_to_world', 'rgbd_comparison', 'diagnostics')}), flush=True)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--atlas', type=Path, required=True)
    parser.add_argument('--channels', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    run(args.bag, args.atlas, args.channels, args.outdir)


if __name__ == '__main__':
    main()
