#!/usr/bin/env python3
"""Render a textured 2D ground plane from calibrated RGB images and camera poses."""

import argparse
from contextlib import closing
import json
from pathlib import Path

import cv2
import numpy as np

from plane_geometry import fit_plane, plane_basis
from recording_camera import COLOR, DepthPairingError, Recording, project, rays
from seam_blending import SeamBlender, measure_seams


def fit_ground(recording, indices, poses):
    """Estimate one reference plane from depth registered to the color camera."""
    clouds = []
    for k in np.linspace(0, len(indices) - 1, min(12, len(indices)), dtype=int):
        try:
            cloud, _ = recording.cloud_for_color(indices[k])
        except DepthPairingError:
            continue
        points = cloud[::8, ::8].reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1) & (points[:, 2] < 3)]
        pose = poses[k]
        clouds.append(points @ pose[:3, :3].T + pose[:3, 3])
    if not clouds:
        raise ValueError("no synchronized depth samples for the ground plane")
    return fit_plane(np.concatenate(clouds))


def plane_to_camera(pose, normal, distance, basis, pixels_per_m):
    axes = np.column_stack((basis / pixels_per_m, normal * distance - pose[:3, 3]))
    return pose[:3, :3].T @ axes


def footprint(forward, sample_rays, width, height, max_depth):
    """Bound valid inverse rendering using sampled source rays through the plane."""
    flat = sample_rays @ np.linalg.inv(forward).T
    depth = np.divide(1, flat[:, 2], out=np.full(len(flat), np.inf), where=flat[:, 2] > 0)
    visible = (depth > .05) & (depth <= max_depth)
    if not visible.any():
        return None
    uv = flat[visible, :2] * depth[visible, None]
    # Pad for the sampling grid and final subpixel interpolation near its ends.
    pad = max(8, round(max(width, height) / 45))
    return np.floor(uv.min(axis=0)).astype(int) - pad, np.ceil(uv.max(axis=0)).astype(int) + pad + 1


def mapped_pixels(forward, lo, hi, calibration, max_depth, min_sampling):
    yy, xx = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
    camera = np.stack((xx, yy, np.ones_like(xx)), axis=-1) @ forward.T
    depth = camera[..., 2]
    positive = depth > .05
    camera[~positive] = (0, 0, 1)
    coordinates = project(camera, calibration).reshape((*depth.shape, 2)).astype(np.float32)
    u, v = coordinates.transpose(2, 0, 1)
    border = np.minimum.reduce((u, v, calibration['width'] - 1 - u, calibration['height'] - 1 - v))
    # The smallest local singular value measures source sampling in the weakest direction.
    uy, ux = np.gradient(u)
    vy, vx = np.gradient(v)
    total = ux**2 + uy**2 + vx**2 + vy**2
    determinant = ux * vy - uy * vx
    largest = np.sqrt((total + np.sqrt(np.maximum(total**2 - 4 * determinant**2, 0))) / 2)
    sampling = np.abs(determinant) / np.maximum(largest, 1e-8)
    valid = positive & (depth <= max_depth) & (border >= 4) & (sampling >= min_sampling)
    score = np.where(valid, np.clip(border / 48, 0, 1) * np.clip(sampling, .02, 1.5), 0)
    return u, v, valid, score


def source_detail(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    gray = gray[height // 10:height - height // 10, width // 10:width - width // 10]
    detail = cv2.Laplacian(gray, cv2.CV_32F)
    return float(np.sqrt(np.mean(detail**2)))


def composite(recording, indices, forwards, outdir, seam_radius, max_depth, min_sampling, blur_weight=True):
    calibration = recording.color
    width, height = calibration['width'], calibration['height']
    xx, yy = np.meshgrid(np.linspace(4, width - 5, 85), np.linspace(4, height - 5, 65))
    probes = rays(np.column_stack((xx.ravel(), yy.ravel())), calibration)
    regions = [(index, forward, footprint(forward, probes, width, height, max_depth))
               for index, forward in zip(indices, forwards)]
    regions = [(index, forward, region) for index, forward, region in regions if region is not None]
    if not regions:
        raise ValueError('ground plane is outside all sampled views')
    origin = np.min([region[0] for _, _, region in regions], axis=0)
    hi = np.max([region[1] for _, _, region in regions], axis=0)
    w, h = (hi - origin).tolist()
    if max(w, h) > 4500 or w * h > 8_000_000:
        raise ValueError(f'ground-plane canvas exceeds render budget ({w}x{h})')

    hard = np.zeros((h, w, 3), np.uint8)
    score = np.zeros((h, w), np.float32)
    owner = np.full((h, w), -1, np.int32)
    alternate = np.full_like(owner, -1)
    alt_score = np.zeros_like(score)
    alt_color = np.zeros_like(hard)
    color_sum = np.zeros_like(hard, dtype=np.float32)
    counts = np.zeros_like(score, dtype=np.uint16)
    details = {i: source_detail(recording.image(COLOR, i)) for i, _, _ in regions}
    typical_detail = max(float(np.median(list(details.values()))), 1e-6)
    used = []
    for index, forward, (lo, hi) in regions:
        map_x, map_y, valid, quality = mapped_pixels(forward, lo, hi, calibration, max_depth, min_sampling)
        if not valid.any():
            continue
        source = recording.image(COLOR, index)
        warped = cv2.remap(source, map_x, map_y, cv2.INTER_LANCZOS4)
        if blur_weight:
            quality *= np.clip(details[index] / typical_detail, .05, 1.)**2
        x0, y0 = lo - origin
        x1, y1 = hi - origin
        region = np.s_[y0:y1, x0:x1]
        owners, scores, colors = owner[region], score[region], hard[region]
        runners, runner_scores, runner_colors = alternate[region], alt_score[region], alt_color[region]
        take = valid & (quality > scores)
        displaced = take & (owners >= 0)
        runners[displaced], runner_scores[displaced], runner_colors[displaced] = (
            owners[displaced], scores[displaced], colors[displaced])
        runner_up = valid & ~take & (quality > runner_scores)
        runners[runner_up], runner_scores[runner_up], runner_colors[runner_up] = (
            index, quality[runner_up], warped[runner_up])
        owners[take], scores[take], colors[take] = index, quality[take], warped[take]
        color_sum[region][valid] += warped[valid]
        counts[region][valid] += 1
        used.append(index)

    valid = owner >= 0
    if not valid.any():
        raise ValueError('no valid samples on the map canvas')
    _, components, component_stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8))
    areas = component_stats[1:, cv2.CC_STAT_AREA]
    # Drop detached slivers smaller than one thousandth of the largest patch.
    drop_components = np.r_[False, areas < max(1, areas.max() / 1000)]
    dropped = drop_components[components]
    owner[dropped] = alternate[dropped] = -1
    counts[dropped] = 0
    hard[dropped] = color_sum[dropped] = 0
    valid &= ~dropped
    canvas = hard.copy()
    if seam_radius:
        seams = SeamBlender(seam_radius)
        seams.owner, seams.alternative, seams.alt_score, seams.alt_color = owner, alternate, alt_score, alt_color
        seams.render(hard, canvas, 0, 0, w, h)
    yy, xx = np.where(valid)
    x0, y0 = xx.min(), yy.min()
    crop = np.s_[y0:yy.max() + 1, x0:xx.max() + 1]
    average = np.rint(color_sum / np.maximum(counts, 1)[..., None]).astype(np.uint8)
    images = {'panorama': canvas, 'hard_joins': hard, 'average': average,
              'owners': (owner + 1).astype(np.uint16), 'observations': counts}
    for name, image in images.items():
        if not cv2.imwrite(str(outdir / f'{name}.png'), image[crop]):
            raise OSError(f'could not save {name}')
    stats, _ = measure_seams(hard, canvas, owner)
    source_ids, owner_counts = np.unique(owner[valid], return_counts=True)
    stats.update(origin=(origin + [x0, y0]).tolist(), size=list(canvas[crop].shape[1::-1]),
                 sampled_frames=list(indices), projected_frames=used,
                 owner_frames=source_ids.tolist(),
                 pixels_owned_by_frame=dict(zip(source_ids.astype(str).tolist(), owner_counts.tolist())),
                 source_laplacian_rms=details, blur_weighting=blur_weight,
                 multiply_observed_pixels=int(np.count_nonzero(counts >= 2)),
                 detached_pixels_removed=int(np.count_nonzero(dropped)))
    return stats


def run(bag, poses_path, outdir, stride=5, pixels_per_m=500, seam_radius=4, max_depth=2.0,
        min_sampling=0., refine_poses=False, blur_weight=True):
    if stride < 1 or not np.isfinite(pixels_per_m) or pixels_per_m <= 0:
        raise ValueError('stride and scale must be positive')
    if not 0 <= seam_radius <= 32 or not np.isfinite(max_depth) or max_depth <= .05:
        raise ValueError('seam radius must be 0 to 32 and maximum depth must exceed 0.05 m')
    if not 0 <= min_sampling <= 1:
        raise ValueError('minimum source sampling must be between 0 and 1')
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stored = json.loads(Path(poses_path).read_text(encoding='utf-8'))
    with closing(Recording(bag)) as recording:
        indices = list(range(0, len(recording.color_times), stride))
        if indices and indices[-1] >= np.iinfo(np.uint16).max:
            raise ValueError('sampled frame indices exceed the range of the owner PNG')
        poses = np.asarray(stored['absolute_poses'], dtype=np.float64)
        if stored['stride'] != 1 or poses.shape != (len(recording.color_times), 4, 4):
            raise ValueError('one camera-to-world pose is required per color frame')
        poses = poses[indices]
        if not np.isfinite(poses).all():
            raise ValueError('camera poses must be finite')
        normal, distance, plane_stats = fit_ground(recording, indices, poses)
        basis = plane_basis(normal)
        forwards = [plane_to_camera(pose, normal, distance, basis, pixels_per_m) for pose in poses]
        audit = []
        if refine_poses:
            from plane_alignment import refine
            print(f'Refining homographies across {len(indices)} sampled views', flush=True)
            forwards, audit = refine(recording, indices, forwards, pixels_per_m)
        print(f'Ground plane at {distance:.3f} m; rendering {len(indices)} color frames', flush=True)
        stats = composite(recording, indices, forwards, outdir, seam_radius, max_depth, min_sampling, blur_weight)
        stats.update(recording=Path(bag).name, poses_file=Path(poses_path).name,
                     pixels_per_m=pixels_per_m, seam_radius_px=seam_radius, maximum_axial_depth_m=max_depth,
                     minimum_source_pixels_per_map_pixel=min_sampling,
                     plane_normal=normal.tolist(), plane_distance_m=distance,
                     plane_basis=basis.tolist(), plane_fit=plane_stats, alignment_audit=audit,
                     map_to_camera={str(i): matrix.tolist() for i, matrix in zip(indices, forwards)})
    (outdir / 'metrics.json').write_text(json.dumps(stats, indent=2) + '\n', encoding='utf-8')
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=5)
    parser.add_argument('--pixels-per-m', type=float, default=500)
    parser.add_argument('--seam-radius', type=int, default=4)
    parser.add_argument('--max-depth', type=float, default=2.0,
                        help='maximum axial distance to the plane in metres')
    parser.add_argument('--min-sampling', type=float, default=0.,
                        help='minimum source pixels per panorama pixel in the weakest direction')
    parser.add_argument('--refine', action='store_true', help='refine homographies with overlapping color frames')
    parser.add_argument('--no-blur-weight', action='store_true',
                        help='score sources by geometry alone for a comparison render')
    args = parser.parse_args()
    run(args.bag, args.poses, args.outdir, args.stride, args.pixels_per_m, args.seam_radius,
        args.max_depth, args.min_sampling, args.refine, not args.no_blur_weight)


if __name__ == '__main__':
    main()
