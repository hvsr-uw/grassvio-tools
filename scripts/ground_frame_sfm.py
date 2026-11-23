#!/usr/bin/env python3
"""Put the 080033 SfM cloud and camera poses in one ground-aligned frame.

The source reconstruction is metric-scaled but expressed in the first-camera
frame. This script estimates the grass plane from the sparse SfM cloud, anchors
the new origin below Frame 0, and applies one rigid transform to SfM, RGB-D and
IMU rotations. It also writes pose/interval residuals and an SVG overview.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def read_ply(path):
    rows = []
    with path.open(encoding="ascii") as stream:
        for line in stream:
            if line.strip() == "end_header":
                break
        else:
            raise ValueError(f"missing PLY header terminator: {path}")
        for line in stream:
            if line.strip():
                rows.append([float(x) for x in line.split()])
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or data.shape[1] != 6:
        raise ValueError("expected ASCII PLY with x, y, z, red, green, blue")
    return data


def fit_ground_plane(points, threshold=0.008, iterations=20_000):
    """Deterministic RANSAC followed by least-squares PCA refinement."""
    state = 0x12345678

    def index():
        nonlocal state
        state = (1664525 * state + 1013904223) & 0xFFFFFFFF
        return int((state / 4294967296.0) * len(points))

    best_normal = None
    best_inliers = np.zeros(0, dtype=int)
    for _ in range(iterations):
        ids = [index(), index(), index()]
        if len(set(ids)) != 3:
            continue
        a, b, c = points[ids, :3]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-12:
            continue
        normal /= length
        offset = -float(normal @ a)
        inliers = np.flatnonzero(np.abs(points[:, :3] @ normal + offset) <= threshold)
        if len(inliers) > len(best_inliers):
            best_normal, best_inliers = normal, inliers

    if best_normal is None or len(best_inliers) < 3:
        raise ValueError("could not find a stable ground-plane model")

    fit_points = points[best_inliers, :3]
    centroid = fit_points.mean(axis=0)
    covariance = (fit_points - centroid).T @ (fit_points - centroid) / len(fit_points)
    _, vectors = np.linalg.eigh(covariance)
    normal = vectors[:, 0]
    if float(normal @ best_normal) < 0:
        normal = -normal
    normal /= np.linalg.norm(normal)
    offset = -float(normal @ centroid)
    inliers = np.flatnonzero(np.abs(points[:, :3] @ normal + offset) <= threshold)
    distances = np.abs(points[inliers, :3] @ normal + offset)
    return normal, offset, inliers, distances


def transform_pose(transform, pose):
    return transform @ np.asarray(pose, dtype=float)


def rotation_error_deg(a, b):
    relative = a.T @ b
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def quantiles(values):
    values = np.sort(np.asarray(values, dtype=float))
    p90_index = math.ceil(0.90 * len(values)) - 1
    return {
        "median": float(np.median(values)),
        "p90": float(values[p90_index]),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def write_svg(path, cloud, sfm_xyz, rgbd_xyz, summary):
    width, height = 1400, 800
    css = """
    <style>
      text{font-family:Arial,sans-serif;fill:#17212b}
      .title{font-size:27px;font-weight:700}.sub{font-size:14px;fill:#475569}
      .grid{stroke:#d7dde5;stroke-width:1}.small{font-size:12px;fill:#475569}
      .legend{font-size:14px}.sfm{fill:#2563eb;stroke:#fff;stroke-width:1}
      .rgbd{fill:#16875d;stroke:#fff;stroke-width:1}.start{fill:#f28c28;stroke:#fff;stroke-width:2}
    </style>
    """
    def project(point):
        x, y, z = point
        return (0.8660254 * (x - y), 0.5 * (x + y) - 0.82 * z)

    scene = np.asarray([project(p) for p in np.vstack((cloud[:, :3], sfm_xyz, rgbd_xyz))])
    sx0, sx1 = float(scene[:, 0].min()), float(scene[:, 0].max())
    sy0, sy1 = float(scene[:, 1].min()), float(scene[:, 1].max())
    sx = lambda x: 90 + (x - sx0) / max(sx1 - sx0, 1e-9) * 550
    sy = lambda y: 180 + (y - sy0) / max(sy1 - sy0, 1e-9) * 480

    x_values = np.r_[cloud[:, 0], sfm_xyz[:, 0], rgbd_xyz[:, 0]]
    y_values = np.r_[cloud[:, 1], sfm_xyz[:, 1], rgbd_xyz[:, 1]]
    x0, x1 = float(x_values.min()), float(x_values.max())
    y0, y1 = float(y_values.min()), float(y_values.max())
    px = lambda x: 770 + (x - x0) / max(x1 - x0, 1e-9) * 560
    py = lambda y: 635 - (y - y0) / max(y1 - y0, 1e-9) * 450

    def polyline(points, mapper, color, dash=""):
        coords = " ".join(f"{mapper(p)[0]:.2f},{mapper(p)[1]:.2f}" for p in points)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2.6"{dash_attr} stroke-linecap="round" stroke-linejoin="round"/>'

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfcfe"/>', css,
        '<text x="54" y="48" class="title">080033 SfM in a ground-aligned shared frame</text>',
        f'<text x="54" y="74" class="sub">Z is height above the fitted grass plane · Frame 0 camera is {summary["world_frame"]["ground_plane_fit"]["frame0_camera_height_above_plane_m"]:.3f} m above ground · {len(sfm_xyz)} registered views</text>',
        '<line x1="700" y1="104" x2="700" y2="742" stroke="#d7dde5"/>',
        '<text x="64" y="120" class="legend" font-weight="700">Sparse scene and camera path</text>',
        '<text x="754" y="120" class="legend" font-weight="700">Ground-plane top view: SfM vs RGB-D</text>',
        '<rect x="82" y="162" width="570" height="520" fill="#fff" stroke="#e3e8ef"/>',
    ]
    for row in cloud:
        x, y = project(row[:3])
        rgb = tuple(int(np.clip(round(v), 0, 255)) for v in row[3:6])
        parts.append(f'<circle cx="{sx(x):.2f}" cy="{sy(y):.2f}" r="2.2" fill="rgb{rgb}" stroke="#344054" stroke-opacity=".15" stroke-width=".5"/>')
    parts += [
        polyline(rgbd_xyz, lambda p: (sx(project(p)[0]), sy(project(p)[1])), "#16875d", "6 4"),
        polyline(sfm_xyz, lambda p: (sx(project(p)[0]), sy(project(p)[1])), "#2563eb"),
    ]
    for p in sfm_xyz:
        x, y = project(p)
        parts.append(f'<circle class="sfm" cx="{sx(x):.2f}" cy="{sy(y):.2f}" r="3.8"/>')
    x, y = project(sfm_xyz[0])
    parts += [
        f'<circle class="start" cx="{sx(x):.2f}" cy="{sy(y):.2f}" r="6"/>',
        f'<text class="small" x="{sx(x)+8:.2f}" y="{sy(y)-9:.2f}">Frame 0</text>',
        '<text class="small" x="100" y="710">Isometric view: reconstructed points with SfM (blue) and RGB-D (green) camera paths.</text>',
        '<rect x="754" y="162" width="590" height="520" fill="#fff" stroke="#e3e8ef"/>',
    ]
    for i in range(6):
        gx = 770 + i * 112
        gy = 178 + i * 96.8
        parts.append(f'<line class="grid" x1="{gx:.2f}" y1="178" x2="{gx:.2f}" y2="662"/>')
        parts.append(f'<line class="grid" x1="770" y1="{gy:.2f}" x2="1330" y2="{gy:.2f}"/>')
    for row in cloud:
        rgb = tuple(int(np.clip(round(v), 0, 255)) for v in row[3:6])
        parts.append(f'<circle cx="{px(row[0]):.2f}" cy="{py(row[1]):.2f}" r="2.1" fill="rgb{rgb}" stroke="#344054" stroke-opacity=".15" stroke-width=".5"/>')
    parts += [
        polyline(rgbd_xyz, lambda p: (px(p[0]), py(p[1])), "#16875d", "6 4"),
        polyline(sfm_xyz, lambda p: (px(p[0]), py(p[1])), "#2563eb"),
    ]
    for p in sfm_xyz:
        parts.append(f'<circle class="sfm" cx="{px(p[0]):.2f}" cy="{py(p[1]):.2f}" r="3.8"/>')
    parts += [
        f'<circle class="start" cx="{px(sfm_xyz[0,0]):.2f}" cy="{py(sfm_xyz[0,1]):.2f}" r="6"/>',
        f'<text class="small" x="{px(sfm_xyz[0,0])+9:.2f}" y="{py(sfm_xyz[0,1])-7:.2f}">start</text>',
        '<line x1="770" y1="662" x2="1330" y2="662" stroke="#697586"/>',
        '<line x1="770" y1="178" x2="770" y2="662" stroke="#697586"/>',
        '<text x="1037" y="700" class="small">X ground-forward (m)</text>',
        '<text x="738" y="455" class="small" transform="rotate(-90 738 455)">Y ground-horizontal (m)</text>',
        '<circle cx="775" cy="728" r="5" fill="#2563eb"/><text x="787" y="732" class="small">SfM path</text>',
        '<circle cx="960" cy="728" r="5" fill="#16875d"/><text x="972" y="732" class="small">RGB-D ORB-SLAM3 path</text>',
        '<circle cx="1160" cy="728" r="5" fill="#667085"/><text x="1172" y="732" class="small">Sparse points</text>',
        f'<text x="54" y="786" class="small">ORB-SLAM3 is a model reference, not independent ground truth. Ground fit: {summary["world_frame"]["ground_plane_fit"]["inliers"]}/{summary["world_frame"]["ground_plane_fit"]["points_total"]} points within 8 mm; median inlier distance {summary["world_frame"]["ground_plane_fit"]["median_inlier_absolute_distance_m"]*1000:.2f} mm.</text>',
        '</svg>',
    ]
    path.write_text("\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sfm", type=Path, required=True)
    parser.add_argument("--cloud", type=Path, required=True)
    parser.add_argument("--rgbd", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()

    sfm = json.loads(args.sfm.read_text(encoding="utf-8"))
    rgbd = json.loads(args.rgbd.read_text(encoding="utf-8"))
    imu = json.loads(args.imu.read_text(encoding="utf-8"))
    cloud = read_ply(args.cloud)
    normal, offset, inliers, distances = fit_ground_plane(cloud)
    z_up = -normal  # the fitted normal points away from the camera at the origin
    origin = offset * z_up
    forward = np.array([0.0, 0.0, 1.0])  # +Z in the recorded initial-camera frame
    x_axis = forward - z_up * float(forward @ z_up)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_up, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    basis = np.vstack((x_axis, y_axis, z_up))
    transform = np.eye(4)
    transform[:3, :3] = basis
    transform[:3, 3] = -(basis @ origin)

    frames = sfm["registered_frames"]
    sfm_camera = np.asarray([transform_pose(transform, pose) for pose in sfm["poses_camera_to_world"]])
    rgbd_camera = np.asarray([transform_pose(transform, rgbd["absolute_poses"][frame]) for frame in frames])
    imu_by_frame = {frame: pose for frame, pose in zip(
        imu["chosen_frame_indices"], imu["absolute_poses_primary"]
    )}
    missing = [frame for frame in frames if frame not in imu_by_frame]
    if missing:
        raise ValueError(f"IMU rotations are missing selected frames: {missing}")
    imu_camera = np.asarray([transform_pose(transform, imu_by_frame[frame]) for frame in frames])
    points_world = (basis @ (cloud[:, :3] - origin).T).T

    pose_rows, interval_rows = [], []
    for i, frame in enumerate(frames):
        sfm_pos, rgbd_pos = sfm_camera[i, :3, 3], rgbd_camera[i, :3, 3]
        pos_error = sfm_pos - rgbd_pos
        pose_rows.append({
            "frame": int(frame),
            "sfm_xyz_ground_m": sfm_pos.tolist(),
            "rgbd_xyz_ground_m": rgbd_pos.tolist(),
            "position_error_xyz_ground_m": pos_error.tolist(),
            "position_error_norm_m": float(np.linalg.norm(pos_error)),
            "rotation_error_vs_rgbd_deg": rotation_error_deg(sfm_camera[i, :3, :3], rgbd_camera[i, :3, :3]),
            "rotation_error_vs_imu_deg": rotation_error_deg(sfm_camera[i, :3, :3], imu_camera[i, :3, :3]),
        })
        if i == 0:
            continue
        sfm_delta = sfm_pos - sfm_camera[i - 1, :3, 3]
        rgbd_delta = rgbd_pos - rgbd_camera[i - 1, :3, 3]
        sfm_relative = sfm_camera[i - 1, :3, :3].T @ sfm_camera[i, :3, :3]
        rgbd_relative = rgbd_camera[i - 1, :3, :3].T @ rgbd_camera[i, :3, :3]
        imu_relative = imu_camera[i - 1, :3, :3].T @ imu_camera[i, :3, :3]
        interval_rows.append({
            "frame_start": int(frames[i - 1]),
            "frame_end": int(frame),
            "sfm_delta_xyz_ground_m": sfm_delta.tolist(),
            "rgbd_delta_xyz_ground_m": rgbd_delta.tolist(),
            "delta_translation_error_xyz_m": (sfm_delta - rgbd_delta).tolist(),
            "delta_translation_error_norm_m": float(np.linalg.norm(sfm_delta - rgbd_delta)),
            "sfm_rotation_deg": rotation_error_deg(np.eye(3), sfm_relative),
            "rgbd_rotation_deg": rotation_error_deg(np.eye(3), rgbd_relative),
            "imu_rotation_deg": rotation_error_deg(np.eye(3), imu_relative),
            "rotation_residual_vs_rgbd_deg": rotation_error_deg(sfm_relative, rgbd_relative),
            "rotation_residual_vs_imu_deg": rotation_error_deg(sfm_relative, imu_relative),
        })

    plane_errors = np.abs(cloud[:, :3] @ normal + offset)
    depth = json.loads((args.sfm.parent / "080033-heldout-depth.json").read_text(encoding="utf-8"))
    depth = depth[0]
    ground = {
        "method": "deterministic RANSAC on the sparse SfM point cloud, refined by PCA",
        "points_total": int(len(cloud)),
        "inlier_threshold_m": 0.008,
        "inliers": int(len(inliers)),
        "inlier_fraction": float(len(inliers) / len(cloud)),
        "normal_in_frame0_camera": normal.tolist(),
        "offset_m": float(offset),
        "median_inlier_absolute_distance_m": float(np.median(distances)),
        "p90_inlier_absolute_distance_m": float(np.sort(distances)[math.ceil(0.90 * len(distances)) - 1]),
        "frame0_camera_height_above_plane_m": float(-offset),
    }
    summary = {
        "recording": sfm["recording"],
        "world_frame": {
            "name": "080033 ground-aligned shared frame",
            "definition": {
                "origin": "Frame 0 camera center projected onto the fitted grass plane",
                "x_axis": "Frame 0 camera +Z projected onto the fitted ground plane",
                "y_axis": "Z_up cross X, right-handed",
                "z_axis": "upward normal of the fitted grass plane",
                "units": "metres",
            },
            "ground_plane_fit": ground,
            "T_ground_from_frame0_camera": transform.tolist(),
        },
        "residuals": {
            "pose_count": len(pose_rows),
            "interval_count": len(interval_rows),
            "absolute_position_error_vs_rgbd_m": quantiles([r["position_error_norm_m"] for r in pose_rows]),
            "absolute_rotation_error_vs_rgbd_deg": quantiles([r["rotation_error_vs_rgbd_deg"] for r in pose_rows]),
            "absolute_rotation_error_vs_imu_deg": quantiles([r["rotation_error_vs_imu_deg"] for r in pose_rows]),
            "interval_translation_displacement_error_vs_rgbd_m": quantiles([r["delta_translation_error_norm_m"] for r in interval_rows]),
            "interval_rotation_error_vs_rgbd_deg": quantiles([r["rotation_residual_vs_rgbd_deg"] for r in interval_rows]),
            "interval_rotation_error_vs_imu_deg": quantiles([r["rotation_residual_vs_imu_deg"] for r in interval_rows]),
        },
        "depth_calibration_context": {
            "metric_scale_source": "registered depth at frame 25",
            "scale_m_per_unit": sfm["metric_scale"]["scale_m_per_unit"],
            "heldout_depth_frames": int(depth["frames_with_depth"]),
            "heldout_observations": int(depth["observations"]),
            "median_absolute_depth_error_m": depth["median_absolute_depth_error_m"],
            "p90_absolute_depth_error_m": depth["p90_absolute_depth_error_m"],
            "median_absolute_relative_depth_error": depth["median_absolute_relative_depth_error"],
            "depth_to_rgb_visual_remapping_check": "not performed by this transform; needs synchronized RGB/depth image inputs",
        },
        "limitations": [
            "ORB-SLAM3 is a comparison reference, not independent ground truth.",
            "The plane comes from sparse SfM points; points outside the 8 mm inlier threshold remain in the reconstruction.",
            "IMU translation is excluded because the recorded inertial translation drifts; only gyro-derived rotation is compared.",
            "The depth-to-RGB visual calibration/remapping check remains pending.",
        ],
    }

    world = dict(sfm)
    world["pose_units"] = "metres"
    world["coordinate_frame"] = "ground_aligned_shared_world"
    world["source_coordinate_frame"] = "frame_0_camera"
    world["source_poses_camera_to_world"] = sfm["poses_camera_to_world"]
    world["poses_camera_to_world"] = sfm_camera.tolist()
    world["world_frame"] = summary["world_frame"]

    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "080033-ground-world-reconstruction.json").write_text(
        json.dumps(world, indent=2) + "\n", encoding="utf-8"
    )
    ply_lines = [
        "ply", "format ascii 1.0", f"element vertex {len(cloud)}",
        "property float x", "property float y", "property float z",
        "property uchar red", "property uchar green", "property uchar blue", "end_header",
    ]
    for xyz, color in zip(points_world, cloud[:, 3:6]):
        ply_lines.append(" ".join([*(f"{v:.8f}" for v in xyz), *(str(int(round(v))) for v in color)]))
    (args.outdir / "080033-ground-world-sparse-reconstruction.ply").write_text(
        "\n".join(ply_lines) + "\n", encoding="ascii"
    )

    pose_fields = [
        "frame", "sfm_x_m", "sfm_y_m", "sfm_z_m", "rgbd_x_m", "rgbd_y_m", "rgbd_z_m",
        "position_error_x_m", "position_error_y_m", "position_error_z_m", "position_error_norm_m",
        "rotation_error_vs_rgbd_deg", "rotation_error_vs_imu_deg",
    ]
    with (args.outdir / "080033-ground-world-pose-residuals.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(pose_fields)
        for row in pose_rows:
            writer.writerow([
                row["frame"], *row["sfm_xyz_ground_m"], *row["rgbd_xyz_ground_m"],
                *row["position_error_xyz_ground_m"], row["position_error_norm_m"],
                row["rotation_error_vs_rgbd_deg"], row["rotation_error_vs_imu_deg"],
            ])

    interval_fields = [
        "frame_start", "frame_end", "sfm_dx_m", "sfm_dy_m", "sfm_dz_m",
        "rgbd_dx_m", "rgbd_dy_m", "rgbd_dz_m",
        "translation_error_x_m", "translation_error_y_m", "translation_error_z_m",
        "translation_error_norm_m", "sfm_rotation_deg", "rgbd_rotation_deg", "imu_rotation_deg",
        "rotation_residual_vs_rgbd_deg", "rotation_residual_vs_imu_deg",
    ]
    with (args.outdir / "080033-ground-world-interval-residuals.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(interval_fields)
        for row in interval_rows:
            writer.writerow([
                row["frame_start"], row["frame_end"], *row["sfm_delta_xyz_ground_m"],
                *row["rgbd_delta_xyz_ground_m"], *row["delta_translation_error_xyz_m"],
                row["delta_translation_error_norm_m"], row["sfm_rotation_deg"], row["rgbd_rotation_deg"],
                row["imu_rotation_deg"], row["rotation_residual_vs_rgbd_deg"],
                row["rotation_residual_vs_imu_deg"],
            ])

    (args.outdir / "080033-ground-world-summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    write_svg(args.outdir / "080033-ground-world-frame.svg", points_world, sfm_camera[:, :3, 3],
              rgbd_camera[:, :3, 3], summary)


if __name__ == "__main__":
    main()
