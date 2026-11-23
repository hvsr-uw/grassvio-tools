"""Express metric SfM and RGB-D poses in a fixed frame based on the measured ground."""

import argparse
from contextlib import closing
import csv
import json
from pathlib import Path
import sys

import numpy as np

from plot_sfm_reconstruction import read_colored_points

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stitch-map"))
from plane_geometry import plane_basis
from plane_panorama import fit_ground
from recording_camera import ACCEL, DepthPairingError, Recording


def ground_transform(normal, distance):
    """Use +X right, +Y forward, +Z up and put the camera's ground foot at zero."""
    right = plane_basis(normal)[:, 0]
    up = -normal
    forward = np.cross(up, right)
    transform = np.eye(4)
    transform[:3, :3] = (right, forward, up)
    transform[:3, 3] = (0, 0, distance)
    return transform


def depth_frames(recording, scale_frame):
    paired = []
    for frame in range(len(recording.color_times)):
        if frame == scale_frame:
            continue
        try:
            recording.depth_pair_for_color(frame)
        except DepthPairingError:
            continue
        paired.append(frame)
    if len(paired) < 8:
        raise ValueError("need eight synchronized frames apart from the SfM scale frame")
    picked = np.array(paired)[np.linspace(0, len(paired) - 1, min(24, len(paired)), dtype=int)]
    return picked[::2].tolist(), picked[1::2].tolist()


def check_plane(recording, frames, reference, transform):
    height = []
    for frame in frames:
        cloud, _ = recording.cloud_for_color(frame)
        points = cloud[::8, ::8].reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1) & (points[:, 2] < 3)]
        pose = transform @ reference[frame]
        height.extend(points @ pose[2, :3] + pose[2, 3])
    return {
        "frames": frames,
        "points": len(height),
        "median_signed_height_m": float(np.median(height)),
        "median_absolute_height_m": float(np.median(np.abs(height))),
        "p90_absolute_height_m": float(np.percentile(np.abs(height), 90)),
    }


def check_up_direction(recording, reference, up):
    """Use the calibrated accelerometer as a check, independent of the plane fit."""
    timestamps, acceleration = recording.imu(ACCEL)
    color_times = recording.color_times
    valid = (timestamps >= color_times[0]) & (timestamps <= color_times[-1])
    timestamps, acceleration = timestamps[valid], acceleration[valid]
    upper = np.searchsorted(color_times, timestamps).clip(1, len(color_times) - 1)
    lower = upper - 1
    nearest = np.where(timestamps - color_times[lower] < color_times[upper] - timestamps, lower, upper)
    in_color = acceleration @ recording.color_from_accel[0].T
    in_first_camera = np.einsum("nij,nj->ni", reference[nearest, :3, :3], in_color)
    median = np.median(in_first_camera, axis=0)
    if not np.isfinite(median).all() or np.linalg.norm(median) < 1:
        raise ValueError("cannot estimate an accelerometer up direction")
    unit = median / np.linalg.norm(median)
    return {
        "samples": len(nearest),
        "median_specific_force_first_camera_mps2": median.tolist(),
        "angle_to_ground_normal_deg": float(np.degrees(np.arccos(np.clip(up @ unit, -1, 1)))),
    }


def export(bag, sfm_path, cloud_path, orb_path, outdir):
    sfm = json.loads(sfm_path.read_text(encoding="utf-8"))
    orb = json.loads(orb_path.read_text(encoding="utf-8"))
    if sfm["recording"] != bag.name or not Path(orb["bag"]).name.endswith(bag.name):
        raise ValueError("the recording, SfM and ORB-SLAM3 pose files must refer to the same clip")
    cloud, colors = read_colored_points(cloud_path)
    with closing(Recording(bag)) as recording:
        frame_count = len(recording.color_times)
        frames = sfm["registered_frames"]
        poses = np.asarray(sfm["poses_camera_to_world"], dtype=float)
        reference = np.asarray(orb["absolute_poses"], dtype=float)
        if sfm["pose_units"] != "metres_from_initial_depth" or not sfm["metric_scale"]["usable"]:
            raise ValueError("the SfM reconstruction must have a measured metric scale")
        if not frames or frames != sorted(set(frames)) or not 0 <= frames[0] <= frames[-1] < frame_count:
            raise ValueError("SfM frames must be unique, ordered and inside the recording")
        if poses.shape != (len(frames), 4, 4) or not np.isfinite(poses).all():
            raise ValueError("each SfM frame must have a finite camera-to-world pose")
        if orb["stride"] != 1 or reference.shape != (frame_count, 4, 4) or not np.isfinite(reference).all():
            raise ValueError("the RGB-D reference must have a finite pose for every color frame")
        if frames[0] != 0 or not np.allclose([poses[0], reference[0]], np.eye(4), atol=1e-6):
            raise ValueError("both pose streams must share the first RGB camera frame")
        training, held_out = depth_frames(recording, sfm["metric_scale"]["frame"])
        normal, distance, fitted = fit_ground(recording, training, reference[training])
        transform = ground_transform(normal, distance)
        checked = check_plane(recording, held_out, reference, transform)
        up_check = check_up_direction(recording, reference, transform[2, :3])

    # Left-multiplication changes the reference frame of the whole pose.
    world_poses = transform @ poses
    world_reference = transform @ reference[frames]
    world_points = cloud @ transform[:3, :3].T + transform[:3, 3]
    result = {
        "recording": bag.name,
        "axes": {"x": "initial camera right projected onto the ground", "y": "forward along the ground",
                 "z": "up along the fitted ground normal"},
        "origin": "orthogonal projection of the frame 0 camera center onto the fitted ground plane",
        "units": "metres",
        "world_from_first_rgb_camera": transform.tolist(),
        "metric_scale": sfm["metric_scale"],
        "ground_fit": {**fitted, "frames": training, "normal_first_camera": normal.tolist(),
                       "distance_m": distance},
        "held_out_depth_plane_check": checked,
        "accelerometer_up_check": up_check,
        "registered_frames": frames,
        "sfm_poses_camera_to_world": world_poses.tolist(),
        "orbslam3_rgbd_poses_camera_to_world": world_reference.tolist(),
        "camera_height_range_m": [float(world_poses[:, 2, 3].min()), float(world_poses[:, 2, 3].max())],
        "point_count": len(cloud),
        "median_landmark_height_m": float(np.median(world_points[:, 2])),
        "p90_abs_landmark_height_m": float(np.percentile(np.abs(world_points[:, 2]), 90)),
    }
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "world-frame.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (outdir / "world-poses.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("frame", "sfm_x", "sfm_y", "sfm_z", "rgbd_x", "rgbd_y", "rgbd_z"))
        for frame, pose, rgbd in zip(frames, world_poses, world_reference):
            writer.writerow((frame, *pose[:3, 3], *rgbd[:3, 3]))
    with (outdir / "world-points.ply").open("w", encoding="ascii") as file:
        file.write(f"ply\nformat ascii 1.0\nelement vertex {len(cloud)}\n"
                   "property float x\nproperty float y\nproperty float z\n"
                   "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        np.savetxt(file, np.column_stack((world_points, np.rint(colors * 255))), fmt=["%.9f"] * 3 + ["%d"] * 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("bag", "sfm", "cloud", "orb", "outdir"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    export(args.bag, args.sfm, args.cloud, args.orb, args.outdir)


if __name__ == "__main__":
    main()
