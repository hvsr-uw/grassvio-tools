"""Export metric SfM in a local world frame defined by gravity and camera 0."""

import argparse
from contextlib import closing
import csv
import json
from pathlib import Path
import sys

import numpy as np

from plot_sfm_reconstruction import read_colored_points

CAMERA_FORWARD = np.array([0.0, 0.0, 1.0])
GRASS_HEIGHT_M = 0.1


def world_from_camera0(gravity_down, mean_depth_m):
    """Use the optical axis for heading and the gravity direction for vertical."""
    down = np.asarray(gravity_down, dtype=float)
    if down.shape != (3,) or not np.isfinite(down).all() or np.linalg.norm(down) < 1e-8:
        raise ValueError("gravity must be a finite, nonzero 3-vector in RGB camera 0")
    if not np.isfinite(mean_depth_m) or mean_depth_m <= 0:
        raise ValueError("mean frame-0 feature depth must be positive and in metres")
    down = down / np.linalg.norm(down)
    up = -down
    left = np.cross(up, CAMERA_FORWARD)
    horizontal = np.linalg.norm(left)
    if horizontal < 1e-6:
        raise ValueError("the frame-0 optical axis is vertical, so world heading is undefined")

    left /= horizontal
    forward = np.cross(left, up)
    transform = np.eye(4)
    transform[:3, :3] = [forward, left, up]
    transform[2, 3] = mean_depth_m * float(CAMERA_FORWARD @ down) + GRASS_HEIGHT_M
    if transform[2, 3] <= 0:
        raise ValueError("the estimated ground must lie below the starting camera")
    return transform


def extract_frame_inputs(bag, rest_end_s, accel_shift_ms):
    """Measure frame-0 feature depth and read a previously verified rest window."""
    import cv2

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stitch-map"))
    from recording_camera import ACCEL, COLOR, Recording

    if not np.isfinite(rest_end_s) or rest_end_s <= 0 or not np.isfinite(accel_shift_ms):
        raise ValueError("use a positive verified rest duration and a finite IMU clock shift")
    with closing(Recording(bag)) as recording:
        if rest_end_s > (recording.color_times[-1] - recording.color_times[0]) / 1e9:
            raise ValueError("the verified rest duration exceeds the recording")
        color = recording.image(COLOR, 0)
        cloud, depth_offset_ms = recording.cloud_for_color(0)
        points = cv2.SIFT_create().detect(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        pixels = np.unique(np.rint([p.pt for p in points]).astype(int).reshape(-1, 2), axis=0)
        height, width = cloud.shape[:2]
        x, y = pixels.T
        inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        depths = cloud[y[inside], x[inside], 2]
        depths = depths[np.isfinite(depths) & (depths > 0)]
        if len(depths) < 3:
            raise ValueError("fewer than three frame-0 features have valid registered depth")

        timestamps, specific_force = recording.imu(ACCEL)
        shift_ns = round(accel_shift_ms * 1e6)
        elapsed = (timestamps - int(recording.color_times[0]) - shift_ns) / 1e9
        resting = (elapsed >= 0) & (elapsed <= rest_end_s)
        if resting.sum() < 3:
            raise ValueError("too few accelerometer samples overlap the supplied rest window")
        # A resting accelerometer measures specific force pointing opposite gravity.
        down = -(recording.color_from_accel[0] @ specific_force[resting].mean(axis=0))
    return {
        "recording": bag.name,
        "gravity_down_camera0": down.tolist(),
        "mean_feature_depth_m": float(depths.mean()),
        "gravity_source": "mean specific force in the supplied verified opening rest window",
        "feature_depth_source": "mean RGB-camera Z at frame-0 SIFT pixels with registered depth",
        "rest_end_s": rest_end_s,
        "accel_time_shift_ms": accel_shift_ms,
        "rest_accel_samples": int(resting.sum()),
        "frame0_depth_samples": len(depths),
        "frame0_depth_pair_offset_ms": depth_offset_ms,
    }


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def camera0_poses(poses, frames, label):
    poses = np.asarray(poses, dtype=float)
    if poses.shape != (len(frames), 4, 4) or not np.isfinite(poses).all():
        raise ValueError(f"{label} must contain one finite 4x4 pose per frame")
    if not frames or frames[0] != 0 or frames != sorted(set(frames)):
        raise ValueError(f"{label} must start at RGB frame 0 and use increasing frame IDs")
    rotations = poses[:, :3, :3]
    if (not np.allclose(poses[:, 3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotations @ rotations.transpose(0, 2, 1), np.eye(3), atol=1e-5)
            or not np.allclose(np.linalg.det(rotations), 1, atol=1e-5)
            or not np.allclose(poses[0], np.eye(4), atol=1e-6)):
        raise ValueError(f"{label} must use rigid camera-to-camera-0 poses")
    return poses


def load_rgbd(path, recording, is_channels):
    rgbd = read_json(path)
    if is_channels:
        if rgbd["recording"] != recording:
            raise ValueError("the RGB-D channels refer to a different recording")
        frames = rgbd["rgbd_pose_indices"]
        components = rgbd["rgbd_component_ids"]
        poses = rgbd["rgbd_poses_to_component"]
        if 0 not in frames:
            raise ValueError("RGB-D channels have no pose at frame 0")
        first = components[frames.index(0)]
        contiguous = [(frame, pose) for frame, cid, pose in zip(frames, components, poses, strict=True)
                      if cid == first]
        frames, poses = map(list, zip(*contiguous))
    else:
        bag_name = Path(rgbd["bag"].replace("\\", "/")).name
        same_recording = bag_name == recording or bag_name.endswith("-" + recording)
        if not same_recording or rgbd["stride"] != 1:
            raise ValueError("the RGB-D comparison needs stride-1 poses from the same recording")
        poses = rgbd["absolute_poses"]
        frames = list(range(len(poses)))
    return dict(zip(frames, camera0_poses(poses, frames, "RGB-D poses"), strict=True))


def export(sfm_path, cloud_path, rgbd_path, inputs, outdir, *, rgbd_channels=False):
    sfm = read_json(sfm_path)
    if sfm["recording"] != inputs["recording"]:
        raise ValueError("the world-frame inputs and SfM must refer to the same recording")
    if sfm["pose_units"] != "metres_from_initial_depth" or not sfm["metric_scale"]["usable"]:
        raise ValueError("the SfM reconstruction must already have metric scale")

    frames = sfm["registered_frames"]
    sfm_poses = camera0_poses(sfm["poses_camera_to_world"], frames, "SfM poses")
    transform = world_from_camera0(inputs["gravity_down_camera0"], inputs["mean_feature_depth_m"])
    sfm_world = transform @ sfm_poses

    rgbd_world = None
    if rgbd_path is not None:
        reference = load_rgbd(rgbd_path, sfm["recording"], rgbd_channels)
        rgbd_world = [transform @ reference[frame] if frame in reference else None for frame in frames]

    world_points = colors = None
    if cloud_path is not None:
        if cloud_path.suffix == ".npz":
            with np.load(cloud_path) as structure:
                cloud = structure["points"]
            colors = np.broadcast_to([.31, .49, .28], cloud.shape)
        else:
            cloud, colors = read_colored_points(cloud_path)
        if (cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud)
                or not np.isfinite(cloud).all() or not np.isfinite(colors).all()):
            raise ValueError("the point cloud must contain finite XYZ points and colors")
        world_points = cloud @ transform[:3, :3].T + transform[:3, 3]

    result = {
        "recording": sfm["recording"],
        "units": "metres",
        "axes": {"x": "frame-0 optical axis projected onto the horizontal plane",
                 "y": "left", "z": "opposite gravity"},
        "origin": "ground-height plane directly below camera 0",
        "grass_height_m": GRASS_HEIGHT_M,
        "world_frame_inputs": inputs,
        "world_from_first_rgb_camera": transform.tolist(),
        "metric_scale": sfm["metric_scale"],
        "registered_frames": frames,
        "sfm_poses_camera_to_world": sfm_world.tolist(),
        "rgbd_reference": ("GrassVIO RGB-D" if rgbd_channels else "ORB-SLAM3 RGB-D") if rgbd_path else None,
        "rgbd_poses_camera_to_world": (
            [pose.tolist() if pose is not None else None for pose in rgbd_world]
            if rgbd_world is not None else None
        ),
    }
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "world-frame.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n",
                                             encoding="utf-8")
    with (outdir / "world-poses.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        axes = ("x", "y", "z")
        header = ["frame", *(f"sfm_{axis}" for axis in axes)]
        if rgbd_world is not None:
            header.extend(f"rgbd_{axis}" for axis in axes)
        writer.writerow(header)
        for i, frame in enumerate(frames):
            row = [frame, *sfm_world[i, :3, 3]]
            if rgbd_world is not None:
                row.extend(rgbd_world[i][:3, 3] if rgbd_world[i] is not None else [""] * 3)
            writer.writerow(row)
    if world_points is not None:
        with (outdir / "world-points.ply").open("w", encoding="ascii") as file:
            file.write(f"ply\nformat ascii 1.0\nelement vertex {len(world_points)}\n"
                       "property float x\nproperty float y\nproperty float z\n"
                       "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            np.savetxt(file, np.column_stack((world_points, np.rint(colors * 255))),
                       fmt=["%.9f"] * 3 + ["%d"] * 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sfm", type=Path, required=True)
    parser.add_argument("--cloud", type=Path, help="SfM point cloud in PLY or structure.npz format")
    rgbd = parser.add_mutually_exclusive_group()
    rgbd.add_argument("--orb", type=Path, help="stride-1 ORB-SLAM3 RGB-D poses")
    rgbd.add_argument("--rgbd-channels", type=Path, help="saved GrassVIO channels.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--frame-inputs", type=Path, help="shared camera-0 gravity and feature depth JSON")
    source.add_argument("--bag", type=Path, help="extract inputs from an original .db3 recording")
    parser.add_argument("--verified-rest-end-s", type=float, help="end time from RGB frame 0")
    parser.add_argument("--accel-time-shift-ms", type=float, default=0)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    if (args.bag is not None) != (args.verified_rest_end_s is not None):
        parser.error("--bag and --verified-rest-end-s must be supplied together")
    inputs = (read_json(args.frame_inputs) if args.frame_inputs is not None
              else extract_frame_inputs(args.bag, args.verified_rest_end_s, args.accel_time_shift_ms))
    export(args.sfm, args.cloud, args.orb or args.rgbd_channels, inputs, args.outdir,
           rgbd_channels=args.rgbd_channels is not None)


if __name__ == "__main__":
    main()
