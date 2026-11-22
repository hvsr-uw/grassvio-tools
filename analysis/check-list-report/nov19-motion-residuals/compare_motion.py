#!/usr/bin/env python3
"""Compare SfM, RGB-D, and calibrated gyro on identical camera intervals."""

import argparse
from contextlib import closing
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "stitch-map"))
from gyro_calibration import integrate_rotation, samples_between
from motion_channels import angle
from recording_camera import GYRO, Recording


ROTATION_FIELDS = (
    "sfm_rgbd_rotation_deg", "sfm_imu_rotation_deg", "rgbd_imu_rotation_deg",
)
TRANSLATION_FIELDS = (
    "translation_magnitude_difference_m", "translation_vector_difference_m",
    "translation_direction_difference_deg",
)
DIRECTION_MIN_M = 0.005


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def rgbd_input(root, clip):
    direct_path = root / clip / "channels.json"
    return direct_path if direct_path.is_file() else root / clip / "channels" / "channels.json"


def compare_recording(bag, recovery_dir, sfm_dir):
    channels = read_json(rgbd_input(recovery_dir, bag.stem))
    sfm_dir = sfm_dir / bag.stem
    reconstruction_path = sfm_dir / "dense_sfm" / "reconstruction.json"
    if not reconstruction_path.is_file():
        reconstruction_path = sfm_dir / "sfm" / "reconstruction.json"
    sfm = read_json(reconstruction_path)
    timing = read_json(sfm_dir / "gyro_timing.json")
    if any(data["recording"] != bag.name for data in (channels, sfm, timing)):
        raise ValueError(f"input recording names disagree for {bag.name}")

    poses = dict(zip(sfm["registered_frames"], map(np.asarray, sfm["poses_camera_to_world"]), strict=True))
    scale = sfm["metric_scale"]
    calibration = timing["selected_hypothesis"]
    shift_ns = round(calibration["shift_ms"] * 1e6)
    bias = np.asarray(calibration["bias"])
    rows = []
    with closing(Recording(bag)) as recording:
        color_times = np.asarray(recording.color_times, dtype=np.int64)
        camera_elapsed = (color_times - color_times[0]) / 1e9
        prefix_end = camera_elapsed[-1] * timing["training_fraction"]
        gyro_times, gyro = recording.imu(GYRO)
        camera_from_gyro = recording.color_from_gyro[0]
        for window in channels["comparisons"]["raw_gyro"]:
            start, end = window["source_frame"], window["target_frame"]
            if camera_elapsed[start] < prefix_end:
                continue
            imu_start, imu_end = color_times[[start, end]] + shift_ns
            rgbd_pose = window.get("T_rgbd_source_to_target")
            row = dict(
                recording=bag.stem, source_frame=start, target_frame=end,
                start_s=float(camera_elapsed[start]), duration_s=float(camera_elapsed[end] - camera_elapsed[start]),
                sfm_available=start in poses and end in poses, rgbd_available=rgbd_pose is not None,
                imu_available=bool(gyro_times[0] <= imu_start < imu_end <= gyro_times[-1]),
            )
            rows.append(row)
            if not all(row[key] for key in ("sfm_available", "rgbd_available", "imu_available")):
                continue

            sfm_transform = np.linalg.solve(poses[end], poses[start])
            rgbd_transform = np.asarray(rgbd_pose)
            sfm_rotation, rgbd_rotation = sfm_transform[:3, :3], rgbd_transform[:3, :3]
            imu_rotation = integrate_rotation(
                samples_between(gyro_times, gyro, imu_start, imu_end), bias, camera_from_gyro,
            )
            row.update(sfm_rgbd_rotation_deg=angle(sfm_rotation, rgbd_rotation),
                       sfm_imu_rotation_deg=angle(sfm_rotation, imu_rotation),
                       rgbd_imu_rotation_deg=angle(rgbd_rotation, imu_rotation))
            if not scale.get("usable"):
                continue

            # Convert world-to-camera translations to camera displacement in source-camera axes.
            sfm_step = -sfm_rotation.T @ sfm_transform[:3, 3]
            rgbd_step = -rgbd_rotation.T @ rgbd_transform[:3, 3]
            sfm_length, rgbd_length = np.linalg.norm(sfm_step), np.linalg.norm(rgbd_step)
            row.update(sfm_displacement_m=float(sfm_length), rgbd_displacement_m=float(rgbd_length),
                       translation_magnitude_difference_m=float(abs(sfm_length - rgbd_length)),
                       translation_vector_difference_m=float(np.linalg.norm(sfm_step - rgbd_step)))
            if min(sfm_length, rgbd_length) > DIRECTION_MIN_M:
                cosine = (sfm_step @ rgbd_step) / (sfm_length * rgbd_length)
                row["translation_direction_difference_deg"] = float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))

    provenance = dict(sfm_variant=reconstruction_path.parent.name, registered_views=len(poses),
                      sfm_scale=scale.get("source"), gyro_shift_ms=calibration["shift_ms"],
                      gyro_fit_fraction=timing["training_fraction"])
    return rows, provenance


def summarize(rows):
    counts = {key: sum(row[key] for row in rows) for key in ("sfm_available", "rgbd_available", "imu_available")}
    summary = dict(later_windows=len(rows), **counts)
    for field in ROTATION_FIELDS + TRANSLATION_FIELDS:
        values = [row[field] for row in rows if field in row]
        summary[field] = dict(count=len(values), median=float(np.median(values)) if values else None,
                              p90=float(np.percentile(values, 90)) if values else None)
    return summary


def plot_summary(clips, output):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True)
    positions = np.arange(len(clips))
    groups = (
        (ROTATION_FIELDS, ("SfM vs RGB-D", "SfM vs IMU", "RGB-D vs IMU"), "Rotation difference (°)", 1),
        (TRANSLATION_FIELDS[:2], ("Magnitude", "Vector"), "SfM vs RGB-D translation (mm)", 1000),
        (TRANSLATION_FIELDS[2:], ("Direction",), "SfM vs RGB-D direction (°)", 1),
    )
    colors = ("#2467A6", "#B64923", "#745399")
    for ax, (fields, labels, title, factor) in zip(axes, groups):
        offsets = (np.arange(len(fields)) - (len(fields) - 1) / 2) * 0.22
        for field, label, color, offset in zip(fields, labels, colors, offsets):
            valid = [(index, clip[field]) for index, clip in enumerate(clips.values()) if clip[field]["count"]]
            medians = np.array([stats["median"] for _, stats in valid]) * factor
            p90s = np.array([stats["p90"] for _, stats in valid]) * factor
            y = np.array([index for index, _ in valid]) + offset
            ax.hlines(y, medians, p90s, color=color, linewidth=1.5, alpha=0.55)
            ax.scatter(medians, y, color=color, label=label, s=24, zorder=3)
        ax.set_title(title, loc="left", fontsize=11)
        ax.grid(axis="x", color="#DCE3EB", linewidth=0.75)
        ax.set_xlim(left=0)
        ax.set_ylim(len(clips) - 0.5, -0.5)
        ax.set_yticks(positions, [clip.rsplit("_", 1)[-1] for clip in clips])
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(loc="upper left", bbox_to_anchor=(-0.02, -0.17), frameon=False, fontsize=8,
                  ncol=min(len(fields), 2))
    fig.suptitle("Nov 19 motion agreement on matched frame intervals", x=0.06, ha="left",
                 fontsize=16, fontweight="bold")
    fig.text(0.06, 0.895, "Dots show the median. Lines extend to the 90th percentile.", fontsize=9)
    fig.subplots_adjust(left=0.09, right=0.98, bottom=0.23, top=0.77, wspace=0.15)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--rgbd-dir", type=Path, required=True)
    parser.add_argument("--sfm-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    bags = sorted(args.recordings.glob("20261119_*.db3"))
    if not bags:
        parser.error("no November 19 recordings found")

    rows, clips = [], {}
    for bag in bags:
        recording_rows, provenance = compare_recording(bag, args.rgbd_dir, args.sfm_dir)
        clips[bag.stem] = {**summarize(recording_rows), **provenance}
        rows.extend(recording_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_fields = list(dict.fromkeys(key for row in rows for key in row))
    with (args.output_dir / "window-residuals.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=output_fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(direction_min_m=DIRECTION_MIN_M, overall=summarize(rows), per_recording=clips)
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot_summary(clips, args.output_dir / "nov19-motion-residuals.png")


if __name__ == "__main__":
    main()
