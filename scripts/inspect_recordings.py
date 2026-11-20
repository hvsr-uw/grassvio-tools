#!/usr/bin/env python3
"""Inspect recording calibration, timing and startup IMU measurements read-only.

Quiet sensor measurements do not independently prove zero camera velocity.
This report does not estimate camera pose or establish channel agreement.
"""
import argparse
import csv
import json
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bag-exploration"))
from explore_bag import ACCEL, COLOR, DEPTH, GYRO, parse_imu, parse_ros_string


STREAMS = {"color": COLOR, "depth": DEPTH, "gyro": GYRO, "accel": ACCEL}


def header_stamp(blob):
    seconds, nanos = struct.unpack_from("<iI", blob, 4)
    return seconds * 1_000_000_000 + nanos


def nearest_offsets_ns(source, target):
    source, target = np.asarray(source, dtype=np.int64), np.sort(np.asarray(target, dtype=np.int64))
    if not len(source) or not len(target):
        raise ValueError("two non-empty timestamp streams required")
    right = np.clip(np.searchsorted(target, source), 0, len(target) - 1)
    left = np.clip(right - 1, 0, len(target) - 1)
    return np.minimum(np.abs(source - target[left]), np.abs(source - target[right]))


def inspect_recording(path, check_integrity=True):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"recording not found: {path.name}")
    report = {"recording": path.name, "size_bytes": path.stat().st_size,
              "integrity": None, "metadata": {}, "streams": {},
              "starts_at_rest_independently_verified": False,
              "camera_stationary_whole_recording_verified": False,
              "three_channel_agreement_tested": False}
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.execute("PRAGMA query_only=ON")
        if check_integrity:
            report["integrity"] = [row[0] for row in db.execute("PRAGMA quick_check")]
            if report["integrity"] != ["ok"]:
                raise ValueError(f"{path.name}: SQLite integrity check failed")
        topics = {name: (tid, kind) for tid, name, kind in db.execute("SELECT id,name,type FROM topics")}
        for name, (tid, kind) in topics.items():
            if kind == "std_msgs/msg/String" and any(
                    word in name for word in ("camera_info", "/tf/", "Depth_Units/value", "imu_intrinsic")):
                row = db.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (tid,)).fetchone()
                if row:
                    report["metadata"][name] = parse_ros_string(row[0])
        timestamps = {}
        for key, name in STREAMS.items():
            if name not in topics:
                report["streams"][key] = {"present": False}
                continue
            tid = topics[name][0]
            # Read only the timestamp header for images, never their large payloads.
            rows = list(db.execute("SELECT timestamp,substr(data,1,16) FROM messages "
                                   "WHERE topic_id=? ORDER BY timestamp", (tid,)))
            if not rows:
                report["streams"][key] = {"present": False}
                continue
            stamps = np.array([header_stamp(blob) for _, blob in rows], dtype=np.int64)
            stored = np.array([stamp for stamp, _ in rows], dtype=np.int64)
            timestamps[key] = stamps
            intervals = np.diff(stamps)
            positive = intervals[intervals > 0]
            stats = {"present": True, "count": len(stamps), "first_ns": int(stamps[0]),
                     "last_ns": int(stamps[-1]), "duration_s": float((stamps[-1] - stamps[0]) / 1e9),
                     "median_rate_hz": float(1e9 / np.median(positive)) if len(positive) else None,
                     "nonincreasing_timestamps": int((intervals <= 0).sum()),
                     "header_minus_storage_median_ms": float(np.median(stamps - stored) / 1e6)}
            if key in ("gyro", "accel"):
                parsed = [parse_imu(blob) for (blob,) in db.execute(
                    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))]
                vectors = np.array([p["angular_velocity"] if key == "gyro" else p["linear_acceleration"]
                                    for p in parsed])
                startup = (stamps >= stamps[0]) & (stamps <= stamps[0] + 1_000_000_000)
                stats.update(first_second_mean=vectors[startup].mean(axis=0).tolist(),
                             first_second_std=vectors[startup].std(axis=0).tolist(),
                             first_second_median_norm=float(np.median(np.linalg.norm(vectors[startup], axis=1))))
            report["streams"][key] = stats
        if "color" in timestamps and "depth" in timestamps:
            offsets = nearest_offsets_ns(timestamps["color"], timestamps["depth"]) / 1e6
            report["color_depth_nearest_offset_ms"] = {
                "median": float(np.median(offsets)), "max": float(offsets.max()),
                "pairs_over_50_ms": int((offsets > 50.).sum())}
    report["all_required_streams_present"] = all(
        report["streams"][key].get("present", False) for key in STREAMS)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--skip-integrity", action="store_true",
                        help="skip the full SQLite check when it has already been run")
    args = parser.parse_args(argv)
    inputs = sorted(args.input_dir.glob("*.db3"))
    if not inputs:
        parser.error("no .db3 recordings found")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summaries = []
    for path in inputs:
        result = inspect_recording(path, not args.skip_integrity)
        destination = args.output_dir / (path.stem + "_sensors.json")
        destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        summary = {"recording": path.name,
                   "all_required_streams_present": result["all_required_streams_present"],
                   "color_frames": result["streams"]["color"].get("count"),
                   "duration_s": result["streams"]["color"].get("duration_s"),
                   "startup_gyro_median_rad_s": result["streams"]["gyro"].get("first_second_median_norm"),
                   "startup_accel_median_m_s2": result["streams"]["accel"].get("first_second_median_norm"),
                   "color_depth_median_offset_ms": result.get("color_depth_nearest_offset_ms", {}).get("median"),
                   "starts_at_rest_verified": False}
        summaries.append(summary)
        print(json.dumps(summary), flush=True)
    (args.output_dir / "sensor_summary.json").write_text(
        json.dumps(summaries, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (args.output_dir / "sensor_summary.csv").open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    return 0 if all(x["all_required_streams_present"] for x in summaries) else 1


if __name__ == "__main__":
    sys.exit(main())
