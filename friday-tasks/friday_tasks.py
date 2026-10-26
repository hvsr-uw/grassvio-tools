#!/usr/bin/env python3
"""Friday tasks for GrassVIO: Harsh's three items from the Oct 2 team chat.

1. Distance between frames by features.
   For each consecutive RGB frame pair, match ORB features and compute the
   pixel displacement of every raw match. Report mean and median displacement
   in pixels. This is a simple feature based distance between frames.

2. Filtering.
   Filter the raw matches with a RANSAC homography fit at 3.0 pixels.
   Report the share of matches that survive (inlier ratio) and the median
   reprojection error of the survivors in pixels.

3. Integration.
   Between the two frame timestamps, integrate the gyroscope to a delta
   rotation and the accelerometer to a delta velocity. Report the delta
   rotation magnitude in radians and in degrees, and the delta velocity
   magnitude in meters per second. The accelerometer integration is naive:
   no gravity removal and no orientation compensation, so treat it as a
   rough motion magnitude only.

Reads the .db3 files directly with the verified recorder layout from
dataset-tools/bag-exploration/explore_bag.py. No ROS install needed.

Usage:
    PYTHONPATH=<path to the feature methods package> \
        python3 friday_tasks.py --bag-dir <folder with the .db3 takes> \
        --out results.json
"""

import argparse
import json
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "bag-exploration"))
from explore_bag import parse_image, parse_imu, COLOR, GYRO, ACCEL  # noqa: E402

from features.registry import match_method  # noqa: E402

BAGS = {
    "102608b": "browser-download-20261009T145113.734224621Z-3-20260926_102608b.db3",
    "102631c": "browser-download-20261009T144901.767060640Z-1-20260926_102631c.db3",
    "102706d": "browser-download-20261009T144923.613095444Z-2-20260926_102706d.db3",
    "102737a": "browser-download-20261009T144731.470583499Z-0-20260926_102737a.db3",
}
# The folder holding the .db3 takes, passed with --bag-dir.
RANSAC_THRESH_PX = 3.0


def load_frames(bag_path):
    """Return (grays, stamps_ns) in timestamp order."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (COLOR,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    db.close()
    grays, stamps = [], []
    for (blob,) in rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(p["height"], p["width"], 3)
        grays.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        stamps.append(p["stamp_ns"])
    return grays, np.array(stamps, dtype=np.float64)


def load_imu(bag_path, topic):
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (topic,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    db.close()
    stamps, vecs = [], []
    for (blob,) in rows:
        p = parse_imu(blob)
        stamps.append(p["stamp_ns"])
        if "Gyro" in topic:
            vecs.append(p["angular_velocity"])
        else:
            vecs.append(p["linear_acceleration"])
    return np.array(stamps, dtype=np.float64), np.array(vecs, dtype=np.float64)


def integrate_interval(stamps, vecs, t_a, t_b):
    """Integrate vector samples over [t_a, t_b] with zero order hold.

    Returns the integrated vector (vec * seconds). Stamps are in ns.
    """
    if t_b <= t_a or len(stamps) == 0:
        return np.zeros(3)
    # Find samples overlapping the interval.
    idx = np.where((stamps >= t_a - 1e9) & (stamps <= t_b + 1e9))[0]
    if len(idx) == 0:
        return np.zeros(3)
    total = np.zeros(3)
    for k in idx:
        s0 = max(stamps[k], t_a)
        s1 = min(stamps[k + 1] if k + 1 < len(stamps) else t_b, t_b)
        dt = (s1 - s0) / 1e9
        if dt > 0:
            total += vecs[k] * dt
    return total


def task_pair(result, gyro_stamps, gyro_vecs, accel_stamps, accel_vecs, t_a, t_b):
    matches = result["matches"]
    n = len(matches)
    if n == 0:
        dist = (float("nan"), float("nan"))
    else:
        d = np.array([np.hypot(m["x_b"] - m["x_a"], m["y_b"] - m["y_a"])
                      for m in matches])
        dist = (float(np.mean(d)), float(np.median(d)))

    if n < 4:
        filt = (n, float("nan"), float("nan"))
    else:
        src = np.array([[m["x_a"], m["y_a"]] for m in matches], dtype=np.float64)
        dst = np.array([[m["x_b"], m["y_b"]] for m in matches], dtype=np.float64)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH_PX)
        if H is None or mask is None:
            filt = (n, float("nan"), float("nan"))
        else:
            inl = mask.ravel().astype(bool)
            if inl.sum() < 4:
                filt = (n, float("nan"), float("nan"))
            else:
                ones = np.ones((inl.sum(), 1))
                proj = (H @ np.hstack([src[inl], ones]).T).T
                proj = proj[:, :2] / proj[:, 2:3]
                err = np.linalg.norm(proj - dst[inl], axis=1)
                filt = (n, float(inl.mean()), float(np.median(err)))

    dtheta = integrate_interval(gyro_stamps, gyro_vecs, t_a, t_b)
    dv = integrate_interval(accel_stamps, accel_vecs, t_a, t_b)
    ang = float(np.linalg.norm(dtheta))
    return {
        "n_matches": n,
        "dist_mean_px": dist[0],
        "dist_median_px": dist[1],
        "inlier_ratio": filt[1],
        "reproj_median_px": filt[2],
        "delta_angle_rad": ang,
        "delta_angle_deg": float(np.degrees(ang)),
        "delta_vel_m_s": float(np.linalg.norm(dv)),
    }


def run_take(take, bag_path):
    print(f"[{take}] loading...", flush=True)
    grays, stamps = load_frames(bag_path)
    gyro_stamps, gyro_vecs = load_imu(bag_path, GYRO)
    accel_stamps, accel_vecs = load_imu(bag_path, ACCEL)
    n_pairs = len(grays) - 1
    pairs = []
    for i in range(n_pairs):
        r = match_method("orb", grays[i], grays[i + 1])
        pairs.append(task_pair(r, gyro_stamps, gyro_vecs,
                               accel_stamps, accel_vecs,
                               stamps[i], stamps[i + 1]))
        if (i + 1) % 60 == 0:
            print(f"[{take}] pair {i + 1}/{n_pairs}", flush=True)

    def avg(key):
        v = np.array([p[key] for p in pairs], dtype=float)
        v = v[~np.isnan(v)]
        return float(np.mean(v)) if len(v) else float("nan")

    def med(key):
        v = np.array([p[key] for p in pairs], dtype=float)
        v = v[~np.isnan(v)]
        return float(np.median(v)) if len(v) else float("nan")

    summary = {
        "take": take,
        "n_frames": len(grays),
        "n_pairs": n_pairs,
        "task1_distance_mean_px": avg("dist_mean_px"),
        "task1_distance_median_px": med("dist_median_px"),
        "task2_inlier_ratio": avg("inlier_ratio"),
        "task2_reproj_median_px": med("reproj_median_px"),
        "task3_delta_angle_mean_deg": avg("delta_angle_deg"),
        "task3_delta_angle_median_deg": med("delta_angle_deg"),
        "task3_delta_vel_mean_m_s": avg("delta_vel_m_s"),
    }
    print(f"[{take}] dist_med={summary['task1_distance_median_px']:.2f}px "
          f"inlier={summary['task2_inlier_ratio']:.3f} "
          f"dtheta_med={summary['task3_delta_angle_median_deg']:.4f}deg", flush=True)
    return summary, pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results.json")
    ap.add_argument("--bag-dir", required=True,
                    help="folder holding the .db3 takes")
    args = ap.parse_args()
    summaries = []
    for take, fname in BAGS.items():
        s, _ = run_take(take, os.path.join(args.bag_dir, fname))
        summaries.append(s)
    summaries.sort(key=lambda s: s["task3_delta_angle_mean_deg"])
    with open(args.out, "w") as f:
        json.dump(summaries, f, indent=2)
    print(f"wrote {args.out}")
    for s in summaries:
        print(f"{s['take']:8} pairs={s['n_pairs']:3} "
              f"dist_med={s['task1_distance_median_px']:5.2f}px "
              f"inlier={s['task2_inlier_ratio']:.3f} "
              f"reproj={s['task2_reproj_median_px']:.2f}px "
              f"dtheta={s['task3_delta_angle_median_deg']:.4f}deg "
              f"dv={s['task3_delta_vel_mean_m_s']:.4f}m/s")


if __name__ == "__main__":
    main()
