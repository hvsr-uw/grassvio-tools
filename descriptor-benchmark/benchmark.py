#!/usr/bin/env python3
"""Descriptor benchmark across the four GrassVIO bag takes.

For every consecutive RGB frame pair in each take, runs ORB and HOG through
the team's FeatureMethod interface (match(gray_a, gray_b) -> MatchResult) and
scores geometric consistency with a RANSAC homography fit. Also computes a
per-take IMU motion score from gyro magnitudes so takes can be ordered by
motion level without knowing the wind labels.

Reads the .db3 files directly with the verified recorder layout from
dataset-tools/bag-exploration/explore_bag.py (Header without seq,
device-clock ns timestamps). No ROS install needed.

Usage:
    PYTHONPATH=<path to the feature methods package> \
        python3 benchmark.py --bag-dir <folder with the .db3 takes> \
        --out results.json
"""

import argparse
import json
import os
import sqlite3
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "bag-exploration"))
from explore_bag import parse_image, parse_imu, COLOR, GYRO  # noqa: E402

from features.registry import match_method  # noqa: E402

BAGS = {
    "102608b": "browser-download-20261009T145113.734224621Z-3-20260926_102608b.db3",
    "102631c": "browser-download-20261009T144901.767060640Z-1-20260926_102631c.db3",
    "102706d": "browser-download-20261009T144923.613095444Z-2-20260926_102706d.db3",
    "102737a": "browser-download-20261009T144731.470583499Z-0-20260926_102737a.db3",
}
# The folder holding the .db3 takes, passed with --bag-dir.
METHODS = ("orb", "hog")
RANSAC_THRESH_PX = 3.0


def load_frames(bag_path):
    """Decode all RGB frames to grayscale, in timestamp order."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (COLOR,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    db.close()
    grays = []
    for (blob,) in rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(p["height"], p["width"], 3)
        grays.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    return grays


def gyro_motion_score(bag_path):
    """Mean and p95 of gyro angular-velocity magnitude (rad/s)."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (GYRO,)).fetchone()[0]
    rows = db.execute("select data from messages where topic_id=?", (tid,)).fetchall()
    db.close()
    mags = np.array([np.linalg.norm(parse_imu(blob)["angular_velocity"]) for (blob,) in rows])
    return float(np.mean(mags)), float(np.percentile(mags, 95)), float(np.max(mags))


def score_pair(result):
    """RANSAC homography on one match result.

    Returns (n_matches, inlier_ratio, median_reproj_px) with NaNs when the
    pair cannot be scored (too few matches or degenerate fit).
    """
    matches = result["matches"]
    n = len(matches)
    if n < 4:
        return n, float("nan"), float("nan")
    src = np.array([[m["x_a"], m["y_a"]] for m in matches], dtype=np.float64)
    dst = np.array([[m["x_b"], m["y_b"]] for m in matches], dtype=np.float64)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH_PX)
    if H is None or mask is None:
        return n, float("nan"), float("nan")
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        return n, float("nan"), float("nan")
    # Reprojection error of inliers: |H * p_a - p_b|.
    ones = np.ones((inl.sum(), 1))
    proj = (H @ np.hstack([src[inl], ones]).T).T
    proj = proj[:, :2] / proj[:, 2:3]
    err = np.linalg.norm(proj - dst[inl], axis=1)
    return n, float(inl.mean()), float(np.median(err))


def benchmark_take(take, bag_path):
    print(f"[{take}] loading frames...", flush=True)
    grays = load_frames(bag_path)
    motion_mean, motion_p95, motion_max = gyro_motion_score(bag_path)
    print(f"[{take}] {len(grays)} frames, motion mean={motion_mean:.4f} rad/s", flush=True)

    per_method = {}
    n_pairs = len(grays) - 1
    for method in METHODS:
        t0 = time.time()
        rows = []
        for i in range(n_pairs):
            r = match_method(method, grays[i], grays[i + 1])
            rows.append(score_pair(r))
            if (i + 1) % 60 == 0:
                print(f"[{take}/{method}] pair {i + 1}/{n_pairs}", flush=True)
        n_m, ir, re = (np.array([r[k] for r in rows], dtype=float) for k in range(3))
        ok = ~np.isnan(ir)
        per_method[method] = {
            "n_pairs": n_pairs,
            "n_pairs_scored": int(ok.sum()),
            "mean_matches_per_pair": float(np.mean(n_m)),
            "mean_inlier_ratio": float(np.mean(ir[ok])) if ok.any() else float("nan"),
            "median_reproj_px": float(np.median(re[ok])) if ok.any() else float("nan"),
            "seconds": round(time.time() - t0, 1),
        }
        m = per_method[method]
        print(f"[{take}/{method}] matches={m['mean_matches_per_pair']:.0f} "
              f"inlier={m['mean_inlier_ratio']:.3f} reproj={m['median_reproj_px']:.2f}px "
              f"({m['seconds']}s)", flush=True)

    return {
        "take": take,
        "n_frames": len(grays),
        "motion_gyro_mean_rads": motion_mean,
        "motion_gyro_p95_rads": motion_p95,
        "motion_gyro_max_rads": motion_max,
        "methods": per_method,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results.json")
    ap.add_argument("--bag-dir", required=True,
                    help="folder holding the .db3 takes")
    args = ap.parse_args()
    results = []
    for take, fname in BAGS.items():
        results.append(benchmark_take(take, os.path.join(args.bag_dir, fname)))
    results.sort(key=lambda r: r["motion_gyro_mean_rads"])
    with open(args.out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"wrote {args.out}")
    print(f"{'take':8} {'motion':>8} | {'orb match':>9} {'orb inl':>7} {'orb px':>6} | "
          f"{'hog match':>9} {'hog inl':>7} {'hog px':>6}")
    for r in results:
        o, h = r["methods"]["orb"], r["methods"]["hog"]
        print(f"{r['take']:8} {r['motion_gyro_mean_rads']:8.4f} | "
              f"{o['mean_matches_per_pair']:9.0f} {o['mean_inlier_ratio']:7.3f} {o['median_reproj_px']:6.2f} | "
              f"{h['mean_matches_per_pair']:9.0f} {h['mean_inlier_ratio']:7.3f} {h['median_reproj_px']:6.2f}")


if __name__ == "__main__":
    main()
