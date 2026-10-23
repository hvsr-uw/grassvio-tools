#!/usr/bin/env python3
"""IMU integration and filtering for GrassVIO.

What this script does per take:
  1. Noise assessment: per-axis std of the demeaned accelerometer, and the
     implied per-interval distance noise from double integration.
  2. Visual distance: ORB matches between neighboring RGB frames, back
     projected to 3D with depth and the color intrinsics. The median 3D
     displacement of valid matches, in meters.
  3. IMU distance: accelerometer demeaned by the take mean (gravity and bias
     proxy), double integrated over each frame interval, magnitude in meters.
     Starts each interval at zero velocity, so this is a per-interval
     displacement proxy, not a trajectory.
  4. Kalman filter: 1-D filter over the pair sequence. The IMU distance
     drives the prediction, the visual distance is the measurement. Process
     noise comes from the measured accel noise, measurement noise from the
     spread of 3D displacements inside the pair.

Reads the .db3 files directly with the verified recorder layout from
dataset-tools/bag-exploration/explore_bag.py. No ROS install needed.

Usage:
    python3 imu_fusion.py --out results.json
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
from explore_bag import parse_image, parse_imu  # noqa: E402
from explore_bag import COLOR, DEPTH, GYRO, ACCEL  # noqa: E402

from features.registry import match_method  # noqa: E402

BAGS = {
    "102608b": "browser-download-20261009T145113.734224621Z-3-20260926_102608b.db3",
    "102631c": "browser-download-20261009T144901.767060640Z-1-20260926_102631c.db3",
    "102706d": "browser-download-20261009T144923.613095444Z-2-20260926_102706d.db3",
    "102737a": "browser-download-20261009T144731.470583499Z-0-20260926_102737a.db3",
}
BAG_DIR = os.path.expanduser("~/workspace/browser_downloads/sess-876766658")

# Color intrinsics per take, from the bag metadata topics (Brown Conrady).
INTR = {
    "102608b": (386.671997, 385.930878, 323.054626, 250.530975),
    "102631c": (386.671997, 385.930878, 323.054626, 250.530975),
    "102706d": (386.566772, 385.825867, 323.054626, 250.530975),
    "102737a": (386.566772, 385.825867, 323.054626, 250.530975),
}
DEPTH_UNIT_M = 0.001
MIN_VALID_MATCHES = 10


def load_images(bag_path, topic):
    """Return (images, stamps_ns). RGB as HxWx3 uint8, depth as HxW float32 meters."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (topic,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    db.close()
    imgs, stamps = [], []
    for (blob,) in rows:
        p = parse_image(blob)
        if topic == COLOR:
            rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(p["height"], p["width"], 3)
            imgs.append(rgb)
        else:
            d = np.frombuffer(p["data"], dtype="<u2").reshape(p["height"], p["width"])
            imgs.append(d.astype(np.float32) * DEPTH_UNIT_M)
        stamps.append(p["stamp_ns"])
    return imgs, np.array(stamps, dtype=np.float64)


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
        vecs.append(p["angular_velocity"] if "Gyro" in topic else p["linear_acceleration"])
    return np.array(stamps, dtype=np.float64), np.array(vecs, dtype=np.float64)


def moving_average(vecs, window=5):
    """Centered moving average per axis; a light low-pass denoise.

    Window 5 at ~100 Hz is about 50 ms. Documented first step; a Kalman
    smoother can replace it. Edges use the available samples.
    """
    w = window // 2
    out = np.empty_like(vecs)
    n = len(vecs)
    for i in range(n):
        lo, hi = max(0, i - w), min(n, i + w + 1)
        out[i] = vecs[lo:hi].mean(axis=0)
    return out


def backproject(x, y, d, fx, fy, ppx, ppy):
    return np.array([(x - ppx) * d / fx, (y - ppy) * d / fy, d])


def visual_distance_3d(matches, depth_a, depth_b, fx, fy, ppx, ppy):
    """Median 3D displacement of matches with valid depth in both frames (meters)."""
    h, w = depth_a.shape
    disps = []
    for m in matches:
        xa, ya, xb, yb = int(round(m["x_a"])), int(round(m["y_a"])), \
                         int(round(m["x_b"])), int(round(m["y_b"]))
        if not (0 <= xa < w and 0 <= ya < h and 0 <= xb < w and 0 <= yb < h):
            continue
        da, db = depth_a[ya, xa], depth_b[yb, xb]
        if da <= 0 or db <= 0:
            continue
        pa = backproject(m["x_a"], m["y_a"], da, fx, fy, ppx, ppy)
        pb = backproject(m["x_b"], m["y_b"], db, fx, fy, ppx, ppy)
        disps.append(float(np.linalg.norm(pb - pa)))
    if len(disps) < MIN_VALID_MATCHES:
        return float("nan"), float("nan"), 0
    disps = np.array(disps)
    med = float(np.median(disps))
    mad = float(np.median(np.abs(disps - med)))
    return med, 1.4826 * mad, len(disps)


def imu_distance(stamps, vecs, mean_vec, t_a, t_b):
    """Double integrate demeaned accel over [t_a, t_b]; displacement magnitude (m)."""
    if t_b <= t_a or len(stamps) == 0:
        return float("nan")
    v = np.zeros(3)
    p = np.zeros(3)
    any_sample = False
    for k in range(len(stamps)):
        s0 = max(stamps[k], t_a)
        s1 = min(stamps[k + 1] if k + 1 < len(stamps) else t_b, t_b)
        dt = (s1 - s0) / 1e9
        if dt <= 0:
            continue
        any_sample = True
        a = vecs[k] - mean_vec
        v = v + a * dt
        p = p + v * dt
    if not any_sample:
        return float("nan")
    return float(np.linalg.norm(p))


def fuse_pair(imu_d, vis_d, q_base, spread):
    """One Kalman weighting step for a single frame pair.

    Combines the IMU distance and the visual distance by inverse variance,
    trusting each according to its measured noise. Returns (fused,
    weight_on_visual).
    """
    Q = q_base + (0.1 * imu_d) ** 2
    R = max(spread ** 2, 1e-8)
    K = Q / (Q + R)  # Kalman gain: weight placed on the visual channel
    return (1 - K) * imu_d + K * vis_d, K


def run_take(take, bag_path):
    print(f"[{take}] loading...", flush=True)
    rgbs, rgb_ts = load_images(bag_path, COLOR)
    depths, depth_ts = load_images(bag_path, DEPTH)
    accel_ts, accel_vec = load_imu(bag_path, ACCEL)
    accel_filt = moving_average(accel_vec, window=5)
    accel_rows = [{
        "take": take,
        "timestamp_ns": int(ts),
        "header_stamp_ns": int(ts),
        "linear_acceleration_x": round(float(v[0]), 6),
        "linear_acceleration_y": round(float(v[1]), 6),
        "linear_acceleration_z": round(float(v[2]), 6),
        "filtered_ax": round(float(f[0]), 6),
        "filtered_ay": round(float(f[1]), 6),
        "filtered_az": round(float(f[2]), 6),
    } for ts, v, f in zip(accel_ts, accel_vec, accel_filt)]
    fx, fy, ppx, ppy = INTR[take]
    h, w = depths[0].shape

    # --- noise assessment: demeaned accel std per axis ---
    mean_a = accel_vec.mean(axis=0)
    std_a = accel_vec.std(axis=0)
    mag_std = float(np.linalg.norm(std_a))
    T_nom = 1.0 / 30.0
    # naive per-interval distance noise from double integration: 0.5*sigma*T^2
    dist_noise_m = 0.5 * mag_std * T_nom ** 2
    print(f"[{take}] accel std xyz=({std_a[0]:.4f},{std_a[1]:.4f},{std_a[2]:.4f}) m/s^2 "
          f"-> ~{dist_noise_m*1000:.3f} mm per 33ms interval", flush=True)

    grays = [cv2.cvtColor(r, cv2.COLOR_RGB2GRAY) for r in rgbs]
    n_pairs = len(grays) - 1
    vis_ds, imu_ds, spreads = [], [], []
    for i in range(n_pairs):
        # nearest depth frame by header timestamp
        ja = int(np.argmin(np.abs(depth_ts - rgb_ts[i])))
        jb = int(np.argmin(np.abs(depth_ts - rgb_ts[i + 1])))
        r = match_method("orb", grays[i], grays[i + 1])
        vd, spread, n_valid = visual_distance_3d(
            r["matches"], depths[ja], depths[jb], fx, fy, ppx, ppy)
        vis_ds.append(vd)
        spreads.append(spread)
        imu_ds.append(imu_distance(accel_ts, accel_vec, mean_a, rgb_ts[i], rgb_ts[i + 1]))
        if (i + 1) % 60 == 0:
            print(f"[{take}] pair {i + 1}/{n_pairs}", flush=True)

    vis = np.array(vis_ds)
    imu = np.array(imu_ds)
    ok_v = ~np.isnan(vis)
    ok_i = ~np.isnan(imu)
    ok_both = ok_v & ok_i

    q_base = (0.5 * mag_std * T_nom ** 2) ** 2
    fused, weights = [], []
    pair_rows = []
    for i, (imu_d, vis_d, spread) in enumerate(zip(imu_ds, vis_ds, spreads)):
        if np.isnan(imu_d) or np.isnan(vis_d):
            fused.append(float("nan"))
            weights.append(float("nan"))
            continue
        f, k = fuse_pair(imu_d, vis_d, q_base, spread)
        fused.append(f)
        weights.append(k)
        pair_rows.append({
            "take": take,
            "pair_index": i,
            "t_a_ns": int(rgb_ts[i]),
            "t_b_ns": int(rgb_ts[i + 1]),
            "visual_dist_mm": round(vis_d * 1000, 4),
            "imu_dist_mm": round(imu_d * 1000, 4),
            "fused_dist_mm": round(f * 1000, 4),
            "weight_on_visual": round(k, 4),
        })
    fused = np.array(fused)
    weights = np.array(weights)
    ok_f = ~np.isnan(fused)

    # cumulative path per channel over pairs where both are valid (meters)
    path_vis = float(np.sum(vis[ok_f]))
    path_imu = float(np.sum(imu[ok_f]))
    path_fused = float(np.sum(fused[ok_f]))

    def avg(a):
        a = a[~np.isnan(a)]
        return float(np.mean(a)) if len(a) else float("nan")

    def med(a):
        a = a[~np.isnan(a)]
        return float(np.median(a)) if len(a) else float("nan")

    corr = float(np.corrcoef(vis[ok_both], imu[ok_both])[0, 1]) if ok_both.sum() > 3 else float("nan")
    summary = {
        "take": take,
        "n_pairs": n_pairs,
        "accel_std_xyz_ms2": [round(float(s), 5) for s in std_a],
        "accel_dist_noise_mm_per_interval": round(dist_noise_m * 1000, 4),
        "visual_dist_median_mm": round(med(vis) * 1000, 3),
        "visual_dist_mean_mm": round(avg(vis) * 1000, 3),
        "imu_dist_median_mm": round(med(imu) * 1000, 3),
        "imu_dist_mean_mm": round(avg(imu) * 1000, 3),
        "visual_imu_correlation": round(corr, 3),
        "fused_dist_median_mm": round(med(fused) * 1000, 3),
        "fused_weight_on_visual_mean": round(avg(weights), 3),
        "path_visual_mm": round(path_vis * 1000, 2),
        "path_imu_mm": round(path_imu * 1000, 2),
        "path_fused_mm": round(path_fused * 1000, 2),
        "n_pairs_fused": int(ok_f.sum()),
    }
    print(f"[{take}] vis_med={summary['visual_dist_median_mm']}mm "
          f"imu_med={summary['imu_dist_median_mm']}mm corr={summary['visual_imu_correlation']} "
          f"fused_med={summary['fused_dist_median_mm']}mm", flush=True)
    return summary, pair_rows, accel_rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results.json")
    ap.add_argument("--pairs-out", default="per_pair_distances.csv")
    ap.add_argument("--accel-out", default="accel_filtered.csv")
    args = ap.parse_args()
    summaries = []
    all_rows = []
    all_accel = []
    for take, fname in BAGS.items():
        s, rows, arows = run_take(take, os.path.join(BAG_DIR, fname))
        summaries.append(s)
        all_rows.extend(rows)
        all_accel.extend(arows)
    with open(args.out, "w") as f:
        json.dump(summaries, f, indent=2)
    print(f"wrote {args.out}")
    if all_rows:
        import csv
        with open(args.pairs_out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)
        print(f"wrote {args.pairs_out} ({len(all_rows)} pairs)")
    if all_accel:
        import csv
        with open(args.accel_out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(all_accel[0].keys()))
            w.writeheader()
            w.writerows(all_accel)
        print(f"wrote {args.accel_out} ({len(all_accel)} samples)")


if __name__ == "__main__":
    main()
