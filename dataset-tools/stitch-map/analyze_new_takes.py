#!/usr/bin/env python3
"""Per-take analysis of the 8 new Nov 6 GrassVIO recordings.

For each take, measures (same methods as the 41-take feature table):
- stream stats + IMU motion stats (same as motion_stats.py)
- wind check: rigid-fit residuals on consecutive pairs vs control 102631c
  (median 0.185 px / p90 0.458 px); labels only "no wind signal" /
  "uncertain" / "non-rigid signal, not separated from parallax"
- loop check: SIFT last-5 vs first-5 stride-5 frames, 20+ inliers = YES
- illumination check (new): per-frame mean gray over time, brightness
  range, largest frame-to-frame jump, and whether SIFT match counts drop
  when brightness jumps. Saves a brightness-curve plot per take.

Also runs the same illumination numbers on the wind-free control
102631c so the verdict bands are calibrated, not guessed.

Bag reads are read-only. One JSON per take in the output dir.
Usage: python3 analyze_new_takes.py --outdir <dir> [--control]
"""
import argparse
import glob
import json
import os
import sqlite3
import sys
import traceback

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "dataset-tools", "bag-exploration"))
from explore_bag import parse_image, parse_imu, COLOR, DEPTH, GYRO, ACCEL  # noqa: E402

RANSAC_THRESH_PX = 3.0
MIN_INLIERS = 20
LOWE_RATIO = 0.75
STRIDE = 5
N_EDGE = 5
NEW_DIR = os.path.expanduser("~/workspace/browser_downloads/sess-2653029502")
CONTROL_BAG = os.path.expanduser(
    "~/workspace/browser_downloads/sess-1861756418/"
    "browser-download-20261027T084630.779088503Z-13-20260926_102631c.db3")

# Wind verdict bands, calibrated on the wind-free control 102631c
# (pooled median 0.185 px, p90 0.458 px).
WIND_NO_SIGNAL = (0.25, 0.60)
WIND_SIGNAL = (0.30, 0.80)

# 085611 has two identical files on disk (first attempt errored, retry
# succeeded; md5-identical). The -8 file is canonical.
BAGS = {
    "20261106_084542": "browser-download-20261106T164822.122638062Z-0-20261106_084542.db3",
    "20261106_084741": "browser-download-20261106T165215.795397528Z-1-20261106_084741.db3",
    "20261106_084852": "browser-download-20261106T165443.220684395Z-2-20261106_084852.db3",
    "20261106_085224": "browser-download-20261106T165757.605027021Z-3-20261106_085224.db3",
    "20261106_085301": "browser-download-20261106T170119.774966625Z-4-20261106_085301.db3",
    "20261106_085422": "browser-download-20261106T170419.658440459Z-5-20261106_085422.db3",
    "20261106_085453": "browser-download-20261106T170707.176918096Z-6-20261106_085453.db3",
    "20261106_085611": "browser-download-20261106T171330.872403110Z-8-20261106_085611.db3",
}


def topic_rows(bag_path, topic):
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    row = db.execute("select id from topics where name=?", (topic,)).fetchone()
    if row is None:
        db.close()
        return []
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (row[0],)).fetchall()
    db.close()
    return rows


def integrity(bag_path):
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    ok = db.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
    topics = [r[0] for r in db.execute("select name from topics")]
    counts = {}
    for t in topics:
        tid = db.execute("select id from topics where name=?", (t,)).fetchone()[0]
        counts[t] = db.execute(
            "select count(*) from messages where topic_id=?", (tid,)).fetchone()[0]
    db.close()
    return ok, topics, counts


def motion_stats(bag_path):
    color_rows = topic_rows(bag_path, COLOR)
    depth_rows = topic_rows(bag_path, DEPTH)
    color_stamps = np.array([parse_image(b)["stamp_ns"] for (b,) in color_rows])
    duration_s = ((color_stamps[-1] - color_stamps[0]) / 1e9
                  if len(color_stamps) > 1 else 0.0)

    gyro_rows = topic_rows(bag_path, GYRO)
    g_vecs = np.array([parse_imu(b)["angular_velocity"] for (b,) in gyro_rows])
    g_mag = np.linalg.norm(g_vecs, axis=1)

    accel_rows = topic_rows(bag_path, ACCEL)
    a_stamps = np.array([parse_imu(b)["stamp_ns"] for (b,) in accel_rows],
                        dtype=np.float64)
    a_vecs = np.array([parse_imu(b)["linear_acceleration"]
                       for (b,) in accel_rows], dtype=np.float64)
    a_mean = a_vecs.mean(axis=0)

    proxy = []
    for k in range(len(color_stamps) - 1):
        t_a, t_b = color_stamps[k], color_stamps[k + 1]
        v = np.zeros(3)
        p3 = np.zeros(3)
        any_s = False
        for j in range(len(a_stamps)):
            s0 = max(a_stamps[j], t_a)
            s1 = min(a_stamps[j + 1] if j + 1 < len(a_stamps) else t_b, t_b)
            dt = (s1 - s0) / 1e9
            if dt <= 0:
                continue
            any_s = True
            v = v + (a_vecs[j] - a_mean) * dt
            p3 = p3 + v * dt
        if any_s:
            proxy.append(float(np.linalg.norm(p3) * 1000.0))
    proxy = np.array(proxy)

    return {
        "color_n": len(color_rows),
        "depth_n": len(depth_rows),
        "duration_s": round(duration_s, 1),
        "gyro_n": len(gyro_rows),
        "gyro_mean": round(float(g_mag.mean()), 5),
        "gyro_median": round(float(np.median(g_mag)), 5),
        "gyro_p95": round(float(np.percentile(g_mag, 95)), 5),
        "gyro_frac_gt_0_25": round(float((g_mag > 0.25).mean()), 4),
        "accel_mean": round(float(np.linalg.norm(a_mean)), 4),
        "transl_proxy_mean_mm": round(float(proxy.mean()), 3) if len(proxy) else 0.0,
        "transl_proxy_median_mm": round(float(np.median(proxy)), 3) if len(proxy) else 0.0,
        "transl_proxy_max_mm": round(float(proxy.max()), 3) if len(proxy) else 0.0,
    }


def pair_fit(kp_cur, des_cur, kp_next, des_next, matcher):
    """(residuals, n_good_matches, n_inliers) of the rigid fit next -> cur."""
    if des_cur is None or des_next is None:
        return None, 0, 0
    matches = matcher.knnMatch(des_next, des_cur, k=2)
    good = []
    for pr in matches:
        if len(pr) != 2:
            continue
        m, n = pr
        if m.distance < LOWE_RATIO * n.distance:
            good.append(m)
    if len(good) < MIN_INLIERS:
        return None, len(good), 0
    src = np.float32([kp_next[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_cur[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH_PX)
    if H is None or mask is None:
        return None, len(good), 0
    n_in = int(mask.sum())
    if n_in < MIN_INLIERS:
        return None, len(good), n_in
    proj = cv2.perspectiveTransform(src, H)
    res = np.linalg.norm(dst - proj, axis=2).ravel()
    return res, len(good), n_in


def fit_inliers(kp_a, des_a, kp_b, des_b, matcher):
    if des_a is None or des_b is None:
        return 0
    matches = matcher.knnMatch(des_b, des_a, k=2)
    good = []
    for pr in matches:
        if len(pr) != 2:
            continue
        m, n = pr
        if m.distance < LOWE_RATIO * n.distance:
            good.append(m)
    if len(good) < MIN_INLIERS:
        return 0
    src = np.float32([kp_b[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_a[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH_PX)
    if H is None or mask is None:
        return 0
    return int(mask.sum())


def process_take(stamp, bag_path, detector, matcher):
    ok, topics, counts = integrity(bag_path)
    if not ok:
        raise RuntimeError("sqlite integrity_check failed")
    rec = {"take": stamp, "bag": os.path.basename(bag_path),
           "integrity_ok": ok, "topics": topics, "topic_counts": counts}
    rec.update(motion_stats(bag_path))

    # ---- decode colour frames once; wind + illumination share the pass ----
    color_rows = topic_rows(bag_path, COLOR)
    color_stamps = np.array([parse_image(b)["stamp_ns"] for (b,) in color_rows])
    t0 = color_stamps[0] / 1e9
    times_s = (color_stamps - color_stamps[0]) / 1e9
    n = len(color_rows)
    if n < 2:
        raise RuntimeError(f"only {n} color frames")

    grays = []
    means = []
    for (blob,) in color_rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
            p["height"], p["width"], 3)
        g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        grays.append(g)
        means.append(float(g.mean()))
    means = np.array(means)

    pooled = []
    pair_meds = []
    pair_inliers = []
    pair_good = []
    fit_ok = 0
    prev = detector.detectAndCompute(grays[0], None)
    for i in range(1, n):
        cur = detector.detectAndCompute(grays[i], None)
        res, n_good, n_in = pair_fit(prev[0], prev[1], cur[0], cur[1], matcher)
        pair_good.append(n_good)
        pair_inliers.append(n_in)
        if res is not None:
            fit_ok += 1
            pair_meds.append(float(np.median(res)))
            pooled.extend(res.tolist())
        prev = cur
        if i % 200 == 0:
            print(f"    {stamp}: pair {i}/{n - 1}", flush=True)
    pooled = np.array(pooled)
    if len(pooled):
        w_med = float(np.median(pooled))
        w_p90 = float(np.percentile(pooled, 90))
        w_mean = float(np.mean(pooled))
        med_max = float(np.max(pair_meds)) if pair_meds else 0.0
        frac_med_gt05 = float(np.mean(np.array(pair_meds) > 0.5)) if pair_meds else 0.0
    else:
        w_med = w_p90 = w_mean = med_max = frac_med_gt05 = 0.0

    if fit_ok < max(10, 0.5 * (n - 1)):
        wind_verdict = "uncertain (too few rigid fits)"
    elif w_med <= WIND_NO_SIGNAL[0] and w_p90 <= WIND_NO_SIGNAL[1]:
        wind_verdict = "no wind signal"
    elif w_med > WIND_SIGNAL[0] or w_p90 > WIND_SIGNAL[1]:
        wind_verdict = "non-rigid signal, not separated from parallax"
    else:
        wind_verdict = "uncertain"

    rec["wind"] = {
        "pairs_total": n - 1, "pairs_fit_ok": fit_ok,
        "pooled_matches": int(len(pooled)),
        "pooled_median_px": round(w_med, 3),
        "pooled_p90_px": round(w_p90, 3),
        "pooled_mean_px": round(w_mean, 3),
        "per_pair_median_max_px": round(med_max, 3),
        "frac_pairs_median_gt_0_5": round(frac_med_gt05, 3),
        "verdict": wind_verdict,
    }

    # ---- illumination: brightness curve + jumps vs matching ----
    jumps = np.abs(np.diff(means))
    il = {
        "mean_brightness": round(float(means.mean()), 2),
        "brightness_min": round(float(means.min()), 2),
        "brightness_max": round(float(means.max()), 2),
        "brightness_range": round(float(means.max() - means.min()), 2),
        "brightness_std": round(float(means.std()), 2),
        "max_jump": round(float(jumps.max()), 2) if len(jumps) else 0.0,
        "max_jump_pair": int(np.argmax(jumps)) + 1 if len(jumps) else -1,
        "median_jump": round(float(np.median(jumps)), 3) if len(jumps) else 0.0,
        "frac_jumps_gt_2": round(float((jumps > 2.0).mean()), 4) if len(jumps) else 0.0,
        "frac_jumps_gt_5": round(float((jumps > 5.0).mean()), 4) if len(jumps) else 0.0,
    }
    # Does matching drop when brightness jumps? Pearson correlation between
    # |brightness jump| and per-pair inlier count, plus median inliers on
    # high-jump pairs vs low-jump pairs.
    inl = np.array(pair_inliers, dtype=float)
    if len(jumps) > 10 and np.std(jumps) > 0 and np.std(inl) > 0:
        il["corr_jump_vs_inliers"] = round(
            float(np.corrcoef(jumps, inl)[0, 1]), 3)
    else:
        il["corr_jump_vs_inliers"] = None
    hi = jumps > 2.0
    il["median_inliers_high_jump_pairs"] = (
        round(float(np.median(inl[hi])), 1) if hi.any() else None)
    il["median_inliers_low_jump_pairs"] = (
        round(float(np.median(inl[~hi])), 1) if (~hi).any() else None)
    il["median_inliers_all_pairs"] = round(float(np.median(inl)), 1)
    rec["illumination"] = il
    rec["brightness_curve"] = {
        "times_s": [round(float(t), 3) for t in times_s],
        "mean_gray": [round(float(m), 2) for m in means],
    }

    # ---- loop pass: first 5 vs last 5 stride-5 chosen frames ----
    chosen = list(range(0, n, STRIDE))
    first = chosen[:N_EDGE]
    last = chosen[-N_EDGE:]
    edge_feats = {}
    for idx in set(first) | set(last):
        edge_feats[idx] = detector.detectAndCompute(grays[idx], None)
    del grays
    matrix = {}
    max_in = 0
    for lf in last:
        row = {}
        for ff in first:
            k = fit_inliers(edge_feats[ff][0], edge_feats[ff][1],
                            edge_feats[lf][0], edge_feats[lf][1], matcher)
            row[str(ff)] = k
            max_in = max(max_in, k)
        matrix[str(lf)] = row
    rec["loop"] = {"first_frames": first, "last_frames": last,
                   "max_inliers": max_in,
                   "verdict": "YES" if max_in >= MIN_INLIERS else "NO",
                   "matrix": matrix}
    return rec


def plot_brightness(stamp, rec, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    times = rec["brightness_curve"]["times_s"]
    vals = rec["brightness_curve"]["mean_gray"]
    il = rec["illumination"]
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(times, vals, lw=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("mean gray (0-255)")
    ax.set_title(f"{stamp} — mean frame brightness "
                 f"(range {il['brightness_range']}, "
                 f"max jump {il['max_jump']})")
    ax.set_ylim(0, 255)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, f"{stamp}_brightness.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--control", action="store_true",
                    help="also run the control 102631c for calibration")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    detector = cv2.SIFT_create(nfeatures=10000)
    matcher = cv2.BFMatcher(cv2.NORM_L2)

    jobs = [(s, os.path.join(NEW_DIR, f)) for s, f in sorted(BAGS.items())]
    if args.control:
        jobs.append(("20260926_102631c_control", CONTROL_BAG))

    summary = {}
    for j, (stamp, bag) in enumerate(jobs):
        out_json = os.path.join(args.outdir, f"{stamp}.json")
        if os.path.exists(out_json) and os.path.exists(
                os.path.join(args.outdir, f"{stamp}_brightness.png")):
            print(f"[{j + 1}/{len(jobs)}] {stamp} — already done, skipping",
                  flush=True)
            with open(out_json) as f:
                r = json.load(f)
            summary[stamp] = {
                "loop": r["loop"]["verdict"], "loop_max": r["loop"]["max_inliers"],
                "wind": r["wind"]["verdict"], "wind_med": r["wind"]["pooled_median_px"],
                "wind_p90": r["wind"]["pooled_p90_px"],
                "illum_range": r["illumination"]["brightness_range"],
                "illum_max_jump": r["illumination"]["max_jump"],
                "gyro_mean": r["gyro_mean"],
                "duration_s": r["duration_s"], "color_n": r["color_n"],
            }
            continue
        print(f"[{j + 1}/{len(jobs)}] {stamp}", flush=True)
        try:
            r = process_take(stamp, bag, detector, matcher)
            plot_brightness(stamp, r, args.outdir)
            # Drop the raw curve from the JSON written to disk? Keep it:
            # it is the evidence behind the plot.
            summary[stamp] = {
                "loop": r["loop"]["verdict"], "loop_max": r["loop"]["max_inliers"],
                "wind": r["wind"]["verdict"], "wind_med": r["wind"]["pooled_median_px"],
                "wind_p90": r["wind"]["pooled_p90_px"],
                "illum_range": r["illumination"]["brightness_range"],
                "illum_max_jump": r["illumination"]["max_jump"],
                "gyro_mean": r["gyro_mean"],
                "duration_s": r["duration_s"], "color_n": r["color_n"],
            }
            print(f"  loop={r['loop']['verdict']} (max {r['loop']['max_inliers']}) "
                  f"wind={r['wind']['verdict']} (med {r['wind']['pooled_median_px']}, "
                  f"p90 {r['wind']['pooled_p90_px']}) "
                  f"illum range={r['illumination']['brightness_range']} "
                  f"maxjump={r['illumination']['max_jump']} "
                  f"gyro={r['gyro_mean']}", flush=True)
        except Exception as e:  # noqa: BLE001 - one bad take must not stop the batch
            r = {"take": stamp, "bag": bag,
                 "error": f"{type(e).__name__}: {e}",
                 "trace": traceback.format_exc()}
            summary[stamp] = {"error": r["error"]}
            print(f"  FAILED: {e}", flush=True)
        with open(os.path.join(args.outdir, f"{stamp}.json"), "w") as f:
            json.dump(r, f, indent=2)
    with open(os.path.join(args.outdir, "_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("SUMMARY:")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    main()
