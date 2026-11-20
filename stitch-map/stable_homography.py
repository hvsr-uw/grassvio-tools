#!/usr/bin/env python3
"""Stable-homography panorama for GrassVIO (Jinming Q3).

Compares a pixel-correct normalized chain with an endpoint re-anchor.
Only a verified endpoint match is used; no linear interpolation of
projective matrices is attempted. Re-anchoring does not optimize the
intermediate chain and is not a global drift correction.

Usage:
    python3 stable_homography.py --bag <take>.db3 --out <prefix> [--stride 5]
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
from explore_bag import parse_image, COLOR  # noqa: E402
from geometry import fit_homography, render_mosaic

RANSAC, LOWE, MIN_IN = 3.0, 0.75, 20
W, H = 640, 480


def hartley_T(pts):
    mean = pts.mean(axis=0)
    dist = np.sqrt(((pts - mean) ** 2).sum(axis=1)).mean()
    s = np.sqrt(2) / (dist + 1e-12)
    return np.array([[s, 0, -s * mean[0]],
                     [0, s, -s * mean[1]], [0, 0, 1.0]])


def apply_T(T, pts):
    h = np.concatenate([pts, np.ones((len(pts), 1))], axis=1) @ T.T
    return h[:, :2] / h[:, 2:3]


def match_h(kp_a, des_a, kp_b, des_b, matcher, normalized):
    """Homography mapping points in frame b into frame a."""
    if des_a is None or des_b is None:
        return None, 0
    good = [m for m, n in
            (pp for pp in matcher.knnMatch(des_b, des_a, k=2)
             if len(pp) == 2)
            if m.distance < LOWE * n.distance]
    if len(good) < MIN_IN:
        return None, 0
    p_b = np.float32([kp_b[m.queryIdx].pt for m in good])
    p_a = np.float32([kp_a[m.trainIdx].pt for m in good])
    H, mask = fit_homography(p_b, p_a, normalize=normalized,
                             threshold_px=RANSAC, min_inliers=MIN_IN)
    return (H, int(mask.sum())) if mask is not None else (None, 0)


def load_frames(bag, stride):
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?",
                     (COLOR,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    db.close()
    frames, gray = [], []
    sift = cv2.SIFT_create(nfeatures=10000)
    feats = []
    for i, (blob,) in enumerate(rows):
        if i % stride:
            continue
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
            p["height"], p["width"], 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        frames.append(bgr)
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray.append(g)
        feats.append(sift.detectAndCompute(g, None))
    print(f"loaded {len(frames)} frames (stride {stride})", flush=True)
    return frames, feats


def chain_canvas(Ts):
    corners = np.float32(
        [[[0, 0], [W, 0], [W, H], [0, H]]]).reshape(-1, 1, 2)
    allc = [cv2.perspectiveTransform(corners, T).reshape(-1, 2)
            for T in Ts]
    allc = np.vstack(allc)
    bw = float(allc[:, 0].max() - allc[:, 0].min())
    bh = float(allc[:, 1].max() - allc[:, 1].min())
    return bw, bh, allc[:, 0].min(), allc[:, 1].min()


def render(frames, Ts, out_path, max_px=4000):
    try:
        image, stats = render_mosaic(frames, Ts, max_side=max_px)
    except ValueError as exc:
        print(f"render skipped: {exc}", flush=True)
        return None
    if not cv2.imwrite(out_path, image):
        raise OSError(f"could not write {out_path}")
    return stats["canvas_width_px"], stats["canvas_height_px"]


def reanchor_chain(transforms, relative, first, last, measured):
    """Use the actual loop endpoints; do not interpolate projective matrices."""
    result = [T.copy() for T in transforms]
    result[last] = transforms[first] @ measured
    for k in range(last + 1, len(result)):
        result[k] = result[k-1] @ relative[k-1]
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    args = ap.parse_args()

    if args.stride < 1:
        ap.error("stride must be positive")
    frames, feats = load_frames(args.bag, args.stride)
    if len(frames) < 2:
        ap.error("need at least two selected frames")
    global W, H
    H, W = frames[0].shape[:2]
    matcher = cv2.BFMatcher()
    n = len(frames)

    # (a) normalized chain, frame k -> frame 0
    Hs, pair_inliers = [], []
    for k in range(1, n):
        Hm, inl = match_h(feats[k - 1][0], feats[k - 1][1],
                           feats[k][0], feats[k][1], matcher, True)
        if Hm is None:
            print(f"pair {k-1}->{k} failed (inliers {inl})", flush=True)
            return 1
        Hs.append(Hm)
        pair_inliers.append(inl)
    Ts = [np.eye(3)]
    for Hm in Hs:
        Ts.append(Ts[-1] @ Hm)
    bw, bh, _, _ = chain_canvas(Ts)
    print(f"normalized chain: {len(Hs)} pairs -> canvas {bw:.0f} x {bh:.0f} px",
          flush=True)
    print(f"pair inliers median {int(np.median(pair_inliers))} "
          f"min {min(pair_inliers)}", flush=True)

    # (b) loop anchor: last 5 frames vs first 5 frames
    loop = []
    for a in range(min(5, n)):
        for b in range(max(0, n - 5), n):
            if b - a < 5:
                continue
            Hm, inl = match_h(feats[a][0], feats[a][1],
                               feats[b][0], feats[b][1], matcher, True)
            loop.append((inl, a, b, Hm))
    loop_ok = [x for x in loop if x[3] is not None]
    print("loop pairs (first5 x last5) inliers:",
          sorted([x[0] for x in loop_ok], reverse=True), flush=True)
    corrected, best_loop = None, None
    if loop_ok:
        inl, a, b, H_loop = max(loop_ok, key=lambda x: x[0])
        best_loop = {"first_frame": a, "last_frame": b, "inliers": inl}
        # H_loop maps b into a, not b into frame 0. Use the ACTUAL
        # matched b (it need not be the last frame) and compose Ts[a].
        corrected = reanchor_chain(Ts, Hs, a, b, H_loop)
        print(f"re-anchored frame {b} via {a}; intermediate drift remains", flush=True)
    else:
        print("no valid non-neighbour endpoint match; normalized chain only", flush=True)

    render(frames, Ts, args.out + "_norm_only.png")
    if corrected is not None:
        render(frames, corrected, args.out + "_loop_reanchored.png")

    json.dump({
        "bag": os.path.basename(args.bag),
        "stride": args.stride, "n_frames": n,
        "pair_inliers_median": int(np.median(pair_inliers)),
        "pair_inliers_min": int(min(pair_inliers)),
        "norm_only_canvas_px": [round(bw, 1), round(bh, 1)],
        "loop_pair_inliers": sorted([x[0] for x in loop_ok], reverse=True),
        "best_loop": best_loop,
        "correction": "endpoint_reanchor_only" if corrected is not None else None,
        "global_drift_corrected": False,
    }, open(args.out + "_stable.json", "w"), indent=2)
    print("done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
