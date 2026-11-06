#!/usr/bin/env python3
"""Stable-homography panorama for GrassVIO (Jinming Q3).

Tries to stabilize the chained-homography panorama on 20261007_080033:
  (a) Hartley-normalized per-pair homographies (re-verify Nov 2 result)
  (b) loop-closure anchor: the last frames match the first frames, so the
      measured end-to-start homography constrains the chain end. The
      accumulated error D = H_loop @ inv(T_chain_end) is distributed
      linearly across the chain: T_k' = ((1-a) I + a D) @ T_k, a = k/n.

Renders both the normalized-only chain and the loop-corrected chain as
panoramas and reports canvas sizes. Reads bags read-only.

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
    if normalized:
        T1, T2 = hartley_T(p_b), hartley_T(p_a)
        Hn, mask = cv2.findHomography(
            apply_T(T1, p_b).reshape(-1, 1, 2),
            apply_T(T2, p_a).reshape(-1, 1, 2), cv2.RANSAC, RANSAC)
        if Hn is None:
            return None, 0
        H = np.linalg.inv(T2) @ Hn @ T1
    else:
        H, mask = cv2.findHomography(p_b, p_a, cv2.RANSAC, RANSAC)
        if H is None or mask is None:
            return None, 0
    return H, int(mask.sum())


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
    bw, bh, x0, y0 = chain_canvas(Ts)
    if bw > max_px or bh > max_px:
        print(f"render skipped: canvas {bw:.0f} x {bh:.0f} px too large",
              flush=True)
        return None
    cw, ch = int(np.ceil(bw)), int(np.ceil(bh))
    shift = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1.0]])
    acc = np.zeros((ch, cw, 3), dtype=np.float64)
    cover = np.zeros((ch, cw), dtype=np.float64)
    ones = np.ones((H, W), dtype=np.float64)
    for f, T in zip(frames, Ts):
        M = shift @ T
        warped = cv2.warpPerspective(f, M, (cw, ch))
        mask = cv2.warpPerspective(ones, M, (cw, ch))
        acc += warped.astype(np.float64)
        cover += mask
    cover[cover == 0] = 1
    img = (acc / cover[:, :, None]).astype(np.uint8)
    cv2.imwrite(out_path, img)
    print(f"wrote {out_path} {cw}x{ch}", flush=True)
    return (cw, ch)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    args = ap.parse_args()

    frames, feats = load_frames(args.bag, args.stride)
    matcher = cv2.BFMatcher()
    n = len(frames)

    # (a) normalized chain, frame k -> frame 0
    Hs, pair_inliers = [], []
    for k in range(1, n):
        Hm, inl = match_h(feats[k - 1][0], feats[k - 1][1],
                           feats[k][0], feats[k][1], matcher, True)
        if Hm is None:
            print(f"pair {k-1}->{k} failed (inliers {inl})", flush=True)
            return
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
    for a in range(5):
        for b in range(n - 5, n):
            Hm, inl = match_h(feats[a][0], feats[a][1],
                               feats[b][0], feats[b][1], matcher, True)
            loop.append((inl, a, b, Hm))
    loop_ok = [x for x in loop if x[3] is not None]
    print("loop pairs (first5 x last5) inliers:",
          sorted([x[0] for x in loop_ok], reverse=True), flush=True)
    best = max(loop_ok, key=lambda x: x[0])
    inl, a, b, H_loop = best
    print(f"best loop pair: frame {b} -> frame {a}, inliers {inl}",
          flush=True)

    # error D maps chain-predicted frame-0 coords to measured ones
    T_end = Ts[-1]
    D = H_loop @ np.linalg.inv(T_end)
    D = D / D[2, 2]
    print(f"loop error D Frobenius norm {np.linalg.norm(D - np.eye(3)):.3f}",
          flush=True)
    # singularity check along the interpolation path
    min_det = min(abs(np.linalg.det((1 - t) * np.eye(3) + t * D))
                  for t in np.linspace(0, 1, 21))
    print(f"min |det| along correction path: {min_det:.4f}", flush=True)

    corrected = None
    if min_det > 0.05:
        Ts_corr = []
        for k, T in enumerate(Ts):
            alpha = k / (n - 1)
            C = (1 - alpha) * np.eye(3) + alpha * D
            Ts_corr.append(C @ T)
        bw2, bh2, _, _ = chain_canvas(Ts_corr)
        print(f"loop-corrected chain: canvas {bw2:.0f} x {bh2:.0f} px",
              flush=True)
        corrected = Ts_corr
    else:
        print("correction path passes near a singularity; "
              "not distributing", flush=True)

    render(frames, Ts, args.out + "_norm_only.png")
    if corrected is not None:
        render(frames, corrected, args.out + "_loop_corrected.png")

    json.dump({
        "bag": os.path.basename(args.bag),
        "stride": args.stride, "n_frames": n,
        "pair_inliers_median": int(np.median(pair_inliers)),
        "pair_inliers_min": int(min(pair_inliers)),
        "norm_only_canvas_px": [round(bw, 1), round(bh, 1)],
        "loop_pair_inliers": sorted([x[0] for x in loop_ok], reverse=True),
        "best_loop": {"first_frame": a, "last_frame": b, "inliers": inl},
        "loop_error_fro": float(np.linalg.norm(D - np.eye(3))),
        "min_det_path": float(min_det),
        "loop_corrected_canvas_px": ([round(bw2, 1), round(bh2, 1)]
                                     if corrected is not None else None),
    }, open(args.out + "_stable.json", "w"), indent=2)
    print("done", flush=True)


if __name__ == "__main__":
    main()
