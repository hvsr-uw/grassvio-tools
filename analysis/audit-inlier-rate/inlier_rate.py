#!/usr/bin/env python3
"""Measure the RANSAC inlier rate on GrassVIO recordings.

Mirrors dataset-tools/stitch-map/stitch_map.py exactly:
SIFT nfeatures=10000, BFMatcher L2, knnMatch k=2, Lowe ratio 0.75,
findHomography RANSAC threshold 3.0 px, Hartley normalization on
(current pipeline default). Also computes the direct-fit variant as a
sensitivity check (same pairs, same matches, normalization off).

For each sampled consecutive stride-5 pair:
  n_knn    = knn matches with 2 neighbours (all matches attempted)
  n_good   = after Lowe 0.75
  n_inlier = RANSAC inliers (0 when n_good < 20, mirroring the pipeline,
             which rejects such pairs before RANSAC)
  rate_good = n_inlier / n_good   (headline)
  rate_all  = n_inlier / n_knn    (second definition)

Usage:
    python3 inlier_rate.py --bag <take>.db3 --out results/<stamp>.json
                           [--max-pairs N] [--stride 5]

Writes one JSON per take as it finishes (checkpoint-friendly).
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "dataset-tools",
                                "stitch-map"))
from stitch_map import (  # noqa: E402
    load_frames_from_bag, make_detector, downscale, fit_homography,
    RANSAC_THRESH_PX, MIN_INLIERS, LOWE_RATIO,
)

STRIDE = 5


def pair_stats(kp_a, des_a, kp_b, des_b, matcher):
    """Match frame b into frame a. Returns dict of counts and rates."""
    if des_a is None or des_b is None:
        return {"n_knn": 0, "n_good": 0, "inliers_norm": 0,
                "inliers_direct": 0, "rate_good_norm": 0.0,
                "rate_good_direct": 0.0, "rate_all_norm": 0.0,
                "rate_all_direct": 0.0, "failed": True}
    matches = matcher.knnMatch(des_b, des_a, k=2)
    pairs = [p for p in matches if len(p) == 2]
    n_knn = len(pairs)
    good = [p[0] for p in pairs if p[0].distance < LOWE_RATIO * p[1].distance]
    n_good = len(good)
    if n_good < MIN_INLIERS:
        return {"n_knn": n_knn, "n_good": n_good, "inliers_norm": 0,
                "inliers_direct": 0, "rate_good_norm": 0.0,
                "rate_good_direct": 0.0, "rate_all_norm": 0.0,
                "rate_all_direct": 0.0, "failed": True}
    src = np.float32([kp_b[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_a[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    out = {"n_knn": n_knn, "n_good": n_good, "failed": False}
    for key, norm in (("norm", True), ("direct", False)):
        _, mask = fit_homography(src, dst, normalize=norm)
        n_in = int(mask.sum()) if mask is not None else 0
        out[f"inliers_{key}"] = n_in
        out[f"rate_good_{key}"] = n_in / n_good if n_good else 0.0
        out[f"rate_all_{key}"] = n_in / n_knn if n_knn else 0.0
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=STRIDE)
    ap.add_argument("--max-pairs", type=int, default=40)
    args = ap.parse_args()

    frames = load_frames_from_bag(args.bag)
    frames = [downscale(f) for f in frames]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    chosen = list(range(0, len(frames), args.stride))
    # Consecutive chosen pairs; sample evenly if more than max-pairs.
    all_pairs = [(chosen[i], chosen[i + 1]) for i in range(len(chosen) - 1)]
    if len(all_pairs) > args.max_pairs:
        idx = np.linspace(0, len(all_pairs) - 1, args.max_pairs).round()
        idx = sorted(set(int(i) for i in idx))
        pairs = [all_pairs[i] for i in idx]
    else:
        pairs = all_pairs

    detector = make_detector("sift")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    feats = {}
    for idx in sorted({f for p in pairs for f in p}):
        feats[idx] = detector.detectAndCompute(grays[idx], None)

    rows = []
    for a, b in pairs:
        kp_a, des_a = feats[a]
        kp_b, des_b = feats[b]
        st = pair_stats(kp_a, des_a, kp_b, des_b, matcher)
        st["frame_a"] = a
        st["frame_b"] = b
        rows.append(st)

    result = {
        "bag": os.path.basename(args.bag),
        "frames_total": len(frames),
        "frames_chosen": len(chosen),
        "stride": args.stride,
        "pairs_sampled": len(rows),
        "pairs_available": len(all_pairs),
        "method": {
            "detector": "SIFT nfeatures=10000",
            "matcher": "BFMatcher L2, knnMatch k=2",
            "lowe_ratio": LOWE_RATIO,
            "ransac_thresh_px": RANSAC_THRESH_PX,
            "min_inliers": MIN_INLIERS,
            "normalize_headline": True,
            "rate_good": "inliers / good matches after Lowe 0.75",
            "rate_all": "inliers / all knn matches with 2 neighbours",
        },
        "pairs": rows,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    med = float(np.median([r["rate_good_norm"] for r in rows])) if rows else 0.0
    print(f"{os.path.basename(args.bag)}: {len(rows)} pairs, "
          f"median rate_good_norm={med:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
