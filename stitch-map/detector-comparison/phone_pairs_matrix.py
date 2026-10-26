#!/usr/bin/env python3
"""All-pairs SIFT match matrix for the phone photos.

For every pair of photos (i, j) with i < j, match frame j against frame i
with a RANSAC homography fit and print the inlier count. This shows which
non-neighbour constraints exist for frames whose neighbour pair fails.

Usage:
    python3 phone_pairs_matrix.py --frames <folder>
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "dataset-tools", "stitch-map"))
from stitch_map import downscale, make_detector  # noqa: E402


def pair_inliers(feats_i, feats_j, matcher, ratio):
    kp_i, des_i = feats_i
    kp_j, des_j = feats_j
    if des_i is None or des_j is None:
        return 0
    matches = matcher.knnMatch(des_j, des_i, k=2)
    good = []
    for pair in matches:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append(m)
    if len(good) < 4:
        return 0
    src = np.float32([kp_j[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_i[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None or mask is None:
        return 0
    return int(mask.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    args = ap.parse_args()
    paths = sorted(glob.glob(os.path.join(args.frames, "*.*")))
    frames = [downscale(cv2.imread(p)) for p in paths]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    detector = make_detector("sift")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    feats = [detector.detectAndCompute(g, None) for g in grays]
    n = len(frames)
    for ratio in (0.75, 0.85):
        print(f"=== Lowe ratio {ratio} (rows: earlier frame, cols: later frame) ===")
        print("      " + " ".join(f"{j:5d}" for j in range(n)))
        for i in range(n):
            row = []
            for j in range(n):
                if j <= i:
                    row.append("    .")
                else:
                    row.append(f"{pair_inliers(feats[i], feats[j], matcher, ratio):5d}")
            print(f"{i:5d} " + " ".join(row))


if __name__ == "__main__":
    main()
