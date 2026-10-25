#!/usr/bin/env python3
"""Loop-closure check for the phone photos (experiment script, not shared).

The stitch script only matches neighbouring frames. This script matches
every frame against every earlier frame with the same matcher settings
(ratio 0.75, RANSAC 3 px, minimum 20 inliers) and reports the inlier count
for each pair. The question from the meeting: does the sixth photo really
overlap the first one, and would checking older frames find usable matches
the chain currently ignores?

Run: python3 loop_check.py
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "dataset-tools", "stitch-map"))
import stitch_map as sm  # noqa: E402


def pair_inliers(kp_a, des_a, kp_b, des_b, matcher):
    if des_a is None or des_b is None:
        return 0
    matches = matcher.knnMatch(des_b, des_a, k=2)
    good = []
    for pair in matches:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < 0.75 * n.distance:
            good.append(m)
    if len(good) < 20:
        return 0
    src = np.float32([kp_b[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_a[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None or mask is None:
        return 0
    return int(mask.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True,
                    help="a folder of frame images, in order")
    args = ap.parse_args()
    names = sorted(f for f in os.listdir(args.frames)
                   if f.lower().endswith((".png", ".jpg", ".jpeg")))
    frames = [sm.downscale(cv2.imread(os.path.join(args.frames, n)))
              for n in names]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    for det_name in ("sift", "orb"):
        detector = sm.make_detector(det_name)
        matcher = (cv2.BFMatcher(cv2.NORM_HAMMING) if det_name == "orb"
                   else cv2.BFMatcher(cv2.NORM_L2))
        feats = [detector.detectAndCompute(g, None) for g in grays]
        print(f"=== {det_name} ===")
        for j in range(len(frames)):
            row = []
            for i in range(j):
                n = pair_inliers(feats[i][0], feats[i][1],
                                 feats[j][0], feats[j][1], matcher)
                row.append(f"{i + 1}:{n}")
            print(f"frame {j + 1} ({names[j]}) vs earlier -> "
                  + ", ".join(row))


if __name__ == "__main__":
    main()
