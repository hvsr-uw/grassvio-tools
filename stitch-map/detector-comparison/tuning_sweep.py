#!/usr/bin/env python3
"""Parameter sweep for the stitch matcher (experiment script, not shared).

Reuses the stitch script's loaders and detectors, but precomputes
keypoints/descriptors once per frame per detector and then varies the
matching parameters: Lowe ratio, RANSAC reprojection threshold, and
minimum inliers. For each setting it reports pairs skipped and median
inliers, the same two numbers the stitch script's stats file carries.

Run: python3 tuning_sweep.py
Inputs: the September bag and the phone photo folder.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "dataset-tools", "stitch-map"))
import stitch_map as sm  # noqa: E402


def pair_stats(kp_a, des_a, kp_b, des_b, matcher, ratio, ransac, min_in):
    """Match frame b into frame a. Returns inlier count or None if skipped."""
    if des_a is None or des_b is None:
        return None
    matches = matcher.knnMatch(des_b, des_a, k=2)
    good = []
    for pair in matches:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append(m)
    if len(good) < min_in:
        return None
    src = np.float32([kp_b[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_a[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransac)
    if H is None or mask is None:
        return None
    inliers = int(mask.sum())
    if inliers < min_in:
        return None
    return inliers


def sweep(name, frames, stride, detectors):
    frames = [sm.downscale(f) for f in frames]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    chosen = list(range(0, len(frames), stride))
    pairs = list(zip(chosen[:-1], chosen[1:]))
    print(f"=== {name}: {len(chosen)} frames, {len(pairs)} pairs ===")
    results = []
    for det_name in detectors:
        detector = sm.make_detector(det_name)
        matcher = (cv2.BFMatcher(cv2.NORM_HAMMING) if det_name == "orb"
                   else cv2.BFMatcher(cv2.NORM_L2))
        feats = [detector.detectAndCompute(g, None) for g in grays]
        for ratio in (0.6, 0.75, 0.85):
            for ransac in (1.0, 3.0, 5.0, 8.0):
                counts = []
                skipped = 0
                for a, b in pairs:
                    kp_a, des_a = feats[a]
                    kp_b, des_b = feats[b]
                    n = pair_stats(kp_a, des_a, kp_b, des_b, matcher,
                                   ratio, ransac, 20)
                    if n is None:
                        skipped += 1
                    else:
                        counts.append(n)
                med = float(np.median(counts)) if counts else 0.0
                row = {"input": name, "detector": det_name, "ratio": ratio,
                       "ransac_px": ransac, "pairs_skipped": skipped,
                       "median_inliers": med}
                results.append(row)
                print(f"{det_name} ratio={ratio} ransac={ransac}: "
                      f"skipped {skipped}/{len(pairs)}, median {med}")
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", help="a .db3 recording to sweep")
    ap.add_argument("--frames", help="a folder of frame images to sweep")
    args = ap.parse_args()
    out = []
    if args.bag:
        bag_frames = sm.load_frames_from_bag(args.bag)
        out += sweep("bag-take", bag_frames, 5, ("sift", "orb"))
    if args.frames:
        phone_frames = sm.load_frames_from_folder(args.frames)
        out += sweep("frame-folder", phone_frames, 1, ("sift", "orb"))
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "tuning_sweep.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
