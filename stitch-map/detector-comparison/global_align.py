#!/usr/bin/env python3
"""Global alignment for stitch maps by iterative corner averaging.

Takes a pairs file from a tree run (every frame pair with its homography)
and the source frames, starts from the tree placement, and then repeatedly
moves every frame toward the consensus of all its pair constraints: each
pair predicts where a frame's corners should land, those predictions are
averaged, and a new homography is fit from the frame's own corners to the
averaged corners. Frames with no usable pairs keep the tree transform.

This spreads the error over all constraints instead of letting it pile
up along one path, which is what causes the far end stretch.

Usage:
    python3 global_align.py --frames <folder> --pairs <pairs.json> \
        --stride 1 --out map.png
    python3 global_align.py --bag <take>.db3 --pairs <pairs.json> \
        --stride 3 --out map.png
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "dataset-tools", "stitch-map"))
from stitch_map import (MIN_INLIERS, downscale,  # noqa: E402
                        load_frames_from_bag, load_frames_from_folder,
                        load_frames_from_video, place_by_tree)


def apply_homography(T, pts):
    """Apply a 3x3 homography to (N, 2) points, returns (N, 2) points."""
    homog = np.concatenate([pts, np.ones((len(pts), 1))], axis=1)
    out = homog @ T.T
    return out[:, :2] / out[:, 2:3]


def mean_residual(transforms, edges, corners):
    """Mean corner distance over all pair constraints, in pixels."""
    if not edges:
        return 0.0
    total = 0.0
    for _, i, j, H in edges:
        # Constraint: T_j should equal T_i @ H.
        a = apply_homography(transforms[j], corners)
        b = apply_homography(transforms[i] @ H, corners)
        total += float(np.mean(np.linalg.norm(a - b, axis=1)))
    return total / len(edges)


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag")
    src.add_argument("--video")
    src.add_argument("--frames")
    ap.add_argument("--pairs", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--iterations", type=int, default=30)
    args = ap.parse_args()

    if args.bag:
        frames = load_frames_from_bag(args.bag)
    elif args.video:
        frames = load_frames_from_video(args.video)
    else:
        frames = load_frames_from_folder(args.frames)
    frames = [downscale(f) for f in frames]
    chosen = list(range(0, len(frames), stride))
    n = len(chosen)

    with open(args.pairs) as f:
        saved = json.load(f)
    pairs = saved.get("pairs", {})
    edges = []
    for key, entry in pairs.items():
        if entry["H"] is None or entry["inliers"] < MIN_INLIERS:
            continue
        i, j = (int(v) for v in key.split(","))
        edges.append((entry["inliers"], i, j,
                      np.array(entry["H"], dtype=float)))
    if n != saved.get("meta", {}).get("n_chosen", n):
        print("warning: pairs file was built for a different frame count",
              file=sys.stderr)

    transforms, _, _, unplaced = place_by_tree(n, edges)
    transforms = [T.astype(float) for T in transforms]
    h, w = frames[0].shape[:2]
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=float)

    neighbours = {}
    for _, i, j, H in edges:
        neighbours.setdefault(i, []).append((j, H))
        neighbours.setdefault(j, []).append((i, H))

    residual_before = mean_residual(transforms, edges, corners)
    for _ in range(args.iterations):
        new_transforms = [T.copy() for T in transforms]
        for pos in range(1, n):
            if pos in unplaced:
                continue
            preds = [apply_homography(transforms[pos], corners)]
            for other, H in neighbours.get(pos, []):
                if other < pos:
                    # Pair (other, pos): H maps pos into other, so the
                    # constraint gives T_pos = T_other @ H.
                    preds.append(apply_homography(
                        transforms[other] @ H, corners))
                else:
                    # Pair (pos, other): H maps other into pos, so the
                    # constraint gives T_pos = T_other @ inv(H).
                    try:
                        preds.append(apply_homography(
                            transforms[other] @ np.linalg.inv(H), corners))
                    except np.linalg.LinAlgError:
                        continue
            if len(preds) < 2:
                continue
            target = np.mean(preds, axis=0).astype(np.float32)
            fitted = cv2.getPerspectiveTransform(
                corners.astype(np.float32), target)
            if fitted is not None:
                new_transforms[pos] = fitted.astype(float)
        transforms = new_transforms
    residual_after = mean_residual(transforms, edges, corners)

    all_pts = []
    for T in transforms:
        all_pts.append(cv2.perspectiveTransform(
            corners.astype(np.float32).reshape(-1, 1, 2),
            T.astype(float)))
    all_pts = np.concatenate(all_pts, axis=0)
    x_min, y_min = np.floor(all_pts.min(axis=0)[0]).astype(int)
    x_max, y_max = np.ceil(all_pts.max(axis=0)[0]).astype(int)
    shift = np.array([[1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]], dtype=float)
    canvas_w, canvas_h = x_max - x_min, y_max - y_min

    acc = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
    cover = np.zeros((canvas_h, canvas_w), dtype=np.float64)
    ones = np.ones((h, w), dtype=np.float64)
    for idx, T in zip(chosen, transforms):
        M = shift @ T
        warped = cv2.warpPerspective(frames[idx], M, (canvas_w, canvas_h))
        mask = cv2.warpPerspective(ones, M, (canvas_w, canvas_h))
        acc += warped.astype(np.float64) * mask[..., None]
        cover += mask
    cover[cover == 0] = 1
    mosaic = (acc / cover[..., None]).astype(np.uint8)

    cv2.imwrite(args.out, mosaic)
    stats = {
        "frames_total": len(frames),
        "frames_used": n,
        "frames_unplaced": unplaced,
        "pairs_used": len(edges),
        "iterations": args.iterations,
        "residual_before_px": residual_before,
        "residual_after_px": residual_after,
        "canvas_width_px": int(canvas_w),
        "canvas_height_px": int(canvas_h),
        "stride": args.stride,
        "placement": "global",
    }
    stats_path = os.path.splitext(args.out)[0] + ".json"
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out} and {stats_path}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
