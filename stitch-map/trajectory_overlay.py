#!/usr/bin/env python3
"""Draw the camera's path on top of the stitched panorama.

Each frame lands on the panorama through one chained homography, so mapping
each frame's centre through its own chain transform gives the camera's path
in panorama pixels. This script builds the panorama exactly like
stitch_map.py (same matching, same warping) and then draws the path as a
polyline with start and end markers.

Inputs and options mirror stitch_map.py:
  --bag / --video / --frames, --stride N, --detector sift|orb.

Usage:
    python3 trajectory_overlay.py --bag <take>.db3 --out map_with_path.png

The path is only as good as the chain: where chained homographies drift,
the drawn path drifts with them. On takes where the chain blows up, the
path is meaningless. Tree placement also yields one transform per frame,
so the same drawing step works there too.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

import stitch_map


def chain_transforms(frames, stride, detector_name):
    """Run the neighbour homography chain, same as stitch_map's default.

    Returns (chosen, frames, transforms, skipped, median_inliers) where
    transforms[i] maps frame chosen[i] into the first frame's coordinates.
    """
    detector = stitch_map.make_detector(detector_name)
    if detector_name == "orb":
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    else:
        matcher = cv2.BFMatcher(cv2.NORM_L2)
    frames = [stitch_map.downscale(f) for f in frames]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]

    chosen = list(range(0, len(frames), stride))
    feats = {idx: detector.detectAndCompute(grays[idx], None)
             for idx in chosen}
    transforms = [np.eye(3)]
    inlier_counts = []
    skipped = 0
    for pos in range(1, len(chosen)):
        a, b = chosen[pos - 1], chosen[pos]
        H, inliers = stitch_map.pair_homography(feats[a], feats[b], matcher)
        if H is None:
            skipped += 1
            transforms.append(transforms[-1].copy())
        else:
            inlier_counts.append(inliers)
            transforms.append(transforms[-1] @ H)
    median_inliers = (float(np.median(inlier_counts))
                      if inlier_counts else 0.0)
    return chosen, frames, transforms, skipped, median_inliers


def build_mosaic(frames, chosen, transforms):
    """Paste every frame onto one canvas, exactly like stitch_map.stitch.

    Returns (mosaic, shift) where shift moves frame-0 coordinates into
    canvas pixels, so shift @ transforms[i] is the warp used for frame i.
    """
    h, w = frames[0].shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_pts = np.concatenate(
        [cv2.perspectiveTransform(corners, T) for T in transforms], axis=0)
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
    return mosaic, shift


def trajectory_points(frames, chosen, transforms, shift):
    """Frame centres mapped into panorama pixels, in time order."""
    pts = []
    for idx, T in zip(chosen, transforms):
        h, w = frames[idx].shape[:2]
        centre = np.float32([[[w / 2.0, h / 2.0]]])
        p = cv2.perspectiveTransform(centre, shift @ T)[0][0]
        pts.append(p)
    return np.array(pts)


def draw_trajectory(mosaic, pts):
    """Draw the path as a polyline with start and end markers.

    The path gets a dark outline so it reads on bright grass, and the
    two labels sit on opposite sides of their markers so they cannot
    overlap when the camera ends near where it started.
    """
    out = mosaic.copy()
    canvas_h, canvas_w = out.shape[:2]
    thick = max(3, int(min(canvas_w, canvas_h) / 150))
    ipts = np.round(pts).astype(int).reshape(-1, 1, 2)

    # Magenta: the complement of grass green, so the path reads on any lawn.
    cv2.polylines(out, [ipts], False, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.polylines(out, [ipts], False, (255, 0, 255), thick, cv2.LINE_AA)

    font = cv2.FONT_HERSHEY_SIMPLEX
    fscale = max(0.5, min(canvas_w, canvas_h) / 500.0)
    fthick = max(1, thick // 2)
    r = thick + 4
    for label, p, colour, below in (("start", ipts[0][0], (0, 200, 0), False),
                                    ("end", ipts[-1][0], (0, 0, 255), True)):
        x, y = int(p[0]), int(p[1])
        cv2.circle(out, (x, y), r + 2, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(out, (x, y), r, colour, -1, cv2.LINE_AA)
        (tw, th), _ = cv2.getTextSize(label, font, fscale, fthick)
        tx = min(max(x - tw // 2, 2), canvas_w - tw - 2)
        ty = y - r - 4 if not below else y + r + 4 + th
        ty = min(max(ty, th + 2), canvas_h - 2)
        cv2.putText(out, label, (tx, ty), font, fscale, (0, 0, 0),
                    fthick + 1, cv2.LINE_AA)
        cv2.putText(out, label, (tx, ty), font, fscale, (255, 255, 255),
                    fthick, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag")
    src.add_argument("--video")
    src.add_argument("--frames")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--detector", choices=("sift", "orb"), default="sift")
    args = ap.parse_args()

    if args.bag:
        frames = stitch_map.load_frames_from_bag(args.bag)
    elif args.video:
        frames = stitch_map.load_frames_from_video(args.video)
    else:
        frames = stitch_map.load_frames_from_folder(args.frames)
    if len(frames) < 2:
        print("not enough frames to stitch", file=sys.stderr)
        return 1

    chosen, frames, transforms, skipped, median_inliers = chain_transforms(
        frames, args.stride, args.detector)
    mosaic, shift = build_mosaic(frames, chosen, transforms)
    pts = trajectory_points(frames, chosen, transforms, shift)
    overlay = draw_trajectory(mosaic, pts)

    cv2.imwrite(args.out, overlay)
    stats_path = os.path.splitext(args.out)[0] + ".json"
    stats = {
        "frames_total": len(frames),
        "frames_used": len(chosen),
        "pairs_skipped": skipped,
        "median_inliers": median_inliers,
        "canvas_width_px": int(overlay.shape[1]),
        "canvas_height_px": int(overlay.shape[0]),
        "trajectory_points": len(pts),
        "stride": args.stride,
        "detector": args.detector,
    }
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out} and {stats_path}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
