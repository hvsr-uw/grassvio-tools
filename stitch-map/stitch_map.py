#!/usr/bin/env python3
"""Stitch all the frames of a GrassVIO recording into one big map (mosaic).

For the SLAM side of the project we need a map built from what the camera
sees. This script takes one recording, lines up its frames using feature
matches between neighbouring frames, and pastes them onto one big canvas.
The camera looks down at a mostly flat ground plane, so a homography is the
right transform between frames.

Inputs (pick one):
  --bag  take.db3     a ROS bag file from our recordings (read directly,
                      no ROS install needed, same as the other tools)
  --video file.mp4    any normal video file
  --frames folder     a folder of frame images (frame_000000.png, ...)

Usage:
    python3 stitch_map.py --bag <take>.db3 --out map.png

Options:
    --stride N      use every Nth frame (default 5). Bigger stride means a
                    bigger step between frames, which makes a wider map but
                    harder matching.
    --detector X    sift or orb (default sift).
    --max-frames N  stop after N frames (default: no limit).
    --loop-check    also match each frame against earlier frames at least
                    5 chosen frames back. A strong loop match (at least 50
                    inliers) re-anchors the frame: its transform comes from
                    the loop match instead of the drifted chain.

Output: the map image plus a small .json file next to it with the run stats.
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

RANSAC_THRESH_PX = 3.0
MIN_INLIERS = 20
LOOP_REANCHOR_MIN = 50
# A loop match must be to a frame at least this many chosen frames back.
# Closer frames still overlap from normal forward motion, so matching them
# is not a loop closure, it is just a shorter hop along the chain.
LOOP_MIN_GAP = 5


def load_frames_from_bag(bag_path):
    """Decode all colour frames of a .db3 bag, in timestamp order."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (COLOR,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    db.close()
    frames = []
    for (blob,) in rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
            p["height"], p["width"], 3)
        frames.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return frames


def load_frames_from_video(video_path):
    cap = cv2.VideoCapture(video_path)
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(frame)
    cap.release()
    return frames


def load_frames_from_folder(folder):
    names = sorted(f for f in os.listdir(folder)
                   if f.lower().endswith((".png", ".jpg", ".jpeg")))
    return [cv2.imread(os.path.join(folder, n)) for n in names]


def make_detector(name):
    if name == "orb":
        return cv2.ORB_create(nfeatures=3000)
    # Cap the feature count: the matcher cannot handle more than about
    # 32000 descriptors per image, and grass easily produces more.
    return cv2.SIFT_create(nfeatures=10000)


def downscale(frame, max_dim=1600):
    """Shrink very large photos so matching stays fast and reliable."""
    h, w = frame.shape[:2]
    scale = min(1.0, max_dim / max(h, w))
    if scale >= 1.0:
        return frame
    return cv2.resize(frame, (round(w * scale), round(h * scale)),
                      interpolation=cv2.INTER_AREA)


def pair_homography(feats_cur, feats_next, matcher):
    """Homography that maps points in the next frame into the current frame.

    feats are (keypoints, descriptors) pairs already computed for each
    frame. Returns (H, inlier_count) or (None, 0) when the match is too weak.
    """
    kp_cur, des_cur = feats_cur
    kp_next, des_next = feats_next
    if des_cur is None or des_next is None:
        return None, 0
    matches = matcher.knnMatch(des_next, des_cur, k=2)
    good = []
    for pair in matches:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < 0.75 * n.distance:
            good.append(m)
    if len(good) < MIN_INLIERS:
        return None, 0
    src = np.float32([kp_next[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_cur[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH_PX)
    if H is None or mask is None:
        return None, 0
    inliers = int(mask.sum())
    if inliers < MIN_INLIERS:
        return None, 0
    return H, inliers


def stitch(frames, stride, detector_name, loop_check=False):
    """Chain pairwise homographies and paste every frame onto one canvas.

    With loop_check on, each frame is also matched against earlier frames
    at least LOOP_MIN_GAP chosen frames back. A strong loop match (at
    least 50 inliers) re-anchors the frame: its transform comes from the
    loop match instead of the drifted chain, and later frames chain from
    there. Neighbour pair stats keep their meaning; loop matches are
    reported separately in the stats.
    """
    detector = make_detector(detector_name)
    if detector_name == "orb":
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    else:
        matcher = cv2.BFMatcher(cv2.NORM_L2)
    frames = [downscale(f) for f in frames]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]

    chosen = list(range(0, len(frames), stride))
    feats = {idx: detector.detectAndCompute(grays[idx], None)
             for idx in chosen}
    # transforms[i] maps frame chosen[i] into frame chosen[0] coordinates.
    transforms = [np.eye(3)]
    inlier_counts = []
    skipped = 0
    loop_matches = []
    for pos in range(1, len(chosen)):
        a, b = chosen[pos - 1], chosen[pos]
        H, inliers = pair_homography(feats[a], feats[b], matcher)
        if H is None:
            skipped += 1
            transforms.append(transforms[-1].copy())
        else:
            inlier_counts.append(inliers)
            transforms.append(transforms[-1] @ H)
        if loop_check:
            best_j, best_H, best_n = None, None, 0
            for jpos in range(pos - LOOP_MIN_GAP + 1):
                j = chosen[jpos]
                Hj, nj = pair_homography(feats[j], feats[b], matcher)
                if Hj is not None and nj > best_n:
                    best_j, best_H, best_n = jpos, Hj, nj
            if best_j is not None:
                reanchored = best_n >= LOOP_REANCHOR_MIN
                if reanchored:
                    transforms[pos] = transforms[best_j] @ best_H
                loop_matches.append({
                    "frame": pos,
                    "matched_frame": best_j,
                    "inliers": best_n,
                    "reanchored": reanchored,
                })

    h, w = frames[0].shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_pts = []
    for T in transforms:
        all_pts.append(cv2.perspectiveTransform(corners, T))
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

    stats = {
        "frames_total": len(frames),
        "frames_used": len(chosen),
        "pairs_skipped": skipped,
        "median_inliers": (float(np.median(inlier_counts))
                           if inlier_counts else 0.0),
        "canvas_width_px": int(canvas_w),
        "canvas_height_px": int(canvas_h),
        "stride": stride,
        "detector": detector_name,
        "loop_check": loop_check,
        "loop_matches": loop_matches,
    }
    return mosaic, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag")
    src.add_argument("--video")
    src.add_argument("--frames")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--detector", choices=("sift", "orb"), default="sift")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--loop-check", action="store_true")
    args = ap.parse_args()

    if args.bag:
        frames = load_frames_from_bag(args.bag)
    elif args.video:
        frames = load_frames_from_video(args.video)
    else:
        frames = load_frames_from_folder(args.frames)
    if args.max_frames:
        frames = frames[:args.max_frames]
    if len(frames) < 2:
        print("not enough frames to stitch", file=sys.stderr)
        return 1

    mosaic, stats = stitch(frames, args.stride, args.detector,
                           loop_check=args.loop_check)
    cv2.imwrite(args.out, mosaic)
    stats_path = os.path.splitext(args.out)[0] + ".json"
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out} and {stats_path}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
