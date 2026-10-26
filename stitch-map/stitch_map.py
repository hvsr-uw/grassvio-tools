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
    --place X       chain or tree (default chain). Chain places each frame
                    from its neighbour only. Tree builds the strongest
                    spanning tree over all frame pairs and places each frame
                    along its strongest path back to the first frame, so a
                    frame whose neighbour pair fails can still be placed
                    from a later frame that matches it well. Tree mode
                    saves every pair result to <out>_pairs.json as it goes,
                    and resumes from that file if the run is interrupted,
                    instead of starting over.
    --lowe-ratio R  Lowe ratio for the matcher (default 0.75). Looser
                    values accept more matches and can rescue a weak pair.

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
LOWE_RATIO = 0.75
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


def pair_homography(feats_cur, feats_next, matcher, ratio=LOWE_RATIO):
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
        if m.distance < ratio * n.distance:
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


def place_by_tree(n, edges):
    """Place every frame along the strongest spanning tree over all pairs.

    edges is a list of (inliers, i, j, H) where H maps frame j into frame i
    and i < j. Prim's algorithm from frame 0 always takes the strongest
    edge that reaches a new frame, so a frame whose neighbour pair fails
    can still be placed from any frame that matches it well, including a
    later frame. Returns (transforms, edge_inliers, edges_used, unplaced),
    where transforms[i] maps frame i into frame 0 coordinates.
    """
    by_frame = {}
    for n_inliers, i, j, H in edges:
        by_frame.setdefault(i, []).append((n_inliers, j, H))
        by_frame.setdefault(j, []).append((n_inliers, i, H))
    transforms = [None] * n
    transforms[0] = np.eye(3)
    reached = {0}
    banned = set()
    edge_inliers = []
    edges_used = []
    while len(reached) < n:
        best = None
        for r in reached:
            for n_inliers, other, H in by_frame.get(r, []):
                if other not in reached and (r, other) not in banned and (
                        best is None or n_inliers > best[0]):
                    best = (n_inliers, r, other, H)
        if best is None:
            break
        n_inliers, r, other, H = best
        if r < other:
            # H maps other into r, and r is already placed.
            transforms[other] = transforms[r] @ H
        else:
            # Edge runs the other way: invert it to place other from r.
            try:
                transforms[other] = transforms[r] @ np.linalg.inv(H)
            except np.linalg.LinAlgError:
                banned.add((r, other))
                continue
        reached.add(other)
        edge_inliers.append(n_inliers)
        edges_used.append({"from": r, "to": other, "inliers": n_inliers})
    unplaced = []
    for pos in range(1, n):
        if transforms[pos] is None:
            transforms[pos] = transforms[pos - 1].copy()
            unplaced.append(pos)
    return transforms, edge_inliers, edges_used, unplaced


def stitch(frames, stride, detector_name, loop_check=False, place="chain",
           lowe_ratio=LOWE_RATIO, pairs_checkpoint=None):
    """Chain pairwise homographies and paste every frame onto one canvas.

    With loop_check on, each frame is also matched against earlier frames
    at least LOOP_MIN_GAP chosen frames back. A strong loop match (at
    least 50 inliers) re-anchors the frame: its transform comes from the
    loop match instead of the drifted chain, and later frames chain from
    there. Neighbour pair stats keep their meaning; loop matches are
    reported separately in the stats.

    With place="tree", every frame pair is matched and frames are placed
    along the strongest spanning tree back to the first frame instead of
    along the neighbour chain.
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
    loop_matches = []
    tree_edges = []
    pairs_from_checkpoint = 0
    if place == "tree":
        ckpt_meta = {"detector": detector_name, "lowe_ratio": lowe_ratio,
                     "n_chosen": len(chosen), "stride": stride}
        ckpt = {}
        if pairs_checkpoint and os.path.exists(pairs_checkpoint):
            try:
                with open(pairs_checkpoint) as f:
                    saved = json.load(f)
                if saved.get("meta") == ckpt_meta:
                    ckpt = saved.get("pairs", {})
            except (OSError, ValueError):
                ckpt = {}
        edges = []
        dirty = 0
        for ipos in range(len(chosen)):
            for jpos in range(ipos + 1, len(chosen)):
                key = f"{ipos},{jpos}"
                if key in ckpt:
                    entry = ckpt[key]
                    pairs_from_checkpoint += 1
                    if (entry["H"] is not None
                            and entry["inliers"] >= MIN_INLIERS):
                        edges.append((entry["inliers"], ipos, jpos,
                                      np.array(entry["H"], dtype=float)))
                    continue
                H, n_inliers = pair_homography(
                    feats[chosen[ipos]], feats[chosen[jpos]], matcher,
                    ratio=lowe_ratio)
                ckpt[key] = {"inliers": n_inliers,
                             "H": H.tolist() if H is not None else None}
                dirty += 1
                if H is not None:
                    edges.append((n_inliers, ipos, jpos, H))
                if pairs_checkpoint and dirty >= 50:
                    with open(pairs_checkpoint, "w") as f:
                        json.dump({"meta": ckpt_meta, "pairs": ckpt}, f)
                    dirty = 0
        if pairs_checkpoint and dirty:
            with open(pairs_checkpoint, "w") as f:
                json.dump({"meta": ckpt_meta, "pairs": ckpt}, f)
        transforms, inlier_counts, tree_edges, unplaced = place_by_tree(
            len(chosen), edges)
        skipped = len(unplaced)
    else:
        transforms = [np.eye(3)]
        inlier_counts = []
        skipped = 0
        for pos in range(1, len(chosen)):
            a, b = chosen[pos - 1], chosen[pos]
            H, inliers = pair_homography(feats[a], feats[b], matcher,
                                         ratio=lowe_ratio)
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
                    Hj, nj = pair_homography(feats[j], feats[b], matcher,
                                             ratio=lowe_ratio)
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
        "placement": place,
        "lowe_ratio": lowe_ratio,
        "loop_check": loop_check,
        "loop_matches": loop_matches,
        "tree_edges": tree_edges,
        "pairs_from_checkpoint": pairs_from_checkpoint,
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
    ap.add_argument("--place", choices=("chain", "tree"), default="chain")
    ap.add_argument("--lowe-ratio", type=float, default=LOWE_RATIO)
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

    ckpt = None
    if args.place == "tree":
        ckpt = os.path.splitext(args.out)[0] + "_pairs.json"
    mosaic, stats = stitch(frames, args.stride, args.detector,
                           loop_check=args.loop_check, place=args.place,
                           lowe_ratio=args.lowe_ratio, pairs_checkpoint=ckpt)
    cv2.imwrite(args.out, mosaic)
    stats_path = os.path.splitext(args.out)[0] + ".json"
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out} and {stats_path}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
