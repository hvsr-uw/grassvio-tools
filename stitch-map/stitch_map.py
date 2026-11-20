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
import hashlib
import json
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bag-exploration"))
from explore_bag import parse_image, COLOR  # noqa: E402
from geometry import (apply_transform, hartley_transform, fit_homography,
                      projected_corners, render_mosaic)
from window_motion import flow_pairs, validate_camera_motion

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


def pair_homography(feats_cur, feats_next, matcher, ratio=LOWE_RATIO,
                    normalize=True):
    """Homography that maps points in the next frame into the current frame.

    feats are (keypoints, descriptors) pairs already computed for each
    frame. Returns (H, inlier_count) or (None, 0) when the match is too weak.
    """
    kp_cur, des_cur = feats_cur
    kp_next, des_next = feats_next
    if des_cur is None or des_next is None or len(des_cur) < 2 or len(des_next) < 2:
        return None, 0
    matches = matcher.knnMatch(des_next, des_cur, k=2)
    reverse = {m.queryIdx: m.trainIdx for m in matcher.match(des_cur, des_next)}
    good = []
    for pair in matches:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance and reverse.get(m.trainIdx) == m.queryIdx:
            good.append(m)
    if len(good) < MIN_INLIERS:
        return None, 0
    src = np.float32([kp_next[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp_cur[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = fit_homography(src, dst, normalize=normalize,
                             threshold_px=RANSAC_THRESH_PX, min_inliers=MIN_INLIERS)
    if H is None or mask is None:
        return None, 0
    inliers = int(mask.sum())
    if inliers < MIN_INLIERS:
        return None, 0
    return H, inliers


def place_by_tree(n, edges, frame_shape=None):
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
        try:
            candidate = transforms[r] @ (H if r < other else np.linalg.inv(H))
            if frame_shape is not None:
                projected_corners(candidate, frame_shape)
            transforms[other] = candidate
        except (np.linalg.LinAlgError, ValueError):
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
           lowe_ratio=LOWE_RATIO, pairs_checkpoint=None, normalize=True,
           motion_filter=False, flow_window=25, motion_residual_px=1.5,
           camera_still=False, camera_motion=None, blend="average",
           max_side=8000, max_pixels=16_000_000, debug_out=None):
    """Fit/track, place connected frames, then warp each original once.

    Flow mode detects at anchors every flow_window frames (independent of
    descriptor --stride), tracks through intermediate frames, rejects high
    temporal residual tracks, then runs final RANSAC on the survivors.
    """
    if stride < 1 or len(frames) < 2 or any(f is None for f in frames):
        raise ValueError("need at least two readable frames and a positive stride")
    if not 0 < lowe_ratio < 1:
        raise ValueError("Lowe ratio must lie between zero and one")
    if (camera_still or camera_motion is not None) and not motion_filter:
        raise ValueError("camera-motion options require --motion-filter")
    if motion_filter and (place != "chain" or loop_check):
        raise ValueError("window flow uses chain placement; omit --place tree and --loop-check")
    if not motion_filter and len(range(0, len(frames), stride)) < 2:
        raise ValueError("stride selects fewer than two frames")
    original_shapes = [f.shape for f in frames]
    frames = [downscale(f) for f in frames]
    if any(f.shape != frames[0].shape for f in frames):
        raise ValueError("all frames must share the same dimensions")
    if camera_motion is not None:
        camera_motion = validate_camera_motion(camera_motion, len(frames))
        # External priors are in ORIGINAL input pixel coordinates.
        scales = [np.diag([f.shape[1]/shape[1], f.shape[0]/shape[0], 1.])
                  for f, shape in zip(frames, original_shapes)]
        camera_motion = np.array([S @ H @ np.linalg.inv(scales[0])
                                  for S, H in zip(scales, camera_motion)])
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    chosen = list(range(0, len(frames), stride))
    transforms, inlier_counts, loop_matches, tree_edges, unplaced = [], [], [], [], []
    window_stats, pair_stats = [], []
    pairs_from_checkpoint = 0
    if motion_filter:
        chosen, transforms = [0], [np.eye(3)]
        anchor_pose = transforms[0]
        for start, end, H, record in flow_pairs(
                grays, flow_window, motion_residual_px, camera_still,
                camera_motion, MIN_INLIERS, normalize):
            T = None
            if anchor_pose is not None and H is not None:
                candidate = anchor_pose @ H
                try:
                    projected_corners(candidate, frames[end].shape)
                    T = candidate
                except ValueError as exc:
                    record["placement_error"] = str(exc)
            record["placed"] = T is not None
            if T is None:
                unplaced.append(len(chosen))
            else:
                inlier_counts.append(record["inliers"])
            chosen.append(end)
            transforms.append(T)
            window_stats.append(record)
            # After a broken window the chain has no known bridge. Later
            # relative fits remain diagnostic, not invented absolute poses.
            anchor_pose = T
    else:
        detector = make_detector(detector_name)
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING if detector_name == "orb" else cv2.NORM_L2)
        feats = {idx: detector.detectAndCompute(grays[idx], None) for idx in chosen}
        if place == "tree":
            digest = hashlib.sha256()
            for idx in chosen:
                digest.update(grays[idx].tobytes())
            ckpt_meta = {"version": 2, "detector": detector_name, "lowe_ratio": lowe_ratio,
                         "n_chosen": len(chosen), "stride": stride, "normalize": normalize,
                         "shape": list(grays[0].shape), "frames_sha256": digest.hexdigest()}
            ckpt = {}
            if pairs_checkpoint and os.path.exists(pairs_checkpoint):
                try:
                    with open(pairs_checkpoint) as f:
                        saved = json.load(f)
                    if saved.get("meta") == ckpt_meta:
                        ckpt = saved.get("pairs", {})
                except (OSError, ValueError):
                    pass
            edges = []
            for ipos in range(len(chosen)):
                for jpos in range(ipos + 1, len(chosen)):
                    key = f"{ipos},{jpos}"
                    if key in ckpt:
                        entry = ckpt[key]
                        pairs_from_checkpoint += 1
                        H = np.array(entry["H"], dtype=float) if entry["H"] is not None else None
                        n_inliers = entry["inliers"]
                    else:
                        H, n_inliers = pair_homography(feats[chosen[ipos]], feats[chosen[jpos]],
                                                       matcher, lowe_ratio, normalize)
                        ckpt[key] = {"inliers": n_inliers, "H": H.tolist() if H is not None else None}
                    if H is not None:
                        try:
                            projected_corners(H, frames[chosen[jpos]].shape)
                            projected_corners(np.linalg.inv(H), frames[chosen[ipos]].shape)
                            edges.append((n_inliers, ipos, jpos, H))
                        except (ValueError, np.linalg.LinAlgError):
                            pass
                if pairs_checkpoint:
                    with open(pairs_checkpoint, "w") as f:
                        json.dump({"meta": ckpt_meta, "pairs": ckpt}, f)
            transforms, inlier_counts, tree_edges, unplaced = place_by_tree(len(chosen), edges, frames[0].shape)
            for pos in unplaced:
                transforms[pos] = None
        else:
            transforms = [np.eye(3)]
            last_placed = 0
            for pos in range(1, len(chosen)):
                H, inliers = pair_homography(feats[chosen[last_placed]], feats[chosen[pos]],
                                             matcher, lowe_ratio, normalize)
                T = transforms[last_placed] @ H if H is not None else None
                if loop_check:
                    for jpos in range(pos - LOOP_MIN_GAP + 1):
                        if transforms[jpos] is None:
                            continue
                        Hj, nj = pair_homography(feats[chosen[jpos]], feats[chosen[pos]],
                                                 matcher, lowe_ratio, normalize)
                        if Hj is not None and nj >= max(LOOP_REANCHOR_MIN, inliers):
                            T, inliers = transforms[jpos] @ Hj, nj
                            loop_matches.append({"frame": pos, "matched_frame": jpos,
                                                 "inliers": nj, "reanchored": True})
                if T is not None:
                    try:
                        projected_corners(T, frames[chosen[pos]].shape)
                    except ValueError:
                        T = None
                pair_stats.append({"from_frame": chosen[last_placed], "to_frame": chosen[pos],
                                   "inliers": inliers, "placed": T is not None})
                transforms.append(T)
                if T is None:
                    unplaced.append(pos)
                else:
                    inlier_counts.append(inliers)
                    last_placed = pos
    selected = [frames[idx] for idx in chosen]
    if sum(T is not None for T in transforms) < 2:
        raise ValueError("fewer than two frames could be placed; no trustworthy mosaic")
    mosaic, render_stats = render_mosaic(selected, transforms, blend, max_side, max_pixels)
    stats = {"frames_total": len(frames), "frames_selected": len(chosen),
             "frames_used": sum(T is not None for T in transforms),
             "pairs_skipped": len(unplaced), "frames_unplaced": [chosen[p] for p in unplaced],
             "median_inliers": float(np.median(inlier_counts)) if inlier_counts else 0.,
             "stride": stride, "detector": "shi-tomasi-lk" if motion_filter else detector_name,
             "placement": place, "normalize": normalize, "lowe_ratio": lowe_ratio,
             "loop_check": loop_check, "loop_matches": loop_matches,
             "tree_edges": tree_edges, "pairs_from_checkpoint": pairs_from_checkpoint,
             "motion_filter": motion_filter, "flow_window": flow_window if motion_filter else None,
             "motion_residual_px": motion_residual_px if motion_filter else None,
             "windows": window_stats, "pairs": pair_stats,
             "selected_frames": chosen,
             "transforms_to_first": [T.tolist() if T is not None else None for T in transforms],
             **render_stats}
    if debug_out:
        write_debug_panel(selected, chosen, transforms, mosaic, render_stats, debug_out)
    return mosaic, stats


def write_debug_panel(frames, indices, transforms, mosaic, stats, path):
    """Original images beside the mosaic, with colored warped footprints."""
    map_view = mosaic.copy()
    shift = np.array(stats["canvas_shift"])
    placed = [i for i, T in enumerate(transforms) if T is not None]
    samples = sorted(set(placed[i] for i in np.linspace(0, len(placed)-1, min(4, len(placed))).astype(int)))
    colors = [(30, 220, 255), (255, 180, 40), (80, 240, 80), (230, 80, 230)]
    thumbs = []
    for pos, color in zip(samples, colors):
        poly = projected_corners(shift @ transforms[pos], frames[pos].shape)
        cv2.polylines(map_view, [np.rint(poly).astype(np.int32)], True, color, 2)
        thumb = cv2.resize(frames[pos], (320, 240))
        cv2.rectangle(thumb, (0, 0), (319, 239), color, 4)
        cv2.putText(thumb, f"original frame {indices[pos]}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        thumbs.append(thumb)
    left = np.vstack(thumbs)
    scale = min(960 / map_view.shape[1], left.shape[0] / map_view.shape[0])
    right = cv2.resize(map_view, (max(1, round(map_view.shape[1]*scale)), max(1, round(map_view.shape[0]*scale))))
    panel = np.zeros((left.shape[0], 320 + right.shape[1], 3), np.uint8)
    panel[:, :320] = left
    panel[:right.shape[0], 320:] = right
    if not cv2.imwrite(path, panel):
        raise OSError(f"could not write debug panel: {path}")


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
    ap.add_argument("--normalize", action="store_true", default=True)
    ap.add_argument("--direct", dest="normalize", action="store_false")
    ap.add_argument("--motion-filter", action="store_true")
    ap.add_argument("--flow-window", type=int, default=25)
    ap.add_argument("--motion-residual-px", type=float, default=1.5)
    camera = ap.add_mutually_exclusive_group()
    camera.add_argument("--camera-still", action="store_true",
                        help="known stationary ACQUISITION, not inferred from low gyro")
    camera.add_argument("--camera-motion", help="JSON full-motion pixel homographies for every input frame")
    ap.add_argument("--blend", choices=("average", "best"), default="average")
    ap.add_argument("--max-side", type=int, default=8000)
    ap.add_argument("--max-pixels", type=int, default=16_000_000)
    ap.add_argument("--debug-out", help="original frames beside map footprints")
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
    camera_motion = None
    if args.camera_motion:
        with open(args.camera_motion) as f:
            camera_motion = json.load(f)
    try:
        mosaic, stats = stitch(frames, args.stride, args.detector,
                               loop_check=args.loop_check, place=args.place,
                               lowe_ratio=args.lowe_ratio, pairs_checkpoint=ckpt,
                               normalize=args.normalize, motion_filter=args.motion_filter,
                               flow_window=args.flow_window, motion_residual_px=args.motion_residual_px,
                               camera_still=args.camera_still, camera_motion=camera_motion,
                               blend=args.blend, max_side=args.max_side, max_pixels=args.max_pixels,
                               debug_out=args.debug_out)
    except ValueError as exc:
        print(f"stitch failed: {exc}", file=sys.stderr)
        return 1
    if not cv2.imwrite(args.out, mosaic):
        raise OSError(f"could not write mosaic: {args.out}")
    stats_path = os.path.splitext(args.out)[0] + ".json"
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out} and {stats_path}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
