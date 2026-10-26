"""
Persistent feature-track statistics for a synthetic lawn frame sequence.

Diagnostic companion to vo_lk.py (which re-detects features every frame, so it
cannot report multi-frame track lengths). Uses the SAME Shi-Tomasi detection
and pyramidal Lucas-Kanade + forward/backward parameters as vo_lk.py, but
keeps persistent tracks: new features are detected only when the live track
count drops below 120 (the same re-detect threshold used in vo_lk.py).

Usage:
  python track_stats.py --video data/stripe_strong
Prints JSON: n_tracks, mean/median track length (frames), mean live tracks.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vo_lk import detect_features  # noqa: E402  (same detector params as VO)


def track_with_mask(gray1, gray2, pts1):
    """LK + forward-backward check; returns (p1, p2, good_mask) so callers can
    propagate per-point track IDs. Same parameters as vo_lk.track_features."""
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    pts2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, pts1, None, **lk)
    pts1_back, st_b, _ = cv2.calcOpticalFlowPyrLK(gray2, gray1, pts2, None, **lk)
    fb = np.linalg.norm(pts1 - pts1_back, axis=2).ravel()
    good = (st.ravel() == 1) & (st_b.ravel() == 1) & (fb < 1.0) & (err.ravel() < 30)
    return pts1[good], pts2[good], good


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, help="directory of frame_*.png")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.video, "*.png")))
    if not files:
        sys.exit("no png frames found in " + args.video)

    gray_prev = cv2.cvtColor(cv2.imread(files[0]), cv2.COLOR_BGR2GRAY)
    pts = detect_features(gray_prev)
    next_id = 0
    ids = np.arange(next_id, next_id + len(pts)); next_id += len(pts)
    live_len = {int(i): 1 for i in ids}   # track id -> current length (frames)
    finished = []                          # lengths of completed tracks
    live_counts = [len(ids)]

    for f in files[1:]:
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        if len(pts):
            p1, p2, good = track_with_mask(gray_prev, gray, pts)
            for i, g in zip(ids, good):
                if g:
                    live_len[int(i)] += 1
                else:
                    finished.append(live_len.pop(int(i)))
            ids, pts = ids[good], p2
        if len(pts) < 120:  # same re-detect policy as vo_lk.py
            fresh = detect_features(gray)
            if fresh is not None and len(fresh):
                new_ids = np.arange(next_id, next_id + len(fresh)); next_id += len(fresh)
                for i in new_ids:
                    live_len[int(i)] = 1
                ids = np.concatenate([ids, new_ids])
                pts = np.concatenate([pts, fresh]) if len(pts) else fresh
        live_counts.append(len(ids))
        gray_prev = gray

    finished.extend(live_len.values())  # tracks still alive at the end
    finished = np.array(finished, dtype=float)
    out = {
        "frames": len(files),
        "n_tracks": int(len(finished)),
        "mean_track_length_frames": float(finished.mean()) if len(finished) else 0.0,
        "median_track_length_frames": float(np.median(finished)) if len(finished) else 0.0,
        "mean_live_tracks": float(np.mean(live_counts)),
    }
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
