#!/usr/bin/env python3
"""Feed ORB-SLAM3 RGB-D-only poses into the render_v2.py orthomosaic
machinery (take 20261007_080033).

Why this adapter exists: render_v2.py reads a poses JSON of absolute
4x4 camera-to-world matrices with a stride, and splats every frame by
interpolating between anchors. The ORB-SLAM3 trajectory is a TUM file
(440 poses, one per paired color frame), so this script converts it to
that JSON shape at stride 1, one pose per color frame in the bag.

Alignment (reused from compare.py in the ORB-SLAM3 work, same idea):
the ORB-SLAM3 trajectory is already in the first-camera frame -- its
first pose is the identity -- exactly like the VO absolute poses, so
no SE(3) alignment is applied. Each pose is matched to a bag color
frame by nearest timestamp. The bag has 441 color frames but only
440 ORB poses: the one color frame with no ORB pose gets the pose
interpolated (slerp + lerp) between its two bracketing ORB poses.
That frame is reported on stdout.

Reused, not re-derived:
  * mat2quat / quat2mat / slerp from render_v2.py (imported).
  * load_synced pairing rule from rgbd_odometry.py: color to depth by
    nearest timestamp, pairs over 50 ms dropped. This script replicates
    only the timestamp side of that pairing to list the color frames
    in exactly the order render_v2.py will see them.
  * DEPTH_UNITS / K_DEPTH / unproject are used inside render_v2.py,
    not here.

Caveats carried over from the VO renders, unchanged:
  * Depth is NOT aligned to color: the color camera sits 5.9 cm from
    the depth camera, and the depth map is splatted as-is with the
    depth intrinsics. Same caveat as the VO map, so the comparison
    is fair.
  * Render rules match the VO map: splat depth capped at 3 m, only
    frames within 35 deg of straight down are splatted (both enforced
    inside render_v2.py with its defaults).

Usage:
    python3 orbslam3_pose_render.py --tum <traj.txt> --bag <take>.db3 \
        --out <poses.json>
Then:
    python3 render_v2.py --poses <poses.json> --bag <take>.db3 \
        --out <prefix> --traj
Bags are read read-only.
"""

import argparse
import json
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "dataset-tools",
                                "stitch-map"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "dataset-tools",
                                "bag-exploration"))
from explore_bag import parse_image, COLOR, DEPTH  # noqa: E402
import render_v2  # noqa: E402  (mat2quat, quat2mat, slerp)


def load_tum(path):
    """TUM trajectory -> timestamps, translations, quaternions (x,y,z,w)."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(x) for x in line.split()])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def tum_to_mat(t_xyz, q):
    M = np.eye(4)
    M[:3, :3] = render_v2.quat2mat(q)
    M[:3, 3] = t_xyz
    return M


def bag_color_timestamps(bag, max_offset_s=0.05):
    """Color-frame timestamps in the order load_synced() returns them.

    Replicates the pairing rule of rgbd_odometry.load_synced: color
    frames are kept in timestamp order, dropping any whose nearest
    depth frame is over max_offset_s away.
    """
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    stamps = {}
    for topic in (COLOR, DEPTH):
        tid = db.execute("select id from topics where name=?",
                         (topic,)).fetchone()[0]
        rows = db.execute(
            "select data from messages where topic_id=? order by timestamp",
            (tid,)).fetchall()
        stamps[topic] = np.array([parse_image(b)["stamp_ns"] / 1e9
                                  for (b,) in rows])
    db.close()
    d_ts = stamps[DEPTH]
    kept = []
    for t in stamps[COLOR]:
        j = int(np.argmin(np.abs(d_ts - t)))
        if abs(d_ts[j] - t) <= max_offset_s:
            kept.append(t)
    return np.array(kept)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    t_orb, xyz_orb, q_orb = load_tum(args.tum)
    n_orb = len(t_orb)
    color_ts = bag_color_timestamps(args.bag)
    n_color = len(color_ts)
    print(f"ORB poses: {n_orb}, bag color frames (paired): {n_color}",
          flush=True)

    quats = [render_v2.mat2quat(render_v2.quat2mat(q)) for q in q_orb]
    poses = []
    n_exact = 0
    n_interp = 0
    worst_gap_ms = 0.0
    for t in color_ts:
        j = int(np.argmin(np.abs(t_orb - t)))
        gap = abs(t_orb[j] - t)
        worst_gap_ms = max(worst_gap_ms, gap * 1000.0)
        if gap < 1e-6:
            poses.append(tum_to_mat(xyz_orb[j], q_orb[j]))
            n_exact += 1
            continue
        # No ORB pose at this exact timestamp: interpolate between the
        # bracketing ORB poses (or hold the nearest at the ends).
        n_interp += 1
        if t <= t_orb[0]:
            poses.append(tum_to_mat(xyz_orb[0], q_orb[0]))
            print(f"  color frame t={t:.6f}: before first ORB pose, "
                  f"holding first (gap {gap * 1000:.1f} ms)", flush=True)
        elif t >= t_orb[-1]:
            poses.append(tum_to_mat(xyz_orb[-1], q_orb[-1]))
            print(f"  color frame t={t:.6f}: after last ORB pose, "
                  f"holding last (gap {gap * 1000:.1f} ms)", flush=True)
        else:
            k = int(np.searchsorted(t_orb, t)) - 1
            f = (t - t_orb[k]) / (t_orb[k + 1] - t_orb[k])
            R = render_v2.quat2mat(render_v2.slerp(quats[k], quats[k + 1], f))
            tr = (1 - f) * xyz_orb[k] + f * xyz_orb[k + 1]
            M = np.eye(4)
            M[:3, :3] = R
            M[:3, 3] = tr
            poses.append(M)
            print(f"  color frame t={t:.6f}: interpolated between ORB "
                  f"poses {k} and {k + 1} (f={f:.2f}, "
                  f"nearest gap {gap * 1000:.1f} ms)", flush=True)

    assert len(poses) == n_color, (len(poses), n_color)
    print(f"exact matches: {n_exact}, interpolated/held: {n_interp}, "
          f"worst nearest gap: {worst_gap_ms:.1f} ms", flush=True)

    # Sanity: first pose must be (near-)identity -- the ORB-SLAM3 world
    # frame is the first camera frame, same as the VO poses.
    print(f"first pose translation: {poses[0][:3, 3]}", flush=True)
    print(f"last pose translation: {poses[-1][:3, 3]}", flush=True)
    end_err = float(np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3]))
    print(f"end-to-start translation: {end_err:.3f} m", flush=True)

    with open(args.out, "w") as f:
        json.dump({
            "tum": os.path.basename(args.tum),
            "bag": os.path.basename(args.bag),
            "stride": 1,
            "n_color_frames": n_color,
            "n_orb_poses": n_orb,
            "n_exact": n_exact,
            "n_interpolated": n_interp,
            "worst_nearest_gap_ms": worst_gap_ms,
            "absolute_poses": [p.tolist() for p in poses],
            "grid_m": 0.01,
        }, f)
    print(f"wrote {args.out} ({len(poses)} poses, stride 1)", flush=True)


if __name__ == "__main__":
    main()
