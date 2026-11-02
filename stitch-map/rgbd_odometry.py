#!/usr/bin/env python3
"""RGB-D odometry + orthomosaic for GrassVIO.

Replaces chained 2D homographies (which diverge on rotation-heavy takes)
with per-pair SE(3) estimated from the depth channel:

  pair:  SIFT match (same settings as stitch_map.py) ->
         back-project frame-i keypoints to 3D with depth + K_depth ->
         cv2.solvePnPRansac against frame-(i+1) 2D keypoints (K_color)
  chain: absolute poses A[i+1] = A[i] @ inv(T_{i+1,i})
  map:   orthomosaic — every valid depth pixel unprojected to 3D,
         transformed by A[i], splatted (averaged) into a top-down
         ground-plane grid. No homography warping anywhere.

Usage:
    python3 rgbd_odometry.py --bag <take>.db3 --out <prefix> [--stride 5]

Writes <prefix>_poses.json (absolute 4x4 poses + per-pair stats),
<prefix>_orthomosaic.png, <prefix>_trajectory.png.
Reads bags read-only.
"""
import argparse
import glob
import json
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bag-exploration"))
from explore_bag import parse_image, COLOR, DEPTH  # noqa: E402

DEPTH_UNITS = 0.001  # m per count (from Depth_Units metadata)
K_DEPTH = np.array([[386.009827, 0, 320.77182],
                    [0, 386.009827, 240.889587],
                    [0, 0, 1.0]])
K_COLOR = np.array([[386.251221, 0, 323.054626],
                    [0, 385.510895, 250.530975],
                    [0, 0, 1.0]])
STRIDE = 5
LOWE = 0.75
MIN_INLIERS = 20
MIN_3D = 30  # minimum keypoints with valid depth to attempt PnP


def load_synced(bag, max_offset_s=0.05):
    """Color+depth frames paired by nearest timestamp (dropped frames ok)."""
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    raw = {}
    for topic in (COLOR, DEPTH):
        tid = db.execute("select id from topics where name=?",
                         (topic,)).fetchone()[0]
        rows = db.execute(
            "select data from messages where topic_id=? order by timestamp",
            (tid,)).fetchall()
        items = []
        for (blob,) in rows:
            p = parse_image(blob)
            if "Depth" in topic:
                img = np.frombuffer(p["data"], dtype="<u2").reshape(
                    p["height"], p["width"])
                items.append((p["stamp_ns"], img, None))
            else:
                rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
                    p["height"], p["width"], 3)
                items.append((p["stamp_ns"],
                              cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                              cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)))
        raw[topic] = items
    db.close()
    d_ts = np.array([t for t, _, _ in raw[DEPTH]])
    color, depth, gray = [], [], []
    worst = 0.0
    for t, bgr, _ in raw[COLOR]:
        j = int(np.argmin(np.abs(d_ts - t)))
        off = abs(d_ts[j] - t) / 1e9
        worst = max(worst, off)
        if off > max_offset_s:
            continue
        color.append(bgr)
        gray.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        depth.append(raw[DEPTH][j][1])
    print(f"paired {len(color)} color+depth frames "
          f"(worst offset {worst*1000:.1f} ms)", flush=True)
    return {COLOR: gray, DEPTH: depth, "color_bgr": color}


def unproject(u, v, d_m):
    x = (u - K_DEPTH[0, 2]) / K_DEPTH[0, 0] * d_m
    y = (v - K_DEPTH[1, 2]) / K_DEPTH[1, 1] * d_m
    return np.stack([x, y, np.full_like(x, d_m)], axis=-1)


def pair_pose(gray_i, depth_i, gray_j, sift, matcher):
    """SE(3) of frame j relative to frame i (maps i-coords to j-coords)."""
    kp_i, des_i = sift.detectAndCompute(gray_i, None)
    kp_j, des_j = sift.detectAndCompute(gray_j, None)
    if des_i is None or des_j is None:
        return None, {"inliers": 0, "n_3d": 0, "ok": False}
    good = [m for m, n in
            (p for p in matcher.knnMatch(des_i, des_j, k=2) if len(p) == 2)
            if m.distance < LOWE * n.distance]
    if len(good) < MIN_INLIERS:
        return None, {"inliers": 0, "n_3d": 0, "ok": False}
    pts3d, pts2d = [], []
    h, w = depth_i.shape
    for m in good:
        u, v = kp_i[m.queryIdx].pt
        ui, vi = int(round(u)), int(round(v))
        if 0 <= ui < w and 0 <= vi < h:
            d = depth_i[vi, ui] * DEPTH_UNITS
            if d > 0.05:  # valid depth, beyond the noise floor
                pts3d.append(unproject(
                    np.array([u]), np.array([v]), np.array([d]))[0])
                pts2d.append(kp_j[m.trainIdx].pt)
    pts3d = np.float64(pts3d)
    pts2d = np.float32(pts2d)
    n_3d = len(pts3d)
    if n_3d < MIN_3D:
        return None, {"inliers": 0, "n_3d": n_3d, "ok": False}
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        pts3d, pts2d, K_COLOR, None,
        reprojectionError=3.0, confidence=0.99, iterationsCount=200)
    if not ok or inl is None or len(inl) < MIN_INLIERS:
        return None, {"inliers": int(len(inl)) if inl is not None else 0,
                      "n_3d": n_3d, "ok": False}
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T, {"inliers": int(len(inl)), "n_3d": n_3d, "ok": True}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=STRIDE)
    ap.add_argument("--grid_m", type=float, default=0.01,
                    help="orthomosaic cell size in meters")
    ap.add_argument("--px_step", type=int, default=2,
                    help="pixel subsample step for splatting")
    ap.add_argument("--max_depth_m", type=float, default=3.0,
                    help="ignore depth beyond this (sky/horizon junk)")
    ap.add_argument("--max_tilt_deg", type=float, default=35.0,
                    help="only splat frames looking down within this tilt")
    args = ap.parse_args()

    data = load_synced(args.bag)
    n = len(data[COLOR])
    idx = list(range(0, n, args.stride))
    print(f"{n} frames, {len(idx)} chosen @ stride {args.stride}",
          flush=True)

    sift = cv2.SIFT_create(nfeatures=10000)
    matcher = cv2.BFMatcher()
    gray = data[COLOR]
    bgr = data["color_bgr"]

    # ---- stage 2: per-pair SE(3) ----
    T_rel, stats = [], []
    for k in range(len(idx) - 1):
        i, j = idx[k], idx[k + 1]
        T, st = pair_pose(gray[i], data[DEPTH][i], gray[j], sift, matcher)
        st["pair"] = (i, j)
        stats.append(st)
        T_rel.append(T)
        print(f"pair {i}->{j}: ok={st['ok']} n_3d={st['n_3d']} "
              f"inliers={st['inliers']}", flush=True)
    n_ok = sum(s["ok"] for s in stats)
    print(f"pairs ok: {n_ok}/{len(stats)}", flush=True)

    # ---- stage 3a: chain to absolute poses ----
    A = [np.eye(4)]
    for T, st in zip(T_rel, stats):
        if T is None:
            A.append(A[-1].copy())  # hold last pose on failed pair
        else:
            A.append(A[-1] @ np.linalg.inv(T))

    # ---- stage 3b: orthomosaic splat ----
    # A top-down map only makes sense from frames looking down: skip
    # splatting (not posing) for frames tilted past max_tilt_deg, and
    # cap depth so horizon/sky points can't blow up the grid.
    DOWN = np.array([0.0, 0.0, 1.0])
    cos_lim = np.cos(np.deg2rad(args.max_tilt_deg))
    splat_ok = []
    for k in range(len(idx)):
        zax = A[k][:3, :3] @ DOWN
        splat_ok.append(bool(zax[2] > cos_lim))
    print(f"splatting {sum(splat_ok)}/{len(idx)} frames "
          f"(tilt <= {args.max_tilt_deg} deg)", flush=True)

    h, w = data[DEPTH][0].shape
    uu, vv = np.meshgrid(np.arange(0, w, args.px_step),
                         np.arange(0, h, args.px_step))
    uu = uu.ravel().astype(np.float64)
    vv = vv.ravel().astype(np.float64)

    # first pass: bounds
    mins = np.full(2, np.inf)
    maxs = np.full(2, -np.inf)
    for k, fidx in enumerate(idx):
        if not splat_ok[k]:
            continue
        d = data[DEPTH][fidx][vv.astype(int), uu.astype(int)] * DEPTH_UNITS
        valid = (d > 0.05) & (d < args.max_depth_m)
        if not np.any(valid):
            continue
        P = unproject(uu[valid], vv[valid], d[valid])
        Pw = (A[k][:3, :3] @ P.T).T + A[k][:3, 3]
        mins = np.minimum(mins, Pw[:, :2].min(axis=0))
        maxs = np.maximum(maxs, Pw[:, :2].max(axis=0))
    ext = maxs - mins
    print(f"map extent: {ext[0]:.2f} x {ext[1]:.2f} m", flush=True)
    gw, gh = int(np.ceil(ext[0] / args.grid_m)) + 1, int(
        np.ceil(ext[1] / args.grid_m)) + 1
    acc = np.zeros((gh, gw, 3), dtype=np.float64)
    cnt = np.zeros((gh, gw), dtype=np.float64)
    for k, fidx in enumerate(idx):
        if not splat_ok[k]:
            continue
        d = data[DEPTH][fidx][vv.astype(int), uu.astype(int)] * DEPTH_UNITS
        valid = (d > 0.05) & (d < args.max_depth_m)
        if not np.any(valid):
            continue
        P = unproject(uu[valid], vv[valid], d[valid])
        Pw = (A[k][:3, :3] @ P.T).T + A[k][:3, 3]
        gx = ((Pw[:, 0] - mins[0]) / args.grid_m).astype(int)
        gy = ((Pw[:, 1] - mins[1]) / args.grid_m).astype(int)
        col = bgr[fidx][vv.astype(int)[valid], uu.astype(int)[valid]
                      ].astype(np.float64)
        np.add.at(acc, (gy, gx), col)
        np.add.at(cnt, (gy, gx), 1)
        if k % 40 == 0:
            print(f"splatted {k}/{len(idx)}", flush=True)
    mask = cnt > 0
    img = np.zeros((gh, gw, 3), dtype=np.uint8)
    img[mask] = (acc[mask] / cnt[mask, None]).astype(np.uint8)
    cv2.imwrite(args.out + "_orthomosaic.png", img)
    print(f"wrote {args.out}_orthomosaic.png {gw}x{gh}", flush=True)

    # trajectory (camera centers in world)
    traj = np.array([a[:3, 3] for a in A])
    np.save(args.out + "_trajectory.npy", traj)

    with open(args.out + "_poses.json", "w") as f:
        json.dump({
            "bag": os.path.basename(args.bag),
            "stride": args.stride,
            "n_chosen": len(idx),
            "pairs_ok": n_ok,
            "pair_stats": stats,
            "absolute_poses": [a.tolist() for a in A],
            "map_extent_m": [float(ext[0]), float(ext[1])],
            "grid_m": args.grid_m,
        }, f)
    print("done", flush=True)


if __name__ == "__main__":
    main()
