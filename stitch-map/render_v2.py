#!/usr/bin/env python3
"""Re-render an RGB-D orthomosaic from existing poses (render v2).

The poses come from rgbd_odometry.py and are NOT recomputed here. This
script only changes how the map is drawn, so a bad render can be thrown
away without touching the odometry:

  1. denser splatting: poses exist at stride 5; SE(3) is interpolated
     (quaternion slerp + translation lerp) so every frame can splat,
     up to 5x more real points than the stride-5 map.
  2. edge-depth filter: depth pixels that disagree with a 5x5 median
     by more than 5 cm are dropped (object edges carry the noisiest
     depth). This is a per-pixel validity test, not smoothing.
  3. two blends per take: per-cell average and last-frame-wins, so the
     better-looking one can be picked by eye.
  4. pinhole fill: unobserved cells within 2 cells of observed data are
     inpainted; genuinely empty regions are left empty.
  5. optional ground-band filter: drop splatted points more than
     --zband metres above the median splatted height (walls and bushes
     seen over the grass). Off by default; judged per take.
  6. optional trajectory overlay: the camera path drawn on the map so
     a split between two patches reads as "camera tipped here".

Usage:
    python3 render_v2.py --poses <poses.json> --bag <take>.db3 --out <prefix>
Reads bags read-only. Writes <prefix>_v2_avg.png,
<prefix>_v2_lastwins.png, <prefix>_v2_avg_fill.png and, with --traj,
<prefix>_v2_traj.png.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bag-exploration"))
from explore_bag import COLOR, DEPTH  # noqa: E402
from rgbd_odometry import (  # noqa: E402
    DEPTH_UNITS, K_DEPTH, load_synced, unproject)

DOWN = np.array([0.0, 0.0, 1.0])


def mat2quat(R):
    """Rotation matrix -> unit quaternion (x, y, z, w)."""
    m = np.trace(R)
    if m > 0:
        s = np.sqrt(m + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat2mat(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, t):
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


def interp_poses(A, stride, n_frames):
    """Absolute pose per frame: slerp between stride-5 anchors."""
    quats = [mat2quat(a[:3, :3]) for a in A]
    out = []
    for f in range(n_frames):
        k = min(f // stride, len(A) - 1)
        if k >= len(A) - 1:
            out.append(A[-1])
            continue
        t = (f - k * stride) / stride
        if t == 0:
            out.append(A[k])
            continue
        R = quat2mat(slerp(quats[k], quats[k + 1], t))
        tr = (1 - t) * A[k][:3, 3] + t * A[k + 1][:3, 3]
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = tr
        out.append(M)
    return out


def frame_points(depth_u16, uu, vv, max_depth_m, edge_filter):
    """Valid depth pixels of one frame, unprojected to camera coords."""
    d = depth_u16[vv.astype(int), uu.astype(int)] * DEPTH_UNITS
    valid = (d > 0.05) & (d < max_depth_m)
    if edge_filter:
        med = cv2.medianBlur(depth_u16, 5).astype(np.float64)[vv.astype(int),
                                                             uu.astype(int)]
        med *= DEPTH_UNITS
        valid &= np.abs(d - med) < 0.05
    if not np.any(valid):
        return None, None
    P = unproject(uu[valid], vv[valid], d[valid])
    return P, valid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--render-stride", type=int, default=1,
                    help="splat every Nth frame (1 = every frame)")
    ap.add_argument("--grid_m", type=float, default=0.01)
    ap.add_argument("--px-step", type=int, default=2)
    ap.add_argument("--max-depth-m", type=float, default=3.0)
    ap.add_argument("--max-tilt-deg", type=float, default=35.0)
    ap.add_argument("--edge-filter", action="store_true")
    ap.add_argument("--zband", type=float, default=0.0,
                    help="if > 0, drop points this far above the median "
                         "splatted height (metres)")
    ap.add_argument("--traj", action="store_true",
                    help="also write a trajectory-overlay variant")
    args = ap.parse_args()

    with open(args.poses) as f:
        pj = json.load(f)
    stride = pj["stride"]
    A_anchor = [np.array(a) for a in pj["absolute_poses"]]
    grid_m = pj.get("grid_m", args.grid_m)

    data = load_synced(args.bag)
    n = len(data[COLOR])
    idx = list(range(0, n, stride))
    assert len(idx) == len(A_anchor), (
        f"poses {len(A_anchor)} != chosen frames {len(idx)}")
    A = interp_poses(A_anchor, stride, n)
    render_idx = list(range(0, n, args.render_stride))
    print(f"{n} frames, {len(render_idx)} rendered @ stride "
          f"{args.render_stride}", flush=True)

    cos_lim = np.cos(np.deg2rad(args.max_tilt_deg))
    splat_ok = []
    for f in render_idx:
        zax = A[f][:3, :3] @ DOWN
        splat_ok.append(bool(zax[2] > cos_lim))
    print(f"splatting {sum(splat_ok)}/{len(render_idx)} frames "
          f"(tilt <= {args.max_tilt_deg} deg)", flush=True)

    h, w = data[DEPTH][0].shape
    uu, vv = np.meshgrid(np.arange(0, w, args.px_step),
                         np.arange(0, h, args.px_step))
    uu = uu.ravel().astype(np.float64)
    vv = vv.ravel().astype(np.float64)

    # pass 1: bounds + median splatted height (for the z-band filter)
    mins = np.full(2, np.inf)
    maxs = np.full(2, -np.inf)
    zs = []
    for j, f in enumerate(render_idx):
        if not splat_ok[j]:
            continue
        P, valid = frame_points(data[DEPTH][f], uu, vv,
                                args.max_depth_m, args.edge_filter)
        if P is None:
            continue
        Pw = (A[f][:3, :3] @ P.T).T + A[f][:3, 3]
        mins = np.minimum(mins, Pw[:, :2].min(axis=0))
        maxs = np.maximum(maxs, Pw[:, :2].max(axis=0))
        zs.append(np.median(Pw[:, 2]))
    ext = maxs - mins
    z_med = float(np.median(zs)) if zs else 0.0
    print(f"map extent: {ext[0]:.2f} x {ext[1]:.2f} m, "
          f"median splatted height {z_med:.2f} m", flush=True)
    gw, gh = int(np.ceil(ext[0] / grid_m)) + 1, int(
        np.ceil(ext[1] / grid_m)) + 1
    ncell = gh * gw

    # pass 2: splat (bincount average + last-wins)
    acc = np.zeros((ncell, 3), dtype=np.float64)
    cnt = np.zeros(ncell, dtype=np.float64)
    last = np.zeros((gh, gw, 3), dtype=np.uint8)
    last_seen = np.zeros((gh, gw), dtype=bool)
    for j, f in enumerate(render_idx):
        if not splat_ok[j]:
            continue
        P, valid = frame_points(data[DEPTH][f], uu, vv,
                                args.max_depth_m, args.edge_filter)
        if P is None:
            continue
        Pw = (A[f][:3, :3] @ P.T).T + A[f][:3, 3]
        if args.zband > 0:
            keep = Pw[:, 2] < z_med + args.zband
            if not np.any(keep):
                continue
            Pw = Pw[keep]
            valid = valid.copy()
            vidx = np.where(valid)[0][keep]
            col = data["color_bgr"][f][vv.astype(int)[vidx],
                                       uu.astype(int)[vidx]].astype(np.float64)
            gx = ((Pw[:, 0] - mins[0]) / grid_m).astype(int)
            gy = ((Pw[:, 1] - mins[1]) / grid_m).astype(int)
        else:
            col = data["color_bgr"][f][vv.astype(int)[valid],
                                       uu.astype(int)[valid]].astype(np.float64)
            gx = ((Pw[:, 0] - mins[0]) / grid_m).astype(int)
            gy = ((Pw[:, 1] - mins[1]) / grid_m).astype(int)
        gx = np.clip(gx, 0, gw - 1)
        gy = np.clip(gy, 0, gh - 1)
        flat = gy * gw + gx
        for c in range(3):
            acc[:, c] += np.bincount(flat, weights=col[:, c],
                                     minlength=ncell)
        cnt += np.bincount(flat, minlength=ncell)
        last[gy, gx] = col.astype(np.uint8)
        last_seen[gy, gx] = True
        if j % 100 == 0:
            print(f"splatted {j}/{len(render_idx)}", flush=True)

    acc = acc.reshape(gh, gw, 3)
    cnt = cnt.reshape(gh, gw)
    mask = cnt > 0
    cov = float(mask.mean())
    print(f"coverage: {cov * 100:.1f}% of {gw}x{gh} cells", flush=True)
    avg = np.zeros((gh, gw, 3), dtype=np.uint8)
    avg[mask] = (acc[mask] / cnt[mask, None]).astype(np.uint8)
    cv2.imwrite(args.out + "_v2_avg.png", avg)
    lw = np.zeros((gh, gw, 3), dtype=np.uint8)
    lw[last_seen] = last[last_seen]
    cv2.imwrite(args.out + "_v2_lastwins.png", lw)
    print(f"wrote {args.out}_v2_avg.png and _v2_lastwins.png "
          f"{gw}x{gh}", flush=True)

    # pinhole fill on the average render: only cells within 2 cells of
    # observed data are filled; real empty regions stay empty.
    kernel = np.ones((5, 5), np.uint8)
    dil = cv2.dilate(mask.astype(np.uint8), kernel) > 0
    fill_mask = (dil & ~mask).astype(np.uint8) * 255
    if np.any(fill_mask):
        filled = cv2.inpaint(avg, fill_mask, 2, cv2.INPAINT_TELEA)
    else:
        filled = avg
    cv2.imwrite(args.out + "_v2_avg_fill.png", filled)
    print(f"wrote {args.out}_v2_avg_fill.png", flush=True)

    # trajectory overlay on the filled average render
    if args.traj:
        traj = np.array([a[:3, 3] for a in A])
        overlay = filled.copy()
        px = ((traj[:, 0] - mins[0]) / grid_m).astype(int)
        py = ((traj[:, 1] - mins[1]) / grid_m).astype(int)
        ok = (px >= 0) & (px < gw) & (py >= 0) & (py < gh)
        pts = np.stack([px[ok], py[ok]], axis=1).reshape(-1, 1, 2)
        cv2.polylines(overlay, [pts], False, (255, 0, 0), 2)
        cv2.circle(overlay, tuple(pts[0, 0]), 5, (0, 255, 0), -1)
        cv2.circle(overlay, tuple(pts[-1, 0]), 5, (0, 0, 255), -1)
        cv2.imwrite(args.out + "_v2_traj.png", overlay)
        print(f"wrote {args.out}_v2_traj.png", flush=True)

    with open(args.out + "_v2.json", "w") as f:
        json.dump({
            "poses": os.path.basename(args.poses),
            "render_stride": args.render_stride,
            "px_step": args.px_step,
            "edge_filter": args.edge_filter,
            "zband": args.zband,
            "extent_m": [float(ext[0]), float(ext[1])],
            "coverage": cov,
            "frames_splatted": int(sum(splat_ok)),
        }, f)
    print("done", flush=True)


if __name__ == "__main__":
    main()
