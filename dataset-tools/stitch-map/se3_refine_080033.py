#!/usr/bin/env python3
"""SE(3) pose-graph refinement of the ORB-SLAM3-pose map, take 20261007_080033.

The ORB-SLAM3 RGB-D-only trajectory already closes its loop to 0.061 m /
13 deg with no loop closure. This script adds the one missing constraint
-- a measured SE(3) loop edge between end frames and start frames, from
depth + PnP -- and re-optimizes the 440 poses with Levenberg-Marquardt
Gauss-Newton in numpy (no scipy in the environment).

Why this graph can work where the homography graph could not: the
homography edges all carried the same systematic planar-model bias, so
joint optimization only redistributed it. The ORB poses come from bundle
adjustment on true 3D depth data, so the chain edges are approximately
unbiased and the loop edge genuinely closes the loop.

Stages:
  1. Loop edge: SIFT + depth back-projection + solvePnPRansac on
     end-vs-start frame pairs (same pair_pose method as rgbd_odometry.py,
     with SIFT detected once per frame and reused across pairs).
  2. Pose graph: 440 nodes, 439 odometry edges + 1 loop edge, first
     pose fixed. Dense Levenberg-Marquardt with numeric Jacobians.
  3. Refined trajectory written as TUM; the existing adapter script
     turns it into a poses JSON and render_v2.py renders the map.

Bags are read read-only. Nothing here is uploaded anywhere.
"""

import argparse
import glob
import json
import math
import os
import sqlite3
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
STITCH = os.path.join(ROOT, "dataset-tools", "stitch-map")
sys.path.insert(0, STITCH)
sys.path.insert(0, os.path.join(ROOT, "dataset-tools", "bag-exploration"))
from explore_bag import parse_image, COLOR, DEPTH  # noqa: E402
from rgbd_odometry import (  # noqa: E402
    DEPTH_UNITS, K_COLOR, K_DEPTH, LOWE, MIN_3D, MIN_INLIERS, unproject)

BAG_GLOB = os.path.expanduser(
    "~/workspace/browser_downloads/*/browser-download-20261027T081611"
    "*.db3")
# The Oct 27 download of take 20261007_080033, the same file the
# ORB-SLAM3-pose map used. The glob above resolves it by date stamp.


# ---------------------------------------------------------------- SO(3)/SE(3)

def skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def log_so3(R):
    """Rotation matrix -> rotation vector (axis * angle, radians)."""
    cos_a = (np.trace(R) - 1.0) / 2.0
    cos_a = min(1.0, max(-1.0, cos_a))
    a = math.acos(cos_a)
    if a < 1e-8:
        return np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) / 2.0
    if math.pi - a < 1e-4:
        # Near pi: recover the axis from the diagonal.
        d = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        axis = d.copy()
        if R[2, 1] - R[1, 2] < 0:
            axis[0] = -axis[0]
        if R[0, 2] - R[2, 0] < 0:
            axis[1] = -axis[1]
        if R[1, 0] - R[0, 1] < 0:
            axis[2] = -axis[2]
        return axis / (np.linalg.norm(axis) + 1e-12) * a
    return a / (2.0 * math.sin(a)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def exp_so3(phi):
    a = np.linalg.norm(phi)
    if a < 1e-12:
        return np.eye(3) + skew(phi)
    k = phi / a
    K = skew(k)
    return np.eye(3) + math.sin(a) * K + (1.0 - math.cos(a)) * (K @ K)


def exp_se3(xi):
    """6-vector [rho, phi] -> 4x4 (left-increment convention)."""
    rho, phi = xi[:3], xi[3:]
    R = exp_so3(phi)
    a = np.linalg.norm(phi)
    if a < 1e-8:
        J = np.eye(3) + 0.5 * skew(phi)
    else:
        K = skew(phi / a)
        J = (np.eye(3) + (1.0 - math.cos(a)) / a * K
             + (a - math.sin(a)) / a * (K @ K))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = J @ rho
    return T


def rot_angle_deg(R):
    cos_a = min(1.0, max(-1.0, (np.trace(R) - 1.0) / 2.0))
    return math.degrees(math.acos(cos_a))


def se3_err_vec(T):
    """Error 6-vector [t; log(R)] for a 4x4 (zero iff T is identity)."""
    return np.concatenate([T[:3, 3], log_so3(T[:3, :3])])


# ---------------------------------------------------------------- trajectory

def load_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(x) for x in line.split()])
    a = np.array(rows)
    return a[:, 0], [tum_row_to_mat(r) for r in a]


def tum_row_to_mat(r):
    M = np.eye(4)
    x, y, z, qx, qy, qz, qw = r[1:8]
    # Quaternion (x, y, z, w) -> rotation matrix, direct formula.
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)]])
    M[:3, :3] = R
    M[:3, 3] = [x, y, z]
    return M


def mat_to_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def write_tum(path, stamps, poses):
    with open(path, "w") as f:
        for t, M in zip(stamps, poses):
            q = mat_to_quat(M[:3, :3])
            f.write(f"{t:.6f} {M[0, 3]:.9f} {M[1, 3]:.9f} {M[2, 3]:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


# ---------------------------------------------------------------- bag frames

def kept_color_stamps(bag, max_offset_s=0.05):
    """Timestamps of kept color frames, in the order load_synced returns
    them (nearest depth within 50 ms). Record timestamps equal the
    header stamps (verified on this bag)."""
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    stamps = {}
    for topic in (COLOR, DEPTH):
        tid = db.execute("select id from topics where name=?",
                         (topic,)).fetchone()[0]
        rows = db.execute(
            "select timestamp from messages where topic_id=? "
            "order by timestamp", (tid,)).fetchall()
        stamps[topic] = np.array([r[0] for r in rows], dtype=np.int64)
    db.close()
    d_ts = stamps[DEPTH]
    kept = []
    for t in stamps[COLOR]:
        j = int(np.argmin(np.abs(d_ts - t)))
        if abs(d_ts[j] - t) <= max_offset_s * 1e9:
            kept.append(t)
    return np.array(kept, dtype=np.int64)


def fetch_frame(bag, topic, stamp_ns, depth_stamps=None):
    """One decoded frame (gray for color, uint16 for depth) by stamp.
    For depth, stamp_ns is a color stamp: the nearest depth frame is
    used, the same pairing rule load_synced applies."""
    if "Depth" in topic and depth_stamps is not None:
        j = int(np.argmin(np.abs(depth_stamps - stamp_ns)))
        stamp_ns = int(depth_stamps[j])
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?",
                     (topic,)).fetchone()[0]
    blob = db.execute(
        "select data from messages where topic_id=? and timestamp=?",
        (tid, int(stamp_ns))).fetchone()[0]
    db.close()
    p = parse_image(blob)
    if "Depth" in topic:
        return np.frombuffer(p["data"], dtype="<u2").reshape(
            p["height"], p["width"])
    rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
        p["height"], p["width"], 3)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def pair_pose_precomp(kp_i, des_i, depth_i, kp_j, des_j, matcher):
    """SE(3) of frame j relative to frame i (maps i-coords to j-coords).

    Same method as rgbd_odometry.pair_pose: SIFT matches, back-project
    frame-i keypoints to 3D with depth + K_DEPTH, solvePnPRansac
    against frame-j 2D points with K_COLOR. SIFT features are
    precomputed by the caller and reused across pairs.
    """
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
            if d > 0.05:
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

# ---------------------------------------------------------------- pose graph

def edge_residual(Ai, Aj, Z):
    """6-vector residual [t; log(R)] of Z^-1 Ai^-1 Aj (zero iff the edge
    measurement Z agrees with the current poses)."""
    return se3_err_vec(np.linalg.inv(Z) @ np.linalg.inv(Ai) @ Aj)


def build_edges(poses, loop):
    """Odometry edges from consecutive ORB poses + the one loop edge.
    loop = (i, j, Z, weight)."""
    edges = []
    for k in range(len(poses) - 1):
        Z = np.linalg.inv(poses[k]) @ poses[k + 1]
        edges.append((k, k + 1, Z, 1.0))
    edges.append(loop)
    return edges


def total_cost(poses, edges):
    c = 0.0
    for (i, j, Z, w) in edges:
        r = edge_residual(poses[i], poses[j], Z)
        c += w * float(r @ r)
    return c


def rms_split(poses, edges):
    """RMS translation (m) / rotation (deg) residual, all edges and
    split by chain vs loop."""
    out = {}
    for tag, grp in (("all", edges), ("chain", edges[:-1]),
                     ("loop", edges[-1:])):
        ts, rs = [], []
        for (i, j, Z, w) in grp:
            r = edge_residual(poses[i], poses[j], Z)
            ts.append(np.linalg.norm(r[:3]))
            rs.append(np.linalg.norm(r[3:]))
        out[tag] = (float(np.sqrt(np.mean(np.square(ts)))),
                    float(np.degrees(np.sqrt(np.mean(np.square(rs))))))
    return out


def lm_optimize(poses, edges, max_iter=30, tol=1e-10, verbose=True):
    """Levenberg-Marquardt over left-increment 6-vectors; pose 0 fixed.
    Dense numeric Jacobians (central differences per edge, 12 local
    params). Returns (poses, n_iter, cost_history)."""
    n = len(poses)
    free = list(range(1, n))  # node indices optimized
    col = {node: 6 * k for k, node in enumerate(free)}
    npar = 6 * len(free)
    nres = 6 * len(edges)
    lam = 1e-3
    cost = total_cost(poses, edges)
    hist = [cost]
    if verbose:
        print(f"  LM start: cost {cost:.6g}", flush=True)

    def resid_vec(P):
        r = np.empty(nres)
        for e, (i, j, Z, w) in enumerate(edges):
            r[6 * e:6 * e + 6] = math.sqrt(w) * edge_residual(P[i], P[j], Z)
        return r

    h = 1e-6
    for it in range(max_iter):
        r0 = resid_vec(poses)
        # Numeric Jacobian, edge by edge (each edge touches 2 nodes).
        J = np.zeros((nres, npar))
        for e, (i, j, Z, w) in enumerate(edges):
            sw = math.sqrt(w)
            for node, sign in ((i, 0), (j, 1)):
                if node == 0:
                    continue
                c0 = col[node]
                for m in range(6):
                    d = np.zeros(6)
                    d[m] = h
                    Pp = list(poses)
                    Pm = list(poses)
                    Pp[node] = exp_se3(d) @ poses[node]
                    Pm[node] = exp_se3(-d) @ poses[node]
                    rp = sw * edge_residual(Pp[i], Pp[j], Z)
                    rm = sw * edge_residual(Pm[i], Pm[j], Z)
                    J[6 * e:6 * e + 6, c0 + m] = (rp - rm) / (2 * h)
        H = J.T @ J
        g = J.T @ r0
        diag = np.maximum(np.diag(H), 1e-12)
        accepted = False
        for _ in range(50):
            try:
                delta = np.linalg.solve(H + lam * np.diag(diag), -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            cand = list(poses)
            for node in free:
                d = delta[col[node]:col[node] + 6]
                cand[node] = exp_se3(d) @ poses[node]
            c_new = total_cost(cand, edges)
            if c_new < cost:
                rel = (cost - c_new) / max(cost, 1e-12)
                poses, cost = cand, c_new
                hist.append(cost)
                lam = max(lam / 3.0, 1e-9)
                accepted = True
                if verbose:
                    print(f"  iter {it + 1}: cost {cost:.6g} "
                          f"(rel drop {rel:.2e}, lambda {lam:.1e})",
                          flush=True)
                if rel < tol:
                    return poses, it + 1, hist
                break
            lam *= 10
            if lam > 1e12:
                break
        if not accepted:
            if verbose:
                print(f"  iter {it + 1}: no accepting step, stopping",
                      flush=True)
            return poses, it + 1, hist
    return poses, max_iter, hist


def end_error(poses):
    """Loop end error: last pose vs first pose (translation m, deg)."""
    d = poses[-1][:3, 3] - poses[0][:3, 3]
    R = np.linalg.inv(poses[0][:3, :3]) @ poses[-1][:3, :3]
    return float(np.linalg.norm(d)), rot_angle_deg(R)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", default=os.path.join(
        ROOT, "analysis-drafts", "leftover-three", "orbslam3", "runs",
        "080033_rgbd_only_traj.txt"))
    ap.add_argument("--bag", default=None,
                    help="bag file; default resolves the Oct 27 download")
    ap.add_argument("--outdir", default=HERE)
    ap.add_argument("--loop-weight", type=float, default=1.0)
    ap.add_argument("--skip-render", action="store_true",
                    help="stop after optimization, do not render the map")
    args = ap.parse_args()

    bag = args.bag
    if bag is None:
        cands = sorted(glob.glob(BAG_GLOB))
        assert len(cands) == 1, f"bag glob matched {len(cands)}: {cands}"
        bag = cands[0]
    print(f"bag: {os.path.basename(bag)}", flush=True)
    print(f"trajectory: {args.tum}", flush=True)

    stamps, poses = load_tum(args.tum)
    n = len(poses)
    print(f"ORB poses: {n}", flush=True)
    t0, r0 = end_error(poses)
    print(f"loop end error before: {t0:.4f} m / {r0:.2f} deg", flush=True)

    # ---- stage 1: loop edge from depth + PnP -------------------------
    kept = kept_color_stamps(bag)
    print(f"kept color frames: {len(kept)} (expect 441)", flush=True)
    assert len(kept) == 441, len(kept)
    # Node index for a kept-frame index: frame 0 holds pose 0, frame f
    # holds pose f-1 (same mapping as the adapter script).
    node_of = lambda f: max(f - 1, 0)

    start_frames = list(range(0, 5))
    end_frames = list(range(436, 441))
    want = sorted(set(start_frames + end_frames))
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid_d = db.execute("select id from topics where name=?",
                       (DEPTH,)).fetchone()[0]
    d_stamps = np.array([r[0] for r in db.execute(
        "select timestamp from messages where topic_id=? order by "
        "timestamp", (tid_d,)).fetchall()], dtype=np.int64)
    db.close()
    gray, depth = {}, {}
    for f in want:
        gray[f] = fetch_frame(bag, COLOR, kept[f])
        depth[f] = fetch_frame(bag, DEPTH, kept[f], d_stamps)
    print(f"loaded {len(want)} frames (gray + depth)", flush=True)

    sift = cv2.SIFT_create(nfeatures=10000)
    matcher = cv2.BFMatcher()
    feats = {f: sift.detectAndCompute(gray[f], None) for f in want}

    fwd, rev = [], []
    for e in end_frames:
        for s in start_frames:
            T, st = pair_pose_precomp(feats[e][0], feats[e][1], depth[e],
                                      feats[s][0], feats[s][1], matcher)
            rec = {"end_frame": e, "start_frame": s,
                   "end_node": node_of(e), "start_node": node_of(s),
                   "inliers": st["inliers"], "n_3d": st["n_3d"],
                   "ok": st["ok"]}
            if st["ok"]:
                Z = T  # maps end-frame coords to start-frame coords,
                # i.e. measured A_start^-1 A_end
                E = np.linalg.inv(Z) @ np.linalg.inv(
                    poses[node_of(s)]) @ poses[node_of(e)]
                rec["implied_t_m"] = float(np.linalg.norm(E[:3, 3]))
                rec["implied_rot_deg"] = rot_angle_deg(E[:3, :3])
                rec["T"] = Z.tolist()
            fwd.append(rec)
            # Reverse direction as a consistency check (back-project
            # start-frame keypoints, match to end-frame 2D points).
            Tr, str_ = pair_pose_precomp(
                feats[s][0], feats[s][1], depth[s],
                feats[e][0], feats[e][1], matcher)
            rec_r = {"end_frame": e, "start_frame": s,
                     "inliers": str_["inliers"], "n_3d": str_["n_3d"],
                     "ok": str_["ok"]}
            if str_["ok"]:
                rec_r["T"] = Tr.tolist()
            rev.append(rec_r)
            print(f"  pair end {e} -> start {s}: ok={st['ok']} "
                  f"inliers={st['inliers']} n_3d={st['n_3d']} "
                  + (f"implied loop err "
                     f"{rec.get('implied_t_m', float('nan')):.4f} m / "
                     f"{rec.get('implied_rot_deg', float('nan')):.2f} deg"
                     if st["ok"] else ""), flush=True)

    ok_fwd = [r for r in fwd if r["ok"]]
    print(f"forward pairs ok: {len(ok_fwd)}/{len(fwd)}", flush=True)
    assert ok_fwd, "no forward loop pair succeeded"
    best = max(ok_fwd, key=lambda r: r["inliers"])
    med_t = float(np.median([r["implied_t_m"] for r in ok_fwd]))
    med_r = float(np.median([r["implied_rot_deg"] for r in ok_fwd]))
    print(f"best loop pair: end frame {best['end_frame']} -> start frame "
          f"{best['start_frame']} (nodes {best['end_node']} -> "
          f"{best['start_node']}), inliers {best['inliers']}, "
          f"implied loop err {best['implied_t_m']:.4f} m / "
          f"{best['implied_rot_deg']:.2f} deg", flush=True)
    print(f"median implied loop err over {len(ok_fwd)} ok pairs: "
          f"{med_t:.4f} m / {med_r:.2f} deg", flush=True)
    # Reverse-direction agreement on the best pair.
    best_rev = [r for r in rev if r["end_frame"] == best["end_frame"]
                and r["start_frame"] == best["start_frame"]][0]
    if best_rev["ok"]:
        Zf = np.array(best["T"])
        Zr = np.array(best_rev["T"])
        E = Zf @ Zr  # should be identity if both directions agree
        print(f"reverse check (best pair): inliers {best_rev['inliers']}, "
              f"fwd@rev vs identity: {np.linalg.norm(E[:3, 3]):.4f} m / "
              f"{rot_angle_deg(E[:3, :3]):.2f} deg", flush=True)
        best["reverse_inliers"] = best_rev["inliers"]
        best["reverse_agree_t_m"] = float(np.linalg.norm(E[:3, 3]))
        best["reverse_agree_rot_deg"] = rot_angle_deg(E[:3, :3])

    with open(os.path.join(args.outdir, "loop_pairs.json"), "w") as f:
        json.dump({"forward": fwd, "reverse": rev,
                   "median_implied_t_m": med_t,
                   "median_implied_rot_deg": med_r}, f, indent=1)
    print("wrote loop_pairs.json", flush=True)

    # ---- stage 2: pose graph ----------------------------------------
    Z_loop = np.array(best["T"])
    loop_edge = (best["start_node"], best["end_node"], Z_loop,
                 args.loop_weight)
    edges = build_edges(poses, loop_edge)
    print(f"graph: {n} nodes, {len(edges)} edges "
          f"(439 odometry + 1 loop, loop weight {args.loop_weight})",
          flush=True)
    rms0 = rms_split(poses, edges)
    print(f"RMS edge residual before: all {rms0['all'][0]:.5f} m / "
          f"{rms0['all'][1]:.3f} deg; loop "
          f"{rms0['loop'][0]:.4f} m / {rms0['loop'][1]:.2f} deg", flush=True)

    poses_ref, n_iter, hist = lm_optimize(poses, edges)
    t1, r1 = end_error(poses_ref)
    rms1 = rms_split(poses_ref, edges)
    print(f"LM: {n_iter} iterations, cost {hist[0]:.6g} -> "
          f"{hist[-1]:.6g}", flush=True)
    print(f"loop end error after: {t1:.4f} m / {r1:.2f} deg", flush=True)
    print(f"RMS edge residual after: all {rms1['all'][0]:.5f} m / "
          f"{rms1['all'][1]:.3f} deg; chain "
          f"{rms1['chain'][0]:.5f} m / {rms1['chain'][1]:.3f} deg; loop "
          f"{rms1['loop'][0]:.4f} m / {rms1['loop'][1]:.2f} deg", flush=True)
    disp = np.array([np.linalg.norm(poses_ref[k][:3, 3] - poses[k][:3, 3])
                     for k in range(n)])
    print(f"node displacement: median {np.median(disp) * 1000:.2f} mm, "
          f"max {disp.max() * 1000:.2f} mm at node {int(disp.argmax())}",
          flush=True)
    for k in (0, 110, 220, 330, 439):
        print(f"  displacement node {k}: {disp[k] * 1000:.2f} mm",
              flush=True)

    results = {
        "loop_edge": {k: v for k, v in best.items() if k != "T"},
        "loop_edge_Z": Z_loop.tolist(),
        "loop_weight": args.loop_weight,
        "median_implied_t_m": med_t,
        "median_implied_rot_deg": med_r,
        "end_error_before_m": t0, "end_error_before_deg": r0,
        "end_error_after_m": t1, "end_error_after_deg": r1,
        "rms_before": {k: list(v) for k, v in rms0.items()},
        "rms_after": {k: list(v) for k, v in rms1.items()},
        "lm_iterations": n_iter,
        "cost_before": hist[0], "cost_after": hist[-1],
        "node_disp_median_mm": float(np.median(disp) * 1000),
        "node_disp_max_mm": float(disp.max() * 1000),
        "node_disp_argmax": int(disp.argmax()),
        "node_disp_mm": [float(d * 1000) for d in disp],
    }
    with open(os.path.join(args.outdir, "refinement_results.json"),
              "w") as f:
        json.dump(results, f, indent=1)

    # ---- stage 3: refined TUM -> adapter -> render -------------------
    tum_ref = os.path.join(args.outdir,
                           "080033_rgbd_only_traj_refined.txt")
    write_tum(tum_ref, stamps, poses_ref)
    print(f"wrote {tum_ref}", flush=True)
    if args.skip_render:
        print("skip-render: stopping before map render", flush=True)
        return

    adapter = os.path.join(ROOT, "analysis-drafts", "pose-graph-080033",
                           "orbslam3-pose", "orbslam3_pose_render.py")
    poses_json = os.path.join(args.outdir,
                              "orbslam3_080033_poses_refined.json")
    subprocess.run([sys.executable, adapter, "--tum", tum_ref,
                    "--bag", bag, "--out", poses_json], check=True)
    prefix = os.path.join(args.outdir, "orbslam3_080033_refined_v2")
    subprocess.run([sys.executable, os.path.join(STITCH, "render_v2.py"),
                    "--poses", poses_json, "--bag", bag,
                    "--out", prefix, "--traj"], check=True)
    print("render done", flush=True)


if __name__ == "__main__":
    main()
