#!/usr/bin/env python3
"""Shared loading, preintegration, and optimizer for the imu-init work.

Reuses the machinery of the preint work (load_synced, load_imu,
rotation helpers, build_D_table) by importing preint.py; adds a
gravity-free preintegrator (needed for the joint fit, where gravity is
a free world vector instead of a subtracted constant), a small
numpy-only Levenberg-Marquardt optimizer, and the per-take data bundle.

Frame convention (verified from the bag tf topics, see PROGRESS.md):
the IMU-to-depth rotation is the identity and the IMU origin expressed
in the depth frame is t = (-0.030220, 0.007400, 0.016020) m. ORB poses
are depth-frame absolute poses. IMU-frame absolute poses are
R_imu = R_cam and p_imu = p_cam - R_cam @ t.
"""
import json
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_AUDIT = os.path.dirname(_HERE)
_ROOT = os.path.dirname(os.path.dirname(_AUDIT))
sys.path.insert(0, os.path.join(_AUDIT, "imu-preint"))
sys.path.insert(0, os.path.join(_ROOT, "dataset-tools", "bag-exploration"))

from preint import (  # noqa: E402
    build_D_table,
    load_imu,
    load_synced,
    rot_angle_deg,
    rotvec_to_R,
)

BAGS = {
    "20261007_080033": os.path.join(
        _ROOT, "..", "browser_downloads", "sess-1861756418",
        "browser-download-20261027T081611.443515079Z-1-20261007_080033.db3"),
    "20261007_075947": os.path.join(
        _ROOT, "..", "browser_downloads", "sess-1861756418",
        "browser-download-20261027T083540.215410135Z-6-20261007_075947.db3"),
    "20260926_155419": os.path.join(
        _ROOT, "..", "browser_downloads", "sess-1861756418",
        "browser-download-20261027T082609.603141732Z-3-20260926_155419.db3"),
}
ORB = {
    "20261007_080033": os.path.join(
        _AUDIT, "imu-preint", "runs", "080033_orb_poses.json"),
    "20261007_075947": os.path.join(
        _AUDIT, "imu-preint", "runs", "075947_orb_poses.json"),
    "20260926_155419": os.path.join(
        _AUDIT, "orb-maps", "runs", "155419_orb_poses.json"),
}
PREINT = {
    "20261007_080033": os.path.join(
        _AUDIT, "imu-preint", "runs", "20261007_080033_preint.json"),
    "20261007_075947": os.path.join(
        _AUDIT, "imu-preint", "runs", "20261007_075947_preint.json"),
    "20260926_155419": os.path.join(
        _AUDIT, "imu-preint", "runs", "20260926_155419_preint.json"),
}

STRIDE = 5
SHIFT_MS = -50.0
IMU_TO_DEPTH_T = np.array([-0.030220, 0.007400, 0.016020])
G_MAG = 9.81


def log_R(R):
    """Rotation vector of R (cv2 Rodrigues, R = Exp(rvec))."""
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    return rvec.reshape(3)


def load_take(take):
    """Bag + ORB + preint record for one take.

    Returns a dict with paired colour stamps, gyro/accel streams, ORB
    absolute poses (4x4 arrays, indexed by paired colour frame), the
    stride-5 chosen keyframe indices, and the preint JSON record.
    """
    bag = os.path.normpath(BAGS[take])
    data = load_synced(bag)
    imu = load_imu(bag)
    with open(ORB[take]) as f:
        orb = json.load(f)
    poses = [np.array(p, dtype=np.float64) for p in orb["absolute_poses"]]
    with open(PREINT[take]) as f:
        pre = json.load(f)
    n = len(data["stamps"])
    assert len(poses) == n, (take, len(poses), n)
    idx = list(range(0, n, STRIDE))
    return {"take": take, "bag": bag, "stamps": data["stamps"],
            "gyro_ts": imu["gyro"][0], "gyro_vec": imu["gyro"][1],
            "accel_ts": imu["accel"][0], "accel_vec": imu["accel"][1],
            "orb_poses": poses, "orb_meta": orb, "preint": pre,
            "chosen": idx}


def imu_origin_poses(take_data):
    """ORB absolute poses re-expressed at the IMU origin.

    R_imu = R_cam, p_imu = p_cam - R_cam @ t (ORB-SLAM3 T_b_c1
    body-to-camera convention). Returns (R_list, p_list).
    """
    R_list, p_list = [], []
    for A in take_data["orb_poses"]:
        R = A[:3, :3]
        R_list.append(R)
        p_list.append(A[:3, 3] - R @ IMU_TO_DEPTH_T)
    return R_list, p_list


def pair_samples(take_data, t_a_ns, t_b_ns):
    """Gyro samples and accel-on-gyro-grid for one shifted pair window.

    Returns (ts, w_raw, f) with ts the gyro stamps in the window, w_raw
    the raw gyro vectors, and f the accel vectors linearly interpolated
    onto ts. Returns None when fewer than 2 gyro samples fall inside.
    """
    sh = SHIFT_MS * 1e6
    g_ts, g_vec = take_data["gyro_ts"], take_data["gyro_vec"]
    a_ts, a_vec = take_data["accel_ts"], take_data["accel_vec"]
    m = (g_ts >= t_a_ns + sh) & (g_ts <= t_b_ns + sh)
    idx = np.where(m)[0]
    if len(idx) < 2:
        return None
    ts = g_ts[idx]
    w = g_vec[idx]
    f = np.column_stack([np.interp(ts, a_ts, a_vec[:, c])
                         for c in range(3)])
    return ts, w, f


def integrate_gyro_R(ts, w_raw, b_g):
    """Relative rotation R_points = Exp(-int (w - b_g) dt), incremental
    trapezoid composition, the preint.py rotation convention."""
    w = w_raw - b_g
    R_rel = np.eye(3)
    R_steps = [np.eye(3)]
    for k in range(len(ts) - 1):
        dt = (ts[k + 1] - ts[k]) / 1e9
        wm = 0.5 * (w[k] + w[k + 1])
        R_rel = R_rel @ rotvec_to_R(wm * dt)
        R_steps.append(R_rel.copy())
    return R_rel.T, R_steps


def preintegrate(ts, w_raw, f, b_g, b_a):
    """Gravity-free preintegration of one pair window.

    Returns (dR, dv, dp) in the pair-start frame: dR is the rotation of
    points, dv/dp the velocity/position deltas from the bias-corrected
    accel only (no gravity subtracted; gravity enters the residual).
    """
    dR, R_steps = integrate_gyro_R(ts, w_raw, b_g)
    a_body = np.array([R_steps[k] @ (f[k] - b_a)
                       for k in range(len(ts))])
    dv = np.zeros(3)
    dp = np.zeros(3)
    v = np.zeros(3)
    for k in range(len(ts) - 1):
        dt = (ts[k + 1] - ts[k]) / 1e9
        a_mid = 0.5 * (a_body[k] + a_body[k + 1])
        v_new = v + a_mid * dt
        dp += 0.5 * (v + v_new) * dt
        v = v_new
    return dR, v, dp


def lm(fun, x0, max_iter=50, lam0=1e-3, tol_cost=1e-9, tol_step=1e-10,
       fd_steps=None, verbose=False):
    """Hand-rolled Levenberg-Marquardt, numpy only.

    fun(x) -> residual vector. Forward-difference Jacobian with
    per-parameter steps fd_steps (default 1e-6). Returns
    (x, cost, n_iter, n_fun, history).
    """
    x = np.array(x0, dtype=np.float64)
    n = len(x)
    steps = (np.full(n, 1e-6) if fd_steps is None
             else np.asarray(fd_steps, dtype=np.float64))
    r = fun(x)
    cost = float(r @ r)
    lam = lam0
    history = [cost]
    n_fun = 1
    n_iter = 0
    for it in range(max_iter):
        n_iter = it + 1
        J = np.empty((len(r), n))
        for j in range(n):
            dx = np.zeros(n)
            dx[j] = steps[j]
            J[:, j] = (fun(x + dx) - r) / steps[j]
            n_fun += 1
        JtJ = J.T @ J
        Jtr = J.T @ r
        accepted = False
        for _ in range(30):
            A = JtJ + lam * np.diag(np.maximum(np.diag(JtJ), 1e-12))
            try:
                step = np.linalg.solve(A, -Jtr)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            x_new = x + step
            r_new = fun(x_new)
            n_fun += 1
            cost_new = float(r_new @ r_new)
            if cost_new < cost:
                rel = (cost - cost_new) / max(cost, 1e-300)
                x, r, cost = x_new, r_new, cost_new
                lam = max(lam / 10.0, 1e-12)
                history.append(cost)
                accepted = True
                if verbose:
                    print(f"  lm it {it}: cost {cost:.6g} "
                          f"rel {rel:.2e} |step| {np.max(np.abs(step)):.2e}",
                          flush=True)
                if rel < tol_cost or np.max(np.abs(step)) < tol_step:
                    return x, cost, n_iter, n_fun, history
                break
            lam *= 10.0
            if lam > 1e12:
                break
        if not accepted:
            break
    return x, cost, n_iter, n_fun, history
