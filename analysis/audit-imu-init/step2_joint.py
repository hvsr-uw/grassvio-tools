#!/usr/bin/env python3
"""Step 2: joint refinement of gravity direction, biases, and velocities
against fixed ORB visual poses (GrassVIO imu-init).

Per short window (12 stride-5 pairs, 2.0 s; 155419 uses its 10 pairs):
state = gravity direction (2 tangent parameters at the prior-mean
direction, |g| fixed at 9.81), gyro bias (3), accel bias (3), and one
velocity per keyframe boundary. Residuals are the standard preintegration
residuals (rotation, velocity, position) between consecutive keyframes,
with the visual poses held fixed. Gravity-free preintegration (accel
bias-corrected in the body frame, no gravity subtracted inside).

Modes:
  baseline   primary init, baseline priors (per PROGRESS.md).
  perturbed  test (a1): gravity tilted 10 deg about world x, b_g = 0,
             b_a = 0 at init.
  weakprior  test (b): bias prior sigmas x4.
  shifted    test (a2): windows starting at pairs 6, 18, 30, ...
  holdout    tests (c) and (d): frozen first-window parameters tested on
             withheld windows (velocity-only refit) and on short
             predictions against a constant-velocity baseline.

Optimizer: hand-rolled Levenberg-Marquardt from common.py, numpy only.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (  # noqa: E402
    G_MAG,
    IMU_TO_DEPTH_T,
    build_D_table,
    imu_origin_poses,
    load_take,
    lm,
    log_R,
    pair_samples,
    preintegrate,
    rot_angle_deg,
    rotvec_to_R,
)

SIG_ROT = 0.003   # rad
SIG_VEL = 0.01    # m/s
SIG_POS = 0.003   # m
SIG_GDIR = np.radians(5.0)
SIG_BG = 0.01     # rad/s per axis
SIG_BA = 0.05     # m/s^2 per axis
WINDOW_PAIRS = 12


def tangent_basis(g0):
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(g0 @ a)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(g0, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(g0, e1)
    return e1, e2


def gravity_from_tangent(g0, e1, e2, u, v):
    g = g0 + u * e1 + v * e2
    return g / np.linalg.norm(g)


def gravity_prior_mean(td, b_g, keyframe_stamp_ns):
    """Preint stillest-window gravity direction, in world coordinates.

    g_ref is gravity in body coordinates at t_ref; carry it to the
    keyframe time with the gyro-chained D-table (using the step-1 bias),
    then rotate into the world frame with the ORB pose at that keyframe.
    Returns a unit vector (magnitude reset to 9.81 by the caller).
    """
    pre = td["preint"]
    grav = pre["gravity"]
    g_ref = np.array(grav["g_ref"], dtype=np.float64)
    g_ts = td["gyro_ts"]
    t_ref_ns = g_ts[0] + grav["t_ref_s"] * 1e9
    D, _ = build_D_table(g_ts, td["gyro_vec"], b_g, t_ref_ns)
    r0 = int(np.argmin(np.abs(g_ts - keyframe_stamp_ns)))
    g_body = D[r0].T @ g_ref
    # ORB absolute rotation at the keyframe nearest this stamp
    stamps = td["stamps"]
    k = int(np.argmin(np.abs(stamps - keyframe_stamp_ns)))
    R_w = td["orb_poses"][k][:3, :3]
    g_world = R_w @ g_body
    return g_world / np.linalg.norm(g_world)


def build_window(td, start_pair, n_pairs, b_g_step1):
    """Per-pair preintegration inputs and visual poses for one window."""
    stamps = td["stamps"]
    idx = td["chosen"]
    R_list, p_list = imu_origin_poses(td)
    pairs = []
    for k in range(start_pair, start_pair + n_pairs):
        i, j = idx[k], idx[k + 1]
        s = pair_samples(td, float(stamps[i]), float(stamps[j]))
        if s is None:
            raise RuntimeError(f"pair {k} ({i},{j}) has <2 gyro samples")
        R_i, R_j = R_list[i], R_list[j]
        pairs.append({
            "k": k, "i": i, "j": j,
            "ts": s[0], "w": s[1], "f": s[2],
            "R_i": R_i, "R_j": R_j,
            "p_i": p_list[i], "p_j": p_list[j],
            "R_vis": R_j.T @ R_i,
            "dt": float((stamps[j] - stamps[i]) / 1e9),
        })
    kf_idx = idx[start_pair:start_pair + n_pairs + 1]
    kf_t = np.array([stamps[q] for q in kf_idx], dtype=np.float64) / 1e9
    kf_p = np.array([p_list[q] for q in kf_idx])
    # visual finite-difference velocities (central, one-sided at ends)
    v_init = np.zeros_like(kf_p)
    for a in range(len(kf_idx)):
        if a == 0:
            v_init[a] = (kf_p[1] - kf_p[0]) / (kf_t[1] - kf_t[0])
        elif a == len(kf_idx) - 1:
            v_init[a] = (kf_p[-1] - kf_p[-2]) / (kf_t[-1] - kf_t[-2])
        else:
            v_init[a] = (kf_p[a + 1] - kf_p[a - 1]) / (kf_t[a + 1] - kf_t[a - 1])
    g0 = gravity_prior_mean(td, b_g_step1, float(stamps[kf_idx[0]]))
    e1, e2 = tangent_basis(g0)
    return {"start_pair": start_pair, "n_pairs": n_pairs,
            "pairs": pairs, "kf_idx": kf_idx, "kf_t": kf_t, "kf_p": kf_p,
            "v_init": v_init, "g0": g0, "e1": e1, "e2": e2}


def unpack(wd, x):
    K = len(wd["kf_idx"])
    u, v = x[0], x[1]
    b_g = x[2:5]
    b_a = x[5:8]
    vels = x[8:].reshape(K, 3)
    g_dir = gravity_from_tangent(wd["g0"], wd["e1"], wd["e2"], u, v)
    return g_dir, b_g, b_a, vels


def data_residuals(wd, x):
    """Unweighted per-pair residuals (rot rad, vel m/s, pos m)."""
    g_dir, b_g, b_a, vels = unpack(wd, x)
    g = G_MAG * g_dir
    r_rot, r_vel, r_pos = [], [], []
    for n, p in enumerate(wd["pairs"]):
        dR, dv, dp = preintegrate(p["ts"], p["w"], p["f"], b_g, b_a)
        r_rot.append(log_R(p["R_vis"].T @ dR))
        r_vel.append(p["R_i"].T @ (vels[n + 1] - vels[n] - g * p["dt"]) - dv)
        r_pos.append(p["R_i"].T @ (p["p_j"] - p["p_i"] - vels[n] * p["dt"]
                                   - 0.5 * g * p["dt"] ** 2) - dp)
    return (np.concatenate(r_rot), np.concatenate(r_vel),
            np.concatenate(r_pos))


def make_residual_fn(wd, sig_bg, sig_ba, b_g_prior, freeze=None):
    """Weighted residual vector incl. priors. freeze=(g_dir,b_g,b_a):
    when given, x is velocities only and the frozen values are used."""
    K = len(wd["kf_idx"])
    if freeze is None:
        def fun(x):
            g_dir, b_g, b_a, vels = unpack(wd, x)
            rr, rv, rp = data_residuals(wd, x)
            prior = np.concatenate([
                x[0:2] / SIG_GDIR,
                (b_g - b_g_prior) / sig_bg,
                b_a / sig_ba])
            return np.concatenate([rr / SIG_ROT, rv / SIG_VEL,
                                   rp / SIG_POS, prior])
        return fun
    g_dir_f, b_g_f, b_a_f = freeze

    def fun(v_flat):
        x = np.zeros(8 + 3 * K)
        x[2:5] = b_g_f
        x[5:8] = b_a_f
        x[8:] = v_flat
        # exact tangent parameters for the frozen gravity direction:
        # normalize(g0 + u e1 + v e2) == g_dir_f
        x[0] = float((g_dir_f @ wd["e1"]) / (g_dir_f @ wd["g0"]))
        x[1] = float((g_dir_f @ wd["e2"]) / (g_dir_f @ wd["g0"]))
        rr, rv, rp = data_residuals(wd, x)
        return np.concatenate([rr / SIG_ROT, rv / SIG_VEL, rp / SIG_POS])
    return fun


def rms(a):
    a = np.asarray(a, dtype=np.float64)
    return float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")


def fit_window(td, wd, b_g_step1, mode, sig_bg=SIG_BG, sig_ba=SIG_BA):
    K = len(wd["kf_idx"])
    x0 = np.zeros(8 + 3 * K)
    x0[2:5] = b_g_step1
    x0[8:] = wd["v_init"].reshape(-1)
    if mode == "perturbed":
        # gravity tilted 10 deg about the world x-axis; biases zeroed
        g_tilt = rotvec_to_R(np.array([np.radians(10.0), 0.0, 0.0])) @ wd["g0"]
        x0[0] = float((g_tilt @ wd["e1"]) / (g_tilt @ wd["g0"]))
        x0[1] = float((g_tilt @ wd["e2"]) / (g_tilt @ wd["g0"]))
        x0[2:5] = 0.0
    fun = make_residual_fn(wd, sig_bg, sig_ba, b_g_step1)
    cost0 = float(fun(x0) @ fun(x0))
    fd = np.concatenate([[1e-4, 1e-4], np.full(3, 1e-5), np.full(3, 1e-4),
                         np.full(3 * K, 1e-4)])
    x, cost, n_iter, n_fun, _ = lm(fun, x0, max_iter=50, fd_steps=fd)
    g_dir, b_g, b_a, vels = unpack(wd, x)
    rr, rv, rp = data_residuals(wd, x)
    tilt_from_prior = float(np.degrees(np.arccos(np.clip(
        float(g_dir @ wd["g0"]), -1.0, 1.0))))
    return {
        "start_pair": wd["start_pair"], "n_pairs": wd["n_pairs"],
        "mode": mode,
        "g_dir": g_dir.tolist(),
        "g_prior_mean": wd["g0"].tolist(),
        "tilt_from_prior_deg": tilt_from_prior,
        "b_g_rad_s": b_g.tolist(),
        "b_a_m_s2": b_a.tolist(),
        "velocities_m_s": vels.tolist(),
        "cost_initial": cost0, "cost_final": float(cost),
        "n_iter": int(n_iter), "n_fun": int(n_fun),
        "rms_rot_deg": float(np.degrees(rms(rr))),
        "rms_vel_m_s": rms(rv),
        "rms_pos_m": rms(rp),
        "rms_rot_rad": rms(rr),
    }


def window_starts(n_pairs_total, mode):
    if mode == "shifted":
        starts = []
        s = 6
        while s + WINDOW_PAIRS <= n_pairs_total:
            starts.append(s)
            s += 12
        return starts
    starts = []
    s = 0
    while s + WINDOW_PAIRS <= n_pairs_total:
        starts.append(s)
        s += WINDOW_PAIRS
    return starts


def run_fit(take, mode):
    td = load_take(take)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "runs", f"{take}_step1.json")) as f:
        s1 = json.load(f)
    b_g_step1 = np.array(s1["fits"]["from_zero"]["bias_rad_s"])
    n_pairs = len(td["chosen"]) - 1
    starts = window_starts(n_pairs, mode)
    if not starts:  # short take: single window over all pairs
        starts = [0]
    sig_bg, sig_ba = SIG_BG, SIG_BA
    if mode == "weakprior":
        sig_bg, sig_ba = 0.04, 0.20
    out = {"take": take, "mode": mode, "n_pairs_total": n_pairs,
           "sig_bg_rad_s": sig_bg, "sig_ba_m_s2": sig_ba,
           "sig_rot_rad": SIG_ROT, "sig_vel_m_s": SIG_VEL,
           "sig_pos_m": SIG_POS, "sig_gdir_deg": 5.0,
           "b_g_step1_rad_s": b_g_step1.tolist(), "windows": []}
    for s in starts:
        n = min(WINDOW_PAIRS, n_pairs - s)
        wd = build_window(td, s, n, b_g_step1)
        res = fit_window(td, wd, b_g_step1, mode, sig_bg, sig_ba)
        out["windows"].append(res)
        print(f"{take} mode={mode} window@{s} ({n} pairs): "
              f"g tilt from prior {res['tilt_from_prior_deg']:.2f} deg, "
              f"b_g={np.round(res['b_g_rad_s'], 5)} "
              f"b_a={np.round(res['b_a_m_s2'], 4)} "
              f"rms rot {res['rms_rot_deg']:.4f} deg "
              f"vel {res['rms_vel_m_s']:.4f} m/s "
              f"pos {res['rms_pos_m']*1000:.2f} mm "
              f"cost {res['cost_initial']:.1f}->{res['cost_final']:.2f} "
              f"({res['n_iter']} it)", flush=True)
    return out


def predict(td, frozen, start_kf, horizon_pairs):
    """Short prediction from keyframe start_kf over horizon_pairs pairs.

    frozen = (g_dir, b_g, b_a). Returns (imu_pred_pos, cv_pred_pos,
    true_pos) at the IMU origin; None if the horizon runs past the end.
    """
    g_dir, b_g, b_a = frozen
    g = G_MAG * g_dir
    idx = td["chosen"]
    stamps = td["stamps"]
    R_list, p_list = imu_origin_poses(td)
    if start_kf + horizon_pairs >= len(idx):
        return None
    k = start_kf
    p0 = p_list[idx[k]]
    v0 = ((p_list[idx[k]] - p_list[idx[k - 2]])
          / ((stamps[idx[k]] - stamps[idx[k - 2]]) / 1e9))
    dt_tot = float((stamps[idx[k + horizon_pairs]] - stamps[idx[k]]) / 1e9)
    cv_pos = p0 + v0 * dt_tot
    p, v, R = p0.copy(), v0.copy(), R_list[idx[k]].copy()
    for h in range(horizon_pairs):
        i, j = idx[k + h], idx[k + h + 1]
        s = pair_samples(td, float(stamps[i]), float(stamps[j]))
        if s is None:
            return None
        dt = float((stamps[j] - stamps[i]) / 1e9)
        dR, dv, dp = preintegrate(s[0], s[1], s[2], b_g, b_a)
        p = p + v * dt + 0.5 * g * dt ** 2 + R @ dp
        v = v + g * dt + R @ dv
        R = R @ dR.T
    return p, cv_pos, p_list[idx[k + horizon_pairs]]


def tilt_range(td, start_pair, n_pairs, g_dir):
    """Max-min tilt over a window; tilt_k = angle between camera z-axis
    in world coordinates and the gravity direction."""
    idx = td["chosen"]
    tilts = []
    for k in range(start_pair, start_pair + n_pairs + 1):
        R = td["orb_poses"][idx[k]][:3, :3]
        z_world = R @ np.array([0.0, 0.0, 1.0])
        tilts.append(float(np.degrees(np.arccos(np.clip(
            float(z_world @ g_dir), -1.0, 1.0)))))
    return float(max(tilts) - min(tilts)), float(np.mean(tilts))


def run_holdout(take, baseline_path):
    td = load_take(take)
    with open(baseline_path) as f:
        base = json.load(f)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "runs", f"{take}_step1.json")) as f:
        s1 = json.load(f)
    b_g_step1 = np.array(s1["fits"]["from_zero"]["bias_rad_s"])
    n_pairs = len(td["chosen"]) - 1
    if n_pairs < 2 * WINDOW_PAIRS:
        print(f"{take}: only {n_pairs} pairs, no withheld window; "
              "tests (c) and (d) not applicable", flush=True)
        return {"take": take, "holdout": "not_applicable",
                "n_pairs_total": n_pairs}
    w0 = base["windows"][0]
    frozen = (np.array(w0["g_dir"]), np.array(w0["b_g_rad_s"]),
              np.array(w0["b_a_m_s2"]))
    out = {"take": take, "holdout": "run", "n_pairs_total": n_pairs,
           "frozen_from_window0": {
               "g_dir": frozen[0].tolist(), "b_g_rad_s": frozen[1].tolist(),
               "b_a_m_s2": frozen[2].tolist()},
           "test_c": {}, "test_d": {}}
    # ---- test (c): velocity-only refit on withheld windows ----
    g0_w1 = gravity_prior_mean(
        td, b_g_step1,
        float(td["stamps"][td["chosen"][WINDOW_PAIRS]]))
    baseline_params = (g0_w1, b_g_step1, np.zeros(3))
    cands = []
    s = WINDOW_PAIRS
    while s + WINDOW_PAIRS <= n_pairs:
        cands.append(s)
        s += WINDOW_PAIRS
    # tilt range per candidate window, using the frozen gravity direction
    tilt_info = []
    for s in cands:
        tr, tm = tilt_range(td, s, WINDOW_PAIRS, frozen[0])
        tilt_info.append({"start_pair": s, "tilt_range_deg": tr,
                          "tilt_mean_deg": tm})
    out["test_c"]["candidate_windows_tilt"] = tilt_info
    max_tilt_start = max(tilt_info,
                         key=lambda d: d["tilt_range_deg"])["start_pair"]
    for label, s in (("next_window_12", WINDOW_PAIRS),
                     ("max_tilt_window", max_tilt_start)):
        wd = build_window(td, s, WINDOW_PAIRS, b_g_step1)
        K = len(wd["kf_idx"])
        for pname, params in (("fitted", frozen),
                              ("baseline", baseline_params)):
            fun = make_residual_fn(wd, SIG_BG, SIG_BA, b_g_step1,
                                   freeze=params)
            v0 = wd["v_init"].reshape(-1)
            c0 = float(fun(v0) @ fun(v0))
            x, cost, n_iter, n_fun, _ = lm(
                fun, v0, max_iter=50, fd_steps=np.full(3 * K, 1e-4))
            # unweighted physical RMS at the solution
            xx = np.zeros(8 + 3 * K)
            xx[2:5] = params[1]
            xx[5:8] = params[2]
            xx[0] = float((params[0] @ wd["e1"]) / (params[0] @ wd["g0"]))
            xx[1] = float((params[0] @ wd["e2"]) / (params[0] @ wd["g0"]))
            xx[8:] = x
            rr, rv, rp = data_residuals(wd, xx)
            key = f"{label}@{s}_{pname}"
            out["test_c"][key] = {
                "start_pair": s, "cost_initial": c0, "cost_final": float(cost),
                "n_iter": int(n_iter),
                "rms_rot_deg": float(np.degrees(rms(rr))),
                "rms_vel_m_s": rms(rv), "rms_pos_m": rms(rp),
                "tilt_range_deg": [t["tilt_range_deg"] for t in tilt_info
                                   if t["start_pair"] == s][0]}
            print(f"{take} test(c) {key}: rms rot "
                  f"{out['test_c'][key]['rms_rot_deg']:.4f} deg, vel "
                  f"{out['test_c'][key]['rms_vel_m_s']:.4f} m/s, pos "
                  f"{out['test_c'][key]['rms_pos_m']*1000:.2f} mm "
                  f"(cost {c0:.1f}->{cost:.2f})", flush=True)
    # ---- test (d): short predictions vs constant velocity ----
    for H in (1, 2, 4, 8):
        errs_imu, errs_cv = [], []
        for k in range(12, len(td["chosen"]) - H):
            r = predict(td, frozen, k, H)
            if r is None:
                continue
            p_imu, p_cv, p_true = r
            errs_imu.append(float(np.linalg.norm(p_imu - p_true)))
            errs_cv.append(float(np.linalg.norm(p_cv - p_true)))
        out["test_d"][f"H{H}"] = {
            "n_starts": len(errs_imu),
            "median_err_imu_m": float(np.median(errs_imu)) if errs_imu else None,
            "median_err_cv_m": float(np.median(errs_cv)) if errs_cv else None,
            "mean_err_imu_m": float(np.mean(errs_imu)) if errs_imu else None,
            "mean_err_cv_m": float(np.mean(errs_cv)) if errs_cv else None,
        }
        d = out["test_d"][f"H{H}"]
        print(f"{take} test(d) H={H}: n={d['n_starts']} median IMU "
              f"{d['median_err_imu_m']*1000:.2f} mm vs CV "
              f"{d['median_err_cv_m']*1000:.2f} mm", flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--take", required=True)
    ap.add_argument("--mode", required=True,
                    choices=["baseline", "perturbed", "weakprior",
                             "shifted", "holdout"])
    ap.add_argument("--baseline", default=None,
                    help="baseline fit JSON (for holdout mode)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    if args.mode == "holdout":
        if args.baseline is None:
            raise SystemExit("holdout mode needs --baseline")
        out = run_holdout(args.take, args.baseline)
    else:
        out = run_fit(args.take, args.mode)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(f"wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
