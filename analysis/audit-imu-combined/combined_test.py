#!/usr/bin/env python3
"""Combined-correction withheld-motion test (GrassVIO imu-init).

The combined model does NOT separate gravity direction from accel
bias: gravity is FROZEN at the fitted direction from the init work
(window-0 step-2 baseline fit) and ONE effective accel correction
b_a is fitted per take (per window for the stability check) as a
single specific-force correction. Gyro bias is frozen at the step-1
fitted value. The fitted correction is then used to predict WITHHELD
motion (never fitted on the test segment) and compared against a
constant-velocity baseline and two ablated IMU variants.

Pre-specification, thresholds, segments, horizons, and pass criteria
are in PROGRESS.md, written before any fit ran. This script is fresh:
the old step2_joint.py holdout code was written but never executed
and does not match this design (it froze the window-0 joint triple,
used 1/2/4/8-pair horizons, compared only IMU vs CV, and excluded
155419). Only the verified machinery in common.py is reused
(load_take, pair_samples, preintegrate, lm, imu_origin_poses) plus
build_window from step2_joint.py for per-window pair lists and visual
finite-difference velocities. The propagation math follows the
verified conventions in common.py: dv/dp are in the pair-start body
frame, so R @ dv / R @ dp map them to world; dR maps frame-1 vectors
to frame-2 vectors, so R_new = R_old @ dR.T.

Optimizer: hand-rolled Levenberg-Marquardt from common.py, numpy only.
"""
import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
sys.path.insert(0, _PARENT)
from common import (  # noqa: E402
    G_MAG,
    imu_origin_poses,
    lm,
    load_take,
    pair_samples,
    preintegrate,
)
from step2_joint import build_window, data_residuals  # noqa: E402

SIG_VEL = 0.01    # m/s
SIG_POS = 0.003   # m
SIG_BA_SWEEP = (0.05, 0.20, 0.80)
WINDOW_PAIRS = 12
HORIZONS_PRIMARY = (1, 2, 6)     # pairs = 0.167 / 0.334 / 1.001 s
HORIZONS_SECONDARY = (4, 8)      # pairs = 0.667 / 1.334 s
HORIZONS = HORIZONS_PRIMARY + HORIZONS_SECONDARY

FIT_SEGMENT = {
    "20261007_080033": (0, 24),   # pairs 0-23 (4.0 s)
    "20261007_075947": (0, 24),   # pairs 0-23 (4.0 s)
    "20260926_155419": (0, 4),    # pairs 0-3 (0.67 s); take is short
}
TEST_START_KF = {
    "20261007_080033": 24,        # first keyframe after the fit segment
    "20261007_075947": 24,
    "20260926_155419": 4,
}


def frozen_g_dir(take):
    """Window-0 fitted gravity direction from the step-2 baseline run."""
    path = os.path.join(_PARENT, "runs", f"{take}_step2_baseline.json")
    with open(path) as f:
        base = json.load(f)
    return np.array(base["windows"][0]["g_dir"], dtype=np.float64)


def step1_bias(take):
    path = os.path.join(_PARENT, "runs", f"{take}_step1.json")
    with open(path) as f:
        s1 = json.load(f)
    return np.array(s1["fits"]["from_zero"]["bias_rad_s"], dtype=np.float64)


def full_x(wd, g_dir_f, b_g, b_a, vels):
    """Full step-2 state vector for a frozen (g_dir, b_g, b_a).

    Tangent parameters are recovered exactly: with u, v as below,
    gravity_from_tangent(g0, e1, e2, u, v) == g_dir_f.
    """
    K = len(wd["kf_idx"])
    x = np.zeros(8 + 3 * K)
    x[0] = float((g_dir_f @ wd["e1"]) / (g_dir_f @ wd["g0"]))
    x[1] = float((g_dir_f @ wd["e2"]) / (g_dir_f @ wd["g0"]))
    x[2:5] = b_g
    x[5:8] = b_a
    x[8:] = np.asarray(vels, dtype=np.float64).reshape(-1)
    return x


def fit_correction(td, start_pair, n_pairs, g_dir_f, b_g, sig_ba):
    """Fit [b_a, velocities] on one segment with gravity and gyro bias
    frozen. Returns a dict with the fitted b_a and fit diagnostics."""
    wd = build_window(td, start_pair, n_pairs, b_g)
    K = len(wd["kf_idx"])
    v0 = wd["v_init"].reshape(-1)

    def fun(y):
        b_a = y[:3]
        vels = y[3:].reshape(K, 3)
        x = full_x(wd, g_dir_f, b_g, b_a, vels)
        _, rv, rp = data_residuals(wd, x)
        return np.concatenate([rv / SIG_VEL, rp / SIG_POS, b_a / sig_ba])

    y0 = np.concatenate([np.zeros(3), v0])
    c0 = float(fun(y0) @ fun(y0))
    y, cost, n_iter, n_fun, _ = lm(
        fun, y0, max_iter=50, fd_steps=np.full(3 + 3 * K, 1e-4))
    b_a = y[:3]
    vels = y[3:].reshape(K, 3)
    x = full_x(wd, g_dir_f, b_g, b_a, vels)
    _, rv, rp = data_residuals(wd, x)
    return {
        "start_pair": start_pair, "n_pairs": n_pairs,
        "sig_ba_m_s2": sig_ba,
        "b_a_m_s2": b_a.tolist(),
        "velocities_m_s": vels.tolist(),
        "cost_initial": c0, "cost_final": float(cost),
        "n_iter": int(n_iter), "n_fun": int(n_fun),
        "rms_vel_m_s": float(np.sqrt(np.mean(rv ** 2))),
        "rms_pos_m": float(np.sqrt(np.mean(rp ** 2))),
    }


def window_starts(n_pairs_total):
    starts = []
    s = 0
    while s + WINDOW_PAIRS <= n_pairs_total:
        starts.append(s)
        s += WINDOW_PAIRS
    return starts


def predict_all(td, start_kf, H, g_dir_f, g0_prior, b_g, b_a_comb):
    """Predictions from keyframe start_kf over H pairs.

    Returns a dict with the predicted positions for the four
    predictors and the true (ORB) position, all at the IMU origin;
    None if the horizon runs past the end or a pair lacks samples.
    v0 uses the 2 preceding keyframes only (no future visual pose).
    """
    idx = td["chosen"]
    stamps = td["stamps"]
    R_list, p_list = imu_origin_poses(td)
    if start_kf + H >= len(idx) or start_kf < 2:
        return None
    k = start_kf
    p0 = p_list[idx[k]]
    v0 = ((p_list[idx[k]] - p_list[idx[k - 2]])
          / ((stamps[idx[k]] - stamps[idx[k - 2]]) / 1e9))
    dt_tot = float((stamps[idx[k + H]] - stamps[idx[k]]) / 1e9)
    p_true = p_list[idx[k + H]]

    def propagate(g_dir, b_a):
        g = G_MAG * g_dir
        p, v, R = p0.copy(), v0.copy(), R_list[idx[k]].copy()
        for h in range(H):
            i, j = idx[k + h], idx[k + h + 1]
            s = pair_samples(td, float(stamps[i]), float(stamps[j]))
            if s is None:
                return None
            dt = float((stamps[j] - stamps[i]) / 1e9)
            dR, dv, dp = preintegrate(s[0], s[1], s[2], b_g, b_a)
            p = p + v * dt + 0.5 * g * dt ** 2 + R @ dp
            v = v + g * dt + R @ dv
            R = R @ dR.T
        return p

    p_comb = propagate(g_dir_f, b_a_comb)
    if p_comb is None:
        return None
    p_unc = propagate(g0_prior, np.zeros(3))
    if p_unc is None:
        return None
    p_gonly = propagate(g_dir_f, np.zeros(3))
    if p_gonly is None:
        return None
    p_cv = p0 + v0 * dt_tot
    return {"imu_combined": p_comb, "cv": p_cv,
            "imu_uncorrected": p_unc, "imu_gravity_only": p_gonly,
            "true": p_true, "dt_total_s": dt_tot}


def run_take(take):
    td = load_take(take)
    b_g = step1_bias(take)
    g_dir_f = frozen_g_dir(take)
    n_pairs = len(td["chosen"]) - 1
    wd0 = build_window(td, 0, min(WINDOW_PAIRS, n_pairs), b_g)
    g0_prior = wd0["g0"]
    fit_start, fit_n = FIT_SEGMENT[take]
    test_k0 = TEST_START_KF[take]

    out = {"take": take, "n_pairs_total": n_pairs,
           "frozen_g_dir": g_dir_f.tolist(),
           "prior_mean_g_dir": g0_prior.tolist(),
           "b_g_step1_rad_s": b_g.tolist(),
           "fit_segment_pairs": [fit_start, fit_start + fit_n - 1],
           "test_start_keyframe": test_k0,
           "per_take_correction": {}, "stability": {}, "predictions": {}}

    # ---- per-take effective correction at each prior sigma ----
    for sig in SIG_BA_SWEEP:
        res = fit_correction(td, fit_start, fit_n, g_dir_f, b_g, sig)
        out["per_take_correction"][str(sig)] = res
        print(f"{take} per-take correction sig={sig}: "
              f"b_a={np.round(res['b_a_m_s2'], 4)} "
              f"rms vel {res['rms_vel_m_s']:.4f} m/s "
              f"pos {res['rms_pos_m']*1000:.2f} mm "
              f"(cost {res['cost_initial']:.1f}->{res['cost_final']:.2f})",
              flush=True)

    # ---- per-window stability fits ----
    starts = window_starts(n_pairs)
    if not starts:
        starts = [0]
    for sig in (0.05, 0.20):
        wins = []
        for s in starts:
            n = min(WINDOW_PAIRS, n_pairs - s)
            r = fit_correction(td, s, n, g_dir_f, b_g, sig)
            wins.append(r)
            print(f"{take} stability window@{s} sig={sig}: "
                  f"b_a={np.round(r['b_a_m_s2'], 4)}", flush=True)
        out["stability"][str(sig)] = wins

    # ---- withheld predictions ----
    n_kf = len(td["chosen"])
    for sig in SIG_BA_SWEEP:
        b_a_comb = np.array(
            out["per_take_correction"][str(sig)]["b_a_m_s2"])
        for H in HORIZONS:
            errs = {m: [] for m in
                    ("imu_combined", "cv", "imu_uncorrected",
                     "imu_gravity_only")}
            n_skip = 0
            for k in range(test_k0, n_kf - H):
                r = predict_all(td, k, H, g_dir_f, g0_prior, b_g,
                                b_a_comb)
                if r is None:
                    n_skip += 1
                    continue
                for m in errs:
                    errs[m].append(float(np.linalg.norm(r[m] - r["true"])))
            d = {"n_starts": len(errs["imu_combined"]),
                 "n_skipped": n_skip}
            for m, e in errs.items():
                if e:
                    d[f"median_{m}_m"] = float(np.median(e))
                    d[f"p90_{m}_m"] = float(np.percentile(e, 90))
                    d[f"mean_{m}_m"] = float(np.mean(e))
                else:
                    d[f"median_{m}_m"] = None
                    d[f"p90_{m}_m"] = None
                    d[f"mean_{m}_m"] = None
            if errs["imu_combined"] and errs["cv"]:
                d["frac_combined_beats_cv"] = float(np.mean(
                    np.array(errs["imu_combined"])
                    < np.array(errs["cv"])))
            else:
                d["frac_combined_beats_cv"] = None
            out["predictions"].setdefault(str(sig), {})[f"H{H}"] = d
            print(f"{take} sig={sig} H={H}: n={d['n_starts']} "
                  f"median comb "
                  f"{(d['median_imu_combined_m']*1000 if d['median_imu_combined_m'] is not None else float('nan')):.2f} mm vs CV "
                  f"{(d['median_cv_m']*1000 if d['median_cv_m'] is not None else float('nan')):.2f} mm, "
                  f"frac beats CV {d['frac_combined_beats_cv']}",
                  flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--take", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = run_take(args.take)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(f"wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
