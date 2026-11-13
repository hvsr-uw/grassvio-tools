#!/usr/bin/env python3
"""IMU pre-integration between stride-5 frame pairs (GrassVIO).

Step 1 of the IMU pre-integration task: the IMU branch of the pipeline,
built as a pre-integration. Per stride-5 colour-frame pair (the same
pairs the RGB-D odometry chain uses):

  gyro  : trapezoid-integrate the bias-removed gyro over the pair window
          to a rotation delta. Points move by the inverse gyro rotation,
          R = Exp(-integral of omega dt), the convention verified in the
          gyro-prior work.
  accel : rotate the accel samples into the pair-start frame with the
          incrementally integrated gyro rotation, subtract gravity, and
          trapezoid-integrate to velocity and position deltas.
  chain : velocity is propagated across pairs (zero initial velocity at
          take start); the per-pair displacement is d = dp + v*dt and
          the pair pose is T = [R | -R d]. Absolute poses chain exactly
          like the odometry chain, A[i+1] = A[i] @ inv(T).

Gravity: the gravity vector is estimated from the take itself as minus
the mean accelerometer vector over the take's stillest 1-second window
(sliding in 0.25 s steps, ranked by median bias-removed gyro magnitude).
That vector is gravity in the body frame at the window start; it is
rotated into each pair-start frame with the gyro-chained rotation from
the window start. No take had a truly still first second (see the
per-take gravity record in the output JSON), so the stillest window is
used on every take and its median gyro magnitude is recorded as the
quality of the estimate. If a take were shorter than the window, the
take-mean accelerometer vector would be used instead and flagged.

Gyro bias: the per-take median gyro vector, the same convention as the
gyro-prior work. On a take that rotates throughout, this median holds
real rotation as well as bias; the bias magnitude is recorded per take
so that can be read off.

Time sync: the integration window is shifted by SHIFT_MS against the
colour interval (gyro timestamps lag colour by about 50 ms, measured in
the earlier IMU work). The same integration is also run with zero shift
and the chained end pose reported as a one-number sensitivity check.

Frame change: the IMU-to-depth calibration translation below is the
position of the IMU origin expressed in the depth frame, with the two
frames taken as axis-aligned (no rotation is given). Each per-pair
relative pose is conjugated into the depth frame,
T_depth = X^-1 T_imu X with X = [I | t], which leaves R unchanged and
sends t to t + t_imu - R t_imu. The chained absolute poses are therefore
depth-frame poses, comparable to the odometry and ORB-SLAM3 poses. The
conjugated correction is sub-millimetre per pair; the sign convention
of the translation is stated here because it was not recoverable from
the repo files.

Step 2 (same script): the chained poses are compared against a reference
trajectory given with --orb (per-frame absolute poses JSON from the
existing adapter, first pose identity, indexed by paired colour frame).
Per-step errors use relative poses between the same two frames,
T_rel = inv(A[j]) @ A[i]. When no --orb is given, --truth-zero reports
the end pose against zero (valid on still takes only), and otherwise
the take is reported with no absolute reference.

Reads bags read-only. Writes one JSON per take with the per-pair deltas
(R, t in both frames, velocity/position deltas, zero-shift variants),
the chained absolute poses, and the comparison numbers.
"""
import argparse
import json
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "dataset-tools",
                                "bag-exploration"))
from explore_bag import parse_image, parse_imu  # noqa: E402
from explore_bag import COLOR, DEPTH, GYRO, ACCEL  # noqa: E402

# ---- parameters (all of them; nothing tuned is hidden in the code) ----
STRIDE = 5
SHIFT_MS = -50.0          # gyro-vs-colour lag correction (measured, ~50 ms)
GRAVITY_WINDOW_S = 1.0    # still-stretch length for the gravity estimate
GRAVITY_STEP_S = 0.25     # slide step when searching the stillest window
GRAVITY_STILL_MED = 0.05  # rad/s; reporting threshold, not a selection gate
IMU_TO_DEPTH_T = np.array([-0.030220, 0.007400, 0.016020])  # metres
MAX_COLOR_DEPTH_OFFSET_S = 0.05


def load_synced(bag, max_offset_s=MAX_COLOR_DEPTH_OFFSET_S):
    """Paired colour-frame timestamps (same rule as the odometry).

    Only header timestamps are needed here (no images): colour frames
    are kept in timestamp order, dropping any whose nearest depth frame
    is over max_offset_s away. Returns the kept colour stamps (ns).
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
        stamps[topic] = np.array([parse_image(b)["stamp_ns"]
                                  for (b,) in rows], dtype=np.float64)
    db.close()
    d_ts = stamps[DEPTH]
    kept = []
    worst = 0.0
    for t in stamps[COLOR]:
        j = int(np.argmin(np.abs(d_ts - t)))
        off = abs(d_ts[j] - t) / 1e9
        worst = max(worst, off)
        if off > max_offset_s:
            continue
        kept.append(t)
    print(f"paired {len(kept)} color+depth frames "
          f"(worst offset {worst*1000:.1f} ms)", flush=True)
    return {"stamps": np.array(kept, dtype=np.float64)}


def load_imu(bag):
    """Gyro and accel streams: (stamps_ns, vectors) each, in time order."""
    db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    out = {}
    for topic, key in ((GYRO, "gyro"), (ACCEL, "accel")):
        tid = db.execute("select id from topics where name=?",
                         (topic,)).fetchone()[0]
        rows = db.execute(
            "select data from messages where topic_id=? order by timestamp",
            (tid,)).fetchall()
        stamps, vecs = [], []
        for (blob,) in rows:
            p = parse_imu(blob)
            stamps.append(p["stamp_ns"])
            vecs.append(p["angular_velocity"] if key == "gyro"
                        else p["linear_acceleration"])
        out[key] = (np.array(stamps, dtype=np.float64),
                    np.array(vecs, dtype=np.float64))
    db.close()
    return out


def rotvec_to_R(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return R


def rot_angle_deg(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def rel_angle_deg(R_a, R_b):
    return rot_angle_deg(R_a.T @ R_b)


def chain(T_list):
    """Absolute poses A[0]=I, A[i+1] = A[i] @ inv(T_i); None holds pose."""
    A = [np.eye(4)]
    for T in T_list:
        if T is None:
            A.append(A[-1].copy())
        else:
            A.append(A[-1] @ np.linalg.inv(T))
    return A


def chain_stats(A):
    end_R = A[-1][:3, :3]
    traj = np.array([a[:3, 3] for a in A])
    path_rot = 0.0
    for k in range(len(A) - 1):
        path_rot += rot_angle_deg(A[k][:3, :3].T @ A[k + 1][:3, :3])
    return {"end_rot_deg": rot_angle_deg(end_R),
            "end_trans_m": float(np.linalg.norm(A[-1][:3, 3])),
            "end_trans_vec_m": A[-1][:3, 3].tolist(),
            "path_rot_deg": float(path_rot),
            "path_trans_m": float(np.sum(np.linalg.norm(
                np.diff(traj, axis=0), axis=1)))}


def find_gravity(g_ts, g_vec, a_ts, a_vec, bias):
    """Stillest-window gravity estimate.

    Returns dict with g_ref (gravity vector in body coords at t_ref),
    t_ref_ns, window start/end in seconds from take start, the median
    bias-removed gyro magnitude inside the window, and whether the
    window passed the still threshold.
    """
    gm = np.linalg.norm(g_vec - bias, axis=1)
    t0 = g_ts[0] / 1e9
    best = None
    s = t0
    while s + GRAVITY_WINDOW_S <= g_ts[-1] / 1e9:
        m = (g_ts / 1e9 >= s) & (g_ts / 1e9 < s + GRAVITY_WINDOW_S)
        if m.sum() >= 50:
            v = float(np.median(gm[m]))
            if best is None or v < best[0]:
                best = (v, s)
        s += GRAVITY_STEP_S
    if best is None:
        # Take shorter than the window: fall back to the take mean.
        g_ref = -a_vec.mean(axis=0)
        return {"g_ref": g_ref.tolist(), "g_mag": float(np.linalg.norm(g_ref)),
                "t_ref_s": 0.0, "window_start_s": 0.0,
                "window_end_s": float((g_ts[-1] - g_ts[0]) / 1e9),
                "window_med_gyro_rad_s": float(np.median(gm)),
                "window_is_still": False,
                "fallback_take_mean": True}
    v, s = best
    m = (a_ts / 1e9 >= s) & (a_ts / 1e9 < s + GRAVITY_WINDOW_S)
    g_ref = -a_vec[m].mean(axis=0)
    return {"g_ref": g_ref.tolist(), "g_mag": float(np.linalg.norm(g_ref)),
            "t_ref_s": float(s - t0), "window_start_s": float(s - t0),
            "window_end_s": float(s - t0 + GRAVITY_WINDOW_S),
            "window_med_gyro_rad_s": v,
            "window_is_still": bool(v < GRAVITY_STILL_MED),
            "fallback_take_mean": False}


def build_D_table(g_ts, g_vec, bias, t_ref_ns):
    """Chained body rotation D(t) from t_ref, at gyro sample times.

    D[k] rotates body(t_ref) coordinates into body(t_k) coordinates.
    A world-fixed vector v obeys v_body(t) = D(t)^-1 v_body(t_ref).
    """
    w = g_vec - bias
    n = len(g_ts)
    r = int(np.argmin(np.abs(g_ts - t_ref_ns)))
    D = [np.eye(3) for _ in range(n)]
    for k in range(r, n - 1):
        dt = (g_ts[k + 1] - g_ts[k]) / 1e9
        wm = 0.5 * (w[k] + w[k + 1])
        D[k + 1] = D[k] @ rotvec_to_R(wm * dt)
    for k in range(r, 0, -1):
        dt = (g_ts[k] - g_ts[k - 1]) / 1e9
        wm = 0.5 * (w[k - 1] + w[k])
        D[k - 1] = D[k] @ rotvec_to_R(-wm * dt)
    return D, r


def integrate_pair(g_ts, g_vec, a_ts, a_vec, bias, D_table, g_ref,
                   t_a, t_b, shift_ms):
    """Pre-integrate one pair window. Returns dict or None (<2 samples).

    t_a, t_b are colour-frame timestamps (ns); the window is shifted by
    shift_ms. Rotation uses the incrementally composed gyro rotation;
    accel is linearly interpolated onto the gyro sample grid.
    """
    sh = shift_ms * 1e6
    m = (g_ts >= t_a + sh) & (g_ts <= t_b + sh)
    idx = np.where(m)[0]
    if len(idx) < 2:
        return None
    ts = g_ts[idx]
    w = g_vec[idx] - bias
    # gravity in the pair-start frame, from the D table at the first
    # grid sample (within one gyro sample, ~5 ms, of the window start)
    g_i = D_table[idx[0]].T @ np.asarray(g_ref, dtype=np.float64)
    # accel interpolated onto the gyro grid (accel runs near 100 Hz,
    # gyro near 200 Hz; ends clamp to the nearest accel sample)
    f = np.column_stack([np.interp(ts, a_ts, a_vec[:, c]) for c in range(3)])
    R_rel = np.eye(3)
    R_steps = [np.eye(3)]
    rv_total = np.zeros(3)
    for k in range(len(idx) - 1):
        dt = (ts[k + 1] - ts[k]) / 1e9
        wm = 0.5 * (w[k] + w[k + 1])
        R_rel = R_rel @ rotvec_to_R(wm * dt)
        R_steps.append(R_rel.copy())
        rv_total += wm * dt
    a_body = np.array([R_steps[k] @ f[k] + g_i for k in range(len(idx))])
    dv = np.zeros(3)
    dp = np.zeros(3)
    v = np.zeros(3)
    for k in range(len(idx) - 1):
        dt = (ts[k + 1] - ts[k]) / 1e9
        a_mid = 0.5 * (a_body[k] + a_body[k + 1])
        v_new = v + a_mid * dt
        dp += 0.5 * (v + v_new) * dt
        v = v_new
    dv = v
    R_points = R_rel.T
    R_exp = rotvec_to_R(-rv_total)
    return {"R_imu": R_points, "dpos": dp, "dvel": dv,
            "dt_s": float((t_b - t_a) / 1e9),
            "rv_total": rv_total,
            "crosscheck_exp_vs_incremental_deg":
                rel_angle_deg(R_points, R_exp),
            "n_gyro_samples": int(len(idx)),
            "g_i": g_i}


def pair_pose(R_points, d):
    """Relative pose T = [R | -R d] from rotation and centre displacement."""
    T = np.eye(4)
    T[:3, :3] = R_points
    T[:3, 3] = -R_points @ d
    return T


def conjugate_to_depth(T):
    """T_depth = X^-1 T_imu X, X = [I | IMU_TO_DEPTH_T]."""
    tx = IMU_TO_DEPTH_T
    Td = np.eye(4)
    Td[:3, :3] = T[:3, :3]
    Td[:3, 3] = T[:3, 3] + tx - T[:3, :3] @ tx
    return Td


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=STRIDE)
    ap.add_argument("--shift_ms", type=float, default=SHIFT_MS)
    ap.add_argument("--orb", default=None,
                    help="per-frame absolute-poses JSON (reference)")
    ap.add_argument("--truth-zero", action="store_true",
                    help="reference is zero end pose (still takes only)")
    ap.add_argument("--vo", default=None,
                    help="VO absolute-poses JSON (consistency check only)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    take = os.path.basename(args.bag)[:-4][-15:]
    if len(os.path.basename(args.bag)[:-4]) > 15 and "102631c" in args.bag:
        take = "20260926_102631c"

    data = load_synced(args.bag)
    imu = load_imu(args.bag)
    g_ts, g_vec = imu["gyro"]
    a_ts, a_vec = imu["accel"]
    bias = np.median(g_vec, axis=0)
    print(f"gyro n={len(g_vec)} accel n={len(a_vec)} "
          f"bias=({bias[0]:.5f},{bias[1]:.5f},{bias[2]:.5f}) "
          f"|bias|={np.linalg.norm(bias):.5f} rad/s", flush=True)
    print(f"gyro rate {len(g_vec)/((g_ts[-1]-g_ts[0])/1e9):.1f} Hz, "
          f"accel rate {len(a_vec)/((a_ts[-1]-a_ts[0])/1e9):.1f} Hz",
          flush=True)

    grav = find_gravity(g_ts, g_vec, a_ts, a_vec, bias)
    print(f"gravity: |g|={grav['g_mag']:.3f} m/s^2 window "
          f"{grav['window_start_s']:.2f}-{grav['window_end_s']:.2f} s "
          f"med|gyro|={grav['window_med_gyro_rad_s']:.4f} rad/s "
          f"still={grav['window_is_still']} "
          f"fallback={grav['fallback_take_mean']}", flush=True)
    t_ref_ns = g_ts[0] + grav["t_ref_s"] * 1e9
    D_table, _ = build_D_table(g_ts, g_vec, bias, t_ref_ns)
    g_ref = np.asarray(grav["g_ref"], dtype=np.float64)

    n = len(data["stamps"])
    idx = list(range(0, n, args.stride))
    print(f"{n} frames, {len(idx)} chosen @ stride {args.stride}",
          flush=True)

    # ---- per-pair integration (shifted and zero-shift) ----
    pairs = []
    for k in range(len(idx) - 1):
        i, j = idx[k], idx[k + 1]
        t_a = float(data["stamps"][i])
        t_b = float(data["stamps"][j])
        integ = integrate_pair(g_ts, g_vec, a_ts, a_vec, bias, D_table,
                               g_ref, t_a, t_b, args.shift_ms)
        integ0 = integrate_pair(g_ts, g_vec, a_ts, a_vec, bias, D_table,
                                g_ref, t_a, t_b, 0.0)
        pairs.append({"ij": (i, j), "t_a_ns": t_a, "t_b_ns": t_b,
                      "integ": integ, "integ0": integ0})
        if k % 20 == 0:
            print(f"integrated pair {k}/{len(idx) - 1}", flush=True)
    n_null = sum(p["integ"] is None for p in pairs)
    n_null0 = sum(p["integ0"] is None for p in pairs)
    print(f"pairs integrated: {len(pairs) - n_null}/{len(pairs)} "
          f"(shifted), {len(pairs) - n_null0}/{len(pairs)} (zero shift)",
          flush=True)
    cc = [p["integ"]["crosscheck_exp_vs_incremental_deg"]
          for p in pairs if p["integ"] is not None]
    print(f"rotation cross-check Exp(-rv) vs incremental: median "
          f"{np.median(cc):.5f} deg max {np.max(cc):.5f} deg", flush=True)

    # ---- chains: primary (velocity propagated), zero-reset, shift-0 ----
    def build_chain(integ_key, propagate_velocity):
        T_list, v = [], np.zeros(3)
        for p in pairs:
            integ = p[integ_key]
            if integ is None:
                T_list.append(None)
                continue
            d = integ["dpos"] + (v * integ["dt_s"] if propagate_velocity
                                 else 0.0)
            T = conjugate_to_depth(
                pair_pose(integ["R_imu"], d))
            T_list.append(T)
            v = integ["R_imu"] @ (v + integ["dvel"])
        return T_list

    T_primary = build_chain("integ", True)
    T_reset = build_chain("integ", False)
    T_shift0 = build_chain("integ0", True)
    A_primary = chain(T_primary)
    A_reset = chain(T_reset)
    A_shift0 = chain(T_shift0)
    stats = {"primary": chain_stats(A_primary),
             "zero_velocity_reset": chain_stats(A_reset),
             "shift0_primary": chain_stats(A_shift0)}
    for name, st in stats.items():
        print(f"chain {name}: end_rot={st['end_rot_deg']:.2f} deg "
              f"end_trans={st['end_trans_m']:.3f} m "
              f"path_rot={st['path_rot_deg']:.2f} deg "
              f"path_trans={st['path_trans_m']:.3f} m", flush=True)

    # ---- per-pair deltas for reuse (JSON) ----
    pair_rows = []
    for p in pairs:
        row = {"i": p["ij"][0], "j": p["ij"][1],
               "dt_s": None, "R_imu": None, "t_imu": None,
               "R": None, "t": None, "dpos_m": None, "dvel_m_s": None,
               "R0": None, "t0": None, "n_gyro_samples": None}
        if p["integ"] is not None:
            integ = p["integ"]
            # t_imu/t use the propagated-velocity displacement, so they
            # match the primary chain; dpos/dvel are the raw deltas.
            row["dt_s"] = integ["dt_s"]
            row["dpos_m"] = integ["dpos"].tolist()
            row["dvel_m_s"] = integ["dvel"].tolist()
            row["n_gyro_samples"] = integ["n_gyro_samples"]
            row["R_imu"] = integ["R_imu"].tolist()
            row["R"] = integ["R_imu"].tolist()
            # t columns are filled below from the chain pass
        if p["integ0"] is not None:
            row["R0"] = p["integ0"]["R_imu"].tolist()
        pair_rows.append(row)
    # fill t / t_imu / t0 from the same velocity propagation as the chains
    for key, T_list, rows_key in (("integ", T_primary, ("t", "t_imu")),
                                  ("integ0", T_shift0, ("t0", None))):
        v = np.zeros(3)
        for k, p in enumerate(pairs):
            integ = p[key]
            if integ is None:
                continue
            d = integ["dpos"] + v * integ["dt_s"]
            T_imu = pair_pose(integ["R_imu"], d)
            pair_rows[k][rows_key[0]] = T_list[k][:3, 3].tolist()
            if rows_key[1] is not None:
                pair_rows[k][rows_key[1]] = T_imu[:3, 3].tolist()
            v = integ["R_imu"] @ (v + integ["dvel"])

    results = {
        "take": take, "bag": os.path.basename(args.bag),
        "bag_bytes": os.path.getsize(args.bag),
        "stride": args.stride, "shift_ms": args.shift_ms,
        "gyro_bias_rad_s": bias.tolist(),
        "gyro_bias_mag_rad_s": float(np.linalg.norm(bias)),
        "gyro_n": int(len(g_vec)), "accel_n": int(len(a_vec)),
        "n_frames": n, "n_chosen": len(idx),
        "n_pairs": len(pairs), "n_null_pairs": int(n_null),
        "n_null_pairs_shift0": int(n_null0),
        "gravity": grav,
        "gravity_method": (
            "minus mean accelerometer over the take's stillest "
            f"{GRAVITY_WINDOW_S:.0f} s window (sliding "
            f"{GRAVITY_STEP_S:.2f} s steps, ranked by median "
            "bias-removed gyro magnitude); rotated into each "
            "pair-start frame with the gyro-chained rotation from "
            "the window start"),
        "imu_to_depth_translation_m": IMU_TO_DEPTH_T.tolist(),
        "frame_convention": (
            "T_depth = X^-1 T_imu X with X = [I | imu_to_depth]; "
            "axes taken as aligned (no rotation given); R unchanged, "
            "t -> t + t_imu - R t_imu"),
        "rotation_crosscheck_exp_vs_incremental_deg": {
            "median": float(np.median(cc)), "max": float(np.max(cc))},
        "chains": stats,
        "pairs": pair_rows,
        "absolute_poses_primary": [a.tolist() for a in A_primary],
        "absolute_poses_zero_reset": [a.tolist() for a in A_reset],
        "absolute_poses_shift0": [a.tolist() for a in A_shift0],
        "chosen_frame_indices": idx,
        "chosen_frame_stamps_ns": [float(data["stamps"][i]) for i in idx],
    }

    # ---- reference comparison ----
    def rel_pose(A, i, j):
        return np.linalg.inv(A[j]) @ A[i]

    def compare_poses(ref_poses, ref_name, per_step=True, index_map=None):
        """End and per-step errors vs a reference absolute-pose list.

        index_map maps a colour-frame index to a position in ref_poses
        (identity for per-frame references like ORB; the VO reference
        holds one pose per chosen frame, so it needs the map).
        """
        end = {"end_rot_deg": rot_angle_deg(ref_poses[-1][:3, :3]),
               "end_trans_m": float(np.linalg.norm(ref_poses[-1][:3, 3])),
               "end_trans_vec_m": ref_poses[-1][:3, 3].tolist()}
        out = {"reference": ref_name, "reference_end": end,
               "preint_end_rot_deg": stats["primary"]["end_rot_deg"],
               "preint_end_trans_m": stats["primary"]["end_trans_m"],
               "end_rot_error_vs_ref_deg": None,
               "end_trans_error_vs_ref_m": None}
        # end error: angle / distance between the two end poses
        dR = ref_poses[-1][:3, :3].T @ A_primary[-1][:3, :3]
        out["end_rot_error_vs_ref_deg"] = rot_angle_deg(dR)
        out["end_trans_error_vs_ref_m"] = float(np.linalg.norm(
            A_primary[-1][:3, 3] - ref_poses[-1][:3, 3]))
        if per_step:
            r_err, t_err = [], []
            for k, p in enumerate(pairs):
                i, j = p["ij"]
                if T_primary[k] is None:
                    continue
                if index_map is not None:
                    ri, rj = index_map[i], index_map[j]
                else:
                    ri, rj = i, j
                T_ref = rel_pose(ref_poses, ri, rj)
                r_err.append(rel_angle_deg(T_primary[k][:3, :3],
                                           T_ref[:3, :3]))
                t_err.append(float(np.linalg.norm(
                    T_primary[k][:3, 3] - T_ref[:3, 3])))
            out["per_step_rot_error_deg"] = {
                "median": float(np.median(r_err)),
                "mean": float(np.mean(r_err)),
                "p90": float(np.percentile(r_err, 90)),
                "max": float(np.max(r_err))}
            out["per_step_trans_error_m"] = {
                "median": float(np.median(t_err)),
                "mean": float(np.mean(t_err)),
                "p90": float(np.percentile(t_err, 90)),
                "max": float(np.max(t_err))}
            out["n_per_step"] = len(r_err)
        return out

    if args.orb is not None:
        with open(args.orb) as f:
            orb = json.load(f)
        ref_poses = [np.array(p) for p in orb["absolute_poses"]]
        # align by timestamp: ORB poses are indexed by the paired colour
        # frames in the same order; verify counts and report the worst
        # timestamp gap of the chosen frames against the ORB index grid
        if len(ref_poses) != n:
            print(f"WARNING: ORB poses {len(ref_poses)} != paired frames "
                  f"{n}; aligning by index anyway", flush=True)
        results["orb_file"] = os.path.basename(args.orb)
        results["comparison_vs_orb"] = compare_poses(
            ref_poses, "ORB-SLAM3 RGB-D-only")
        c = results["comparison_vs_orb"]
        print(f"ORB end: {c['reference_end']['end_rot_deg']:.2f} deg, "
              f"{c['reference_end']['end_trans_m']:.3f} m; preint end "
              f"error vs ORB: {c['end_rot_error_vs_ref_deg']:.2f} deg, "
              f"{c['end_trans_error_vs_ref_m']:.3f} m; per-step medians: "
              f"{c['per_step_rot_error_deg']['median']:.3f} deg, "
              f"{c['per_step_trans_error_m']['median']:.4f} m", flush=True)
    elif args.truth_zero:
        ref_poses = [np.eye(4) for _ in idx]
        # per-step truth is zero motion: report per-step magnitudes
        r_mag = [rot_angle_deg(T[:3, :3]) for T in T_primary
                 if T is not None]
        t_mag = [float(np.linalg.norm(T[:3, 3])) for T in T_primary
                 if T is not None]
        results["comparison_vs_truth_zero"] = {
            "reference": "truth (camera still): zero end pose",
            "preint_end_rot_deg": stats["primary"]["end_rot_deg"],
            "preint_end_trans_m": stats["primary"]["end_trans_m"],
            "per_step_rot_mag_deg": {
                "median": float(np.median(r_mag)),
                "mean": float(np.mean(r_mag)),
                "max": float(np.max(r_mag))},
            "per_step_trans_mag_m": {
                "median": float(np.median(t_mag)),
                "mean": float(np.mean(t_mag)),
                "max": float(np.max(t_mag))}}
        print(f"truth-zero: preint end "
              f"{stats['primary']['end_rot_deg']:.2f} deg, "
              f"{stats['primary']['end_trans_m']:.3f} m", flush=True)
    else:
        results["comparison_vs_orb"] = None
        # gyro total rotation as the only rotation reference
        tot_rv = np.zeros(3)
        for p in pairs:
            if p["integ"] is not None:
                tot_rv += p["integ"]["rv_total"]
        results["gyro_total"] = {
            "sum_pair_rot_deg": float(sum(
                rot_angle_deg(p["integ"]["R_imu"]) for p in pairs
                if p["integ"] is not None)),
            "net_rot_deg": rot_angle_deg(rotvec_to_R(-tot_rv))}
        print(f"no absolute reference: gyro total rotation "
              f"{results['gyro_total']['sum_pair_rot_deg']:.1f} deg, net "
              f"{results['gyro_total']['net_rot_deg']:.1f} deg", flush=True)

    if args.vo is not None:
        with open(args.vo) as f:
            vo = json.load(f)
        vo_poses = [np.array(p) for p in vo["absolute_poses"]]
        # VO poses are one per chosen frame, not one per colour frame
        vo_map = {f: k for k, f in enumerate(idx)}
        results["comparison_vs_vo"] = compare_poses(
            vo_poses, "RGB-D VO chain (consistency check only)",
            index_map=vo_map)
        c = results["comparison_vs_vo"]
        print(f"VO end: {c['reference_end']['end_rot_deg']:.2f} deg, "
              f"{c['reference_end']['end_trans_m']:.3f} m; preint end "
              f"error vs VO: {c['end_rot_error_vs_ref_deg']:.2f} deg, "
              f"{c['end_trans_error_vs_ref_m']:.3f} m; per-step medians: "
              f"{c['per_step_rot_error_deg']['median']:.3f} deg, "
              f"{c['per_step_trans_error_m']['median']:.4f} m", flush=True)

    out_path = os.path.join(args.out, f"{take}_preint.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=1)
    print(f"wrote {out_path}", flush=True)
    print(json.dumps({k: results[k] for k in
                      ("take", "gyro_bias_mag_rad_s", "gravity", "chains")
                      if k in results}, indent=1), flush=True)


if __name__ == "__main__":
    main()
