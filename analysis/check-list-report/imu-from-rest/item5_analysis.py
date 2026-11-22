#!/usr/bin/env python3
"""IMU vs RGB-D translation from a verified rest start (see PROTOCOL.md)."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

# Fixed thresholds (PROTOCOL.md). Do not tune after seeing results.
R1_FLOW_PX, R1_POS_M, R1_ROT_DEG = 0.3, 0.003, 0.2
R2_GYRO_P95 = 0.02
R3_ACC_STD = 0.05
MIN_REST_S = 0.5
HORIZONS = (0.1, 0.3, 1.0, 3.0)
MIN_MOTION_M = 0.01
G2_FRACTION, G3_DEG = 0.25, 1.0
LOCAL_G = 9.803


def exp_so3(w):
    R, _ = cv2.Rodrigues(np.asarray(w, float).reshape(3, 1))
    return R


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def visual_chain(rows, poses, start_pair):
    """Camera poses of frames after pair index start_pair, in the start frame.

    Returns list of (frame, position, R_world_from_cam); stops at a failed pair.
    """
    A = np.eye(4)
    out = [(int(rows[start_pair, 0]), np.zeros(3), np.eye(3))]
    for k in range(start_pair, len(rows)):
        if not np.isfinite(poses[k]).all():
            break
        T = np.eye(4)
        T[:3, :3] = exp_so3(poses[k][:3])
        T[:3, 3] = poses[k][3:]
        A = T @ A  # x_j = A x_start
        R, t = A[:3, :3], A[:3, 3]
        out.append((int(rows[k, 1]), -R.T @ t, R.T))
    return out


def rest_window(d, shift_s, from_end=False):
    rows, poses = d["pair_rows"], d["pair_pose_rvec_tvec"]
    ct = (d["color_t"] - d["color_t"][0]) / 1e9
    order = range(len(rows) - 1, -1, -1) if from_end else range(len(rows))
    A = np.eye(4)
    kept = []
    for k in order:
        flow = rows[k, 5]
        if not np.isfinite(poses[k]).all() or not (flow < R1_FLOW_PX):
            break
        T = np.eye(4)
        T[:3, :3] = exp_so3(poses[k][:3])
        T[:3, 3] = poses[k][3:]
        A = (T @ A) if not from_end else (np.linalg.inv(T) @ A)
        if np.linalg.norm(-A[:3, :3].T @ A[:3, 3]) > R1_POS_M or angle_deg(A[:3, :3]) > R1_ROT_DEG:
            break
        kept.append(k)
    if not kept:
        return dict(verified=False, visual_s=0.0, reason="first pair already moving")
    first, last = min(kept), max(kept)
    f0, f1 = int(rows[first, 0]), int(rows[last, 1])
    t0, t1 = ct[f0], ct[f1]
    gt = (d["gyro_t"] - d["color_t"][0]) / 1e9 - shift_s
    at = (d["accel_t"] - d["color_t"][0]) / 1e9 - shift_s
    gsel = (gt >= t0) & (gt <= t1)
    asel = (at >= t0) & (at <= t1)
    gyro_p95 = float(np.percentile(np.linalg.norm(d["gyro"][gsel], axis=1), 95)) if gsel.sum() > 5 else np.inf
    acc_std = float(np.std(np.linalg.norm(d["accel"][asel], axis=1))) if asel.sum() > 5 else np.inf
    dur = float(t1 - t0)
    ok = dur >= MIN_REST_S and gyro_p95 < R2_GYRO_P95 and acc_std < R3_ACC_STD
    reasons = []
    if dur < MIN_REST_S:
        reasons.append(f"visual rest only {dur:.2f} s")
    if not gyro_p95 < R2_GYRO_P95:
        reasons.append(f"gyro p95 {gyro_p95:.4f} rad/s")
    if not acc_std < R3_ACC_STD:
        reasons.append(f"accel std {acc_std:.4f} m/s2")
    return dict(verified=bool(ok), start_frame=f0, end_frame=f1, t_start=float(t0), t_end=float(t1),
                visual_s=dur, gyro_p95=gyro_p95, accel_norm_std=acc_std,
                first_pair=first, last_pair=last, reason="; ".join(reasons) or "all three checks pass")


def integrate(d, shift_s, R_cg, R_ca, lever_c, t_rest0, t_origin, t_stop):
    """IMU motion of the colour camera from t_origin, initialised on [t_rest0, t_origin]."""
    gt = (d["gyro_t"] - d["color_t"][0]) / 1e9 - shift_s
    at = (d["accel_t"] - d["color_t"][0]) / 1e9 - shift_s
    gyro = d["gyro"] @ R_cg.T
    acc = d["accel"] @ R_ca.T
    rest_g = (gt >= t_rest0) & (gt <= t_origin)
    rest_a = (at >= t_rest0) & (at <= t_origin)
    bias = gyro[rest_g].mean(axis=0)
    f0 = acc[rest_a].mean(axis=0)
    sel = (gt >= t_origin) & (gt <= t_stop)
    t = np.concatenate([[t_origin], gt[sel]])
    w = np.vstack([np.interp(t, gt, gyro[:, i]) for i in range(3)]).T - bias
    f = np.vstack([np.interp(t, at, acc[:, i]) for i in range(3)]).T
    R = np.eye(3)
    Rs, v, p = [R], np.zeros(3), np.zeros(3)
    ps = [p.copy()]
    a_prev = R @ f[0] - f0
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        R = R @ exp_so3(0.5 * (w[k - 1] + w[k]) * dt)
        a = R @ f[k] - f0
        v_new = v + 0.5 * (a_prev + a) * dt
        p = p + 0.5 * (v + v_new) * dt
        v, a_prev = v_new, a
        Rs.append(R)
        ps.append(p.copy())
    ps = np.array(ps)
    # body (IMU) origin -> camera origin: camera = imu_pos + R (-lever) - (-lever)
    cam = ps + np.array([(Rk - np.eye(3)) @ (-lever_c) for Rk in Rs])
    return dict(t=t, pos=cam, R=np.array(Rs), gyro_bias=bias, f0=f0, v_end=v)


def score(d, rest, shift_s, R_cg, R_ca, lever_c, rest_t0=None, label="verified"):
    rows, poses = d["pair_rows"], d["pair_pose_rvec_tvec"]
    ct = (d["color_t"] - d["color_t"][0]) / 1e9
    origin_pair = rest["last_pair"] + 1 if "last_pair" in rest else 0
    t_origin = rest["t_end"]
    t0 = rest["t_start"] if rest_t0 is None else rest_t0
    chain = visual_chain(rows, poses, origin_pair) if origin_pair < len(rows) else []
    imu = integrate(d, shift_s, R_cg, R_ca, lever_c, t0, t_origin, t_origin + max(HORIZONS) + 0.2)
    out = dict(label=label, origin_frame=int(rows[origin_pair, 0]) if origin_pair < len(rows) else None,
               t_origin=t_origin, f0_norm=float(np.linalg.norm(imu["f0"])),
               gyro_bias=imu["gyro_bias"].tolist(), horizons={})
    times = np.array([ct[f] - t_origin for f, _, _ in chain]) if chain else np.array([])
    for h in HORIZONS:
        if not len(times) or times[-1] < h - 0.02:
            out["horizons"][str(h)] = dict(status="visual chain ended before horizon")
            continue
        k = int(np.argmin(np.abs(times - h)))
        frame, pv, Rv = chain[k]
        th = ct[frame] - t_origin
        i = int(np.argmin(np.abs(imu["t"] - (t_origin + th))))
        pi, Ri = imu["pos"][i], imu["R"][i]
        disp = float(np.linalg.norm(pv))
        err = float(np.linalg.norm(pi - pv))
        rec = dict(frame=frame, seconds=float(th), visual_m=pv.tolist(), imu_m=pi.tolist(),
                   visual_disp_m=disp, imu_disp_m=float(np.linalg.norm(pi)), error_m=err,
                   rotation_diff_deg=angle_deg(Rv.T @ Ri))
        if disp >= 1e-9 and np.linalg.norm(pi) > 1e-9:
            rec["direction_diff_deg"] = float(np.degrees(np.arccos(np.clip(
                pv @ pi / (np.linalg.norm(pv) * np.linalg.norm(pi)), -1, 1))))
        rec["status"] = "scored" if disp >= MIN_MOTION_M else "camera moved < 1 cm"
        rec["relative_error"] = err / disp if disp > 0 else None
        out["horizons"][str(h)] = rec
    return out


def gates(results):
    verified = [r for r in results if r["rest"]["verified"]]
    table = []
    for r in verified:
        s = r["score"]["horizons"]
        g = {}
        for name, h, test in (("G1@0.3", "0.3", lambda x: x["error_m"] < x["visual_disp_m"]),
                              ("G1@1.0", "1.0", lambda x: x["error_m"] < x["visual_disp_m"]),
                              ("G2@1.0", "1.0", lambda x: x["error_m"] <= G2_FRACTION * x["visual_disp_m"]),
                              ("G3@1.0", "1.0", lambda x: x["rotation_diff_deg"] <= G3_DEG)):
            x = s.get(h, {})
            g[name] = (bool(test(x)) if x.get("status") == "scored" else None)
        table.append(dict(recording=r["recording"], **g))
    decided = [v for row in table for k, v in row.items() if k != "recording"]
    passed = (len(verified) >= 3 and decided and all(v is True for v in decided))
    return dict(rest_verified=len(verified), per_recording=table, item5_passes=bool(passed),
                unscored_gates=sum(v is None for v in decided))


def analyse(npz, shift_ms):
    d = dict(np.load(npz))
    shift_s = shift_ms / 1000.
    R_cg, R_ca = d["color_from_gyro_R"], d["color_from_accel_R"]
    lever_c = d["color_from_accel_t"]  # IMU origin expressed in the colour frame
    rest = rest_window(d, shift_s)
    end = rest_window(d, shift_s, from_end=True)
    res = dict(recording=Path(npz).stem, shift_ms=shift_ms, rest=rest, end_rest=end)
    if rest["verified"]:
        res["score"] = score(d, rest, shift_s, R_cg, R_ca, lever_c)
    else:
        ct = (d["color_t"] - d["color_t"][0]) / 1e9
        k = int(np.searchsorted(ct[d["pair_rows"][:, 1].astype(int)], 0.5))
        fake = dict(t_start=0.0, t_end=float(ct[int(d["pair_rows"][k, 1])]), last_pair=k)
        res["control_unverified_start"] = score(d, fake, shift_s, R_cg, R_ca, lever_c,
                                                label="unverified 0.5 s start")
    return res


def load_shift_ms(timing, recording):
    if timing.is_dir():
        data = json.loads((timing / recording / "gyro_timing.json").read_text())
        return data["selected_hypothesis"]["shift_ms"]
    return json.loads(timing.read_text())[recording]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--extract-dir", type=Path, required=True)
    ap.add_argument("--timing", "--timing-dir", type=Path, required=True,
                    help="Recording-to-shift JSON or directory of per-recording timing results")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    results = []
    for npz in sorted(a.extract_dir.glob("*.npz")):
        results.append(analyse(npz, load_shift_ms(a.timing, npz.stem)))
    summary = dict(protocol="PROTOCOL.md", results=results, gates=gates(results),
                   reference="dense RGB-D odometry, not ground truth")
    a.out.write_text(json.dumps(summary, indent=2, default=float))
    print(json.dumps(summary["gates"], indent=2))


if __name__ == "__main__":
    main()
