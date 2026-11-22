#!/usr/bin/env python3
"""Post-hoc (added after seeing results): horizons measured from motion onset.

Integration is unchanged: it still starts at the end of the verified rest
window with zero velocity and uses no visual information afterwards. Only the
scoring times move: onset = first visual frame >= 1 cm from the origin.
Gates are the same G1-G3 numbers, evaluated at onset + 0.3 s and + 1.0 s,
plus descriptive + 2.0 s.
"""
import json
import sys
from pathlib import Path

import numpy as np

import item5_analysis as ia

ONSET_M = 0.01
POST_H = (0.3, 1.0, 2.0)


def series(npz, shift_ms, t_extra=6.0):
    d = dict(np.load(npz))
    s = shift_ms / 1000.
    rest = ia.rest_window(d, s)
    if not rest["verified"]:
        return d, rest, None
    rows, poses = d["pair_rows"], d["pair_pose_rvec_tvec"]
    ct = (d["color_t"] - d["color_t"][0]) / 1e9
    chain = ia.visual_chain(rows, poses, rest["last_pair"] + 1)
    t0 = rest["t_end"]
    imu = ia.integrate(d, s, d["color_from_gyro_R"], d["color_from_accel_R"],
                       d["color_from_accel_t"], rest["t_start"], t0, t0 + t_extra + 3)
    tv = np.array([ct[f] - t0 for f, _, _ in chain])
    pv = np.array([p for _, p, _ in chain])
    Rv = np.array([R for _, _, R in chain])
    ti = imu["t"] - t0
    keep = tv <= t_extra + 3
    tv, pv, Rv = tv[keep], pv[keep], Rv[keep]
    idx = np.array([int(np.argmin(np.abs(ti - x))) for x in tv])
    pi, Ri = imu["pos"][idx], imu["R"][idx]
    return d, rest, dict(t=tv, vis=pv, imu=pi, vis_rotation=Rv, imu_rotation=Ri)


def onset_scores(ser):
    disp = np.linalg.norm(ser["vis"], axis=1)
    moving = np.nonzero(disp >= ONSET_M)[0]
    if not len(moving):
        return dict(onset_s=None, horizons={})
    k0 = moving[0]
    t_on = ser["t"][k0]
    # displacement is measured from the onset frame for both channels
    out = dict(onset_s=float(t_on), horizons={})
    for h in POST_H:
        if ser["t"][-1] < t_on + h - 0.02:
            out["horizons"][str(h)] = dict(status="visual chain ended")
            continue
        k = int(np.argmin(np.abs(ser["t"] - (t_on + h))))
        dv = ser["vis"][k] - ser["vis"][k0]
        di = ser["imu"][k] - ser["imu"][k0]
        visual_rotation = ser["vis_rotation"][k0].T @ ser["vis_rotation"][k]
        imu_rotation = ser["imu_rotation"][k0].T @ ser["imu_rotation"][k]
        err = float(np.linalg.norm(di - dv))
        vd = float(np.linalg.norm(dv))
        rec = dict(seconds_after_origin=float(ser["t"][k]), visual_m=float(vd),
                   imu_m=float(np.linalg.norm(di)), error_m=err,
                   relative_error=err / vd if vd > 0 else None,
                   direction_diff_deg=float(np.degrees(np.arccos(np.clip(
                       dv @ di / (np.linalg.norm(dv) * np.linalg.norm(di) + 1e-12), -1, 1)))),
                   rotation_diff_deg=ia.angle_deg(visual_rotation.T @ imu_rotation),
                   status="scored" if vd >= ia.MIN_MOTION_M else "camera moved < 1 cm")
        out["horizons"][str(h)] = rec
    g = {}
    for name, h, test in (("G1@0.3", "0.3", lambda x: x["error_m"] < x["visual_m"]),
                          ("G1@1.0", "1.0", lambda x: x["error_m"] < x["visual_m"]),
                          ("G2@1.0", "1.0", lambda x: x["error_m"] <= ia.G2_FRACTION * x["visual_m"]),
                          ("G3@1.0", "1.0", lambda x: x["rotation_diff_deg"] <= ia.G3_DEG)):
        x = out["horizons"].get(h, {})
        g[name] = bool(test(x)) if x.get("status") == "scored" else None
    out["gates"] = g
    return out


if __name__ == "__main__":
    extract, timing, out = map(Path, sys.argv[1:4])
    res = {}
    for npz in sorted(extract.glob("*.npz")):
        sh = ia.load_shift_ms(timing, npz.stem)
        _, rest, ser = series(npz, sh)
        res[npz.stem] = dict(rest_verified=rest["verified"],
                             posthoc=onset_scores(ser) if ser else None)
    out.write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=1))
