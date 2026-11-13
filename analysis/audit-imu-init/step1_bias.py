#!/usr/bin/env python3
"""Step 1: gyro bias from visual relative rotations (GrassVIO imu-init).

For each stride-5 pair, the ORB-SLAM3 relative rotation is compared with
the gyro-integrated relative rotation as a function of a constant gyro
bias b. A 3-parameter Levenberg-Marquardt fit finds the bias that makes
the integrated gyro rotations agree with the camera's measured relative
rotations. Rotation only; the IMU-to-depth translation is not applied
(R is unchanged by it) and the visual poses stay at the camera origin.

Two initializations are run (b = 0 and b = per-take median gyro) and both
are reported. Residual rotation errors are also reported at b = 0, at
the fitted b, and at the median bias, so the fitted bias can be judged
against the median-bias convention of the preint work.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (  # noqa: E402
    integrate_gyro_R,
    load_take,
    log_R,
    lm,
    pair_samples,
    rot_angle_deg,
)


def pair_rotations(take_data):
    """Per stride-5 pair: visual relative rotation and gyro samples."""
    stamps = take_data["stamps"]
    poses = take_data["orb_poses"]
    idx = take_data["chosen"]
    out = []
    for k in range(len(idx) - 1):
        i, j = idx[k], idx[k + 1]
        R_vis = poses[j][:3, :3].T @ poses[i][:3, :3]
        s = pair_samples(take_data, float(stamps[i]), float(stamps[j]))
        if s is None:
            continue
        out.append({"i": i, "j": j, "R_vis": R_vis,
                    "ts": s[0], "w": s[1]})
    return out


def residuals(b, pairs):
    r = []
    for p in pairs:
        R_gyro, _ = integrate_gyro_R(p["ts"], p["w"], b)
        r.append(log_R(p["R_vis"].T @ R_gyro))
    return np.concatenate(r)


def angle_stats(b, pairs):
    angs = []
    for p in pairs:
        R_gyro, _ = integrate_gyro_R(p["ts"], p["w"], b)
        angs.append(rot_angle_deg(p["R_vis"].T @ R_gyro))
    angs = np.array(angs)
    return {"median_deg": float(np.median(angs)),
            "rms_deg": float(np.sqrt(np.mean(angs ** 2))),
            "mean_deg": float(np.mean(angs)),
            "max_deg": float(np.max(angs)),
            "n": int(len(angs))}


def path_rotation_deg(b, pairs):
    tot = 0.0
    for p in pairs:
        R_gyro, _ = integrate_gyro_R(p["ts"], p["w"], b)
        tot += rot_angle_deg(R_gyro)
    return float(tot)


def orb_path_rotation_deg(pairs):
    return float(sum(rot_angle_deg(p["R_vis"]) for p in pairs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--take", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    td = load_take(args.take)
    pairs = pair_rotations(td)
    print(f"{args.take}: {len(pairs)} stride-5 pairs with gyro samples",
          flush=True)
    median_bias = np.array(td["preint"]["gyro_bias_rad_s"])
    orb_total = orb_path_rotation_deg(pairs)
    print(f"ORB total path rotation: {orb_total:.2f} deg; "
          f"median gyro bias {median_bias} "
          f"|.|={np.linalg.norm(median_bias):.5f} rad/s", flush=True)

    fits = {}
    for name, b0 in (("from_zero", np.zeros(3)),
                     ("from_median", median_bias.copy())):
        x, cost, n_iter, n_fun, _ = lm(
            lambda b: residuals(b, pairs), b0, max_iter=50,
            fd_steps=np.full(3, 1e-4), verbose=False)
        fits[name] = {
            "bias_rad_s": x.tolist(),
            "bias_mag_rad_s": float(np.linalg.norm(x)),
            "cost": float(cost),
            "n_iter": int(n_iter),
            "n_fun": int(n_fun),
            "residual_deg": angle_stats(x, pairs),
            "path_rotation_deg": path_rotation_deg(x, pairs),
        }
        print(f"fit {name}: b={np.round(x, 5)} |b|={np.linalg.norm(x):.5f} "
              f"rad/s, residual median "
              f"{fits[name]['residual_deg']['median_deg']:.4f} deg, "
              f"path rot {fits[name]['path_rotation_deg']:.2f} deg",
              flush=True)

    result = {
        "take": args.take,
        "n_pairs": len(pairs),
        "median_bias_rad_s": median_bias.tolist(),
        "median_bias_mag_rad_s": float(np.linalg.norm(median_bias)),
        "orb_total_path_rotation_deg": orb_total,
        "residual_at_zero_bias_deg": angle_stats(np.zeros(3), pairs),
        "residual_at_median_bias_deg": angle_stats(median_bias, pairs),
        "path_rotation_at_zero_bias_deg": path_rotation_deg(np.zeros(3),
                                                            pairs),
        "path_rotation_at_median_bias_deg": path_rotation_deg(median_bias,
                                                              pairs),
        "fits": fits,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=1)
    print(f"wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
