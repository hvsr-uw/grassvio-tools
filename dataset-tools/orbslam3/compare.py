#!/usr/bin/env python3
"""Compare an ORB-SLAM3 TUM trajectory against the project's RGB-D VO
poses (absolute_poses, 4x4 camera-to-world matrices, stride 5 over the
color frames) for one take.

Metrics:
  * coverage: fraction of color frames present in the ORB-SLAM3 output
  * end error: |t_last - t_first| and rotation angle between first and
    last pose, for ORB-SLAM3 and for the VO (loop-closure error on a
    take that returns to its start)
  * ATE: RMSE of translation after SE(3) (Umeyama, no scale) alignment
    of the ORB-SLAM3 poses onto the VO poses at matching frames

Usage: compare.py <take> <orb_tum.txt> <vo_poses.json> [--out prefix]
Writes <prefix>_overlay.png and prints a JSON summary.
"""

import argparse
import json
import sys

import numpy as np

sys.path.insert(0, ".")


def load_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = [float(x) for x in line.split()]
            rows.append(v)
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]  # t, xyz, qxqyqzw


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def umeyama(P, Q):
    """SE(3) (no scale) mapping P onto Q. P, Q: (N,3). Returns R, t."""
    pc, qc = P.mean(0), Q.mean(0)
    X, Y = P - pc, Q - qc
    H = X.T @ Y
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, qc - R @ pc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("take")
    ap.add_argument("orb_tum")
    ap.add_argument("vo_json")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    vo = json.load(open(args.vo_json))
    vo_poses = [np.array(p) for p in vo["absolute_poses"]]
    stride = vo.get("stride", 5)

    t_orb, xyz_orb, q_orb = load_tum(args.orb_tum)
    n_orb = len(t_orb)

    # VO pose k corresponds to color frame index k*stride. Color frame
    # timestamps come from the take's rgb.txt.
    rgb_ts = []
    with open("data/%s/rgb.txt" % args.take) as f:
        for line in f:
            rgb_ts.append(float(line.split()[0]))
    rgb_ts = np.array(rgb_ts)

    vo_t = np.array([rgb_ts[min(k * stride, len(rgb_ts) - 1)]
                     for k in range(len(vo_poses))])
    vo_xyz = np.array([p[:3, 3] for p in vo_poses])
    vo_R = np.array([p[:3, :3] for p in vo_poses])

    # Match ORB poses to VO frames by nearest timestamp.
    idx = []
    for tt in vo_t:
        j = int(np.argmin(np.abs(t_orb - tt)))
        if abs(t_orb[j] - tt) < 0.05:
            idx.append(j)
        else:
            idx.append(-1)
    ok = np.array([i >= 0 for i in idx])
    orb_xyz_m = np.array([xyz_orb[i] if i >= 0 else [np.nan] * 3
                          for i in idx])
    orb_R_m = np.array([quat_to_R(q_orb[i]) if i >= 0 else np.eye(3)
                        for i in idx])

    def end_error(xyz, R):
        d = np.linalg.norm(xyz[-1] - xyz[0])
        ang = rot_angle_deg(R[-1] @ R[0].T)
        return float(d), float(ang)

    orb_end = end_error(xyz_orb, np.array([quat_to_R(q) for q in q_orb]))
    vo_end = end_error(vo_xyz, vo_R)

    ate = None
    if ok.sum() >= 3:
        R, t = umeyama(orb_xyz_m[ok], vo_xyz[ok])
        aligned = (R @ orb_xyz_m[ok].T).T + t
        err = np.linalg.norm(aligned - vo_xyz[ok], axis=1)
        ate = {"rmse_m": float(np.sqrt(np.mean(err ** 2))),
               "mean_m": float(np.mean(err)),
               "max_m": float(np.max(err)),
               "n_matched": int(ok.sum())}

    out = {
        "take": args.take,
        "orb_frames": int(n_orb),
        "color_frames": int(len(rgb_ts)),
        "coverage": float(n_orb / len(rgb_ts)),
        "orb_end_error_m": orb_end[0],
        "orb_end_rot_deg": orb_end[1],
        "vo_end_error_m": vo_end[0],
        "vo_end_rot_deg": vo_end[1],
        "ate_vs_vo": ate,
        "orb_path_length_m": float(np.sum(np.linalg.norm(
            np.diff(xyz_orb, axis=0), axis=1))),
        "vo_path_length_m": float(np.sum(np.linalg.norm(
            np.diff(vo_xyz, axis=0), axis=1))),
    }
    print(json.dumps(out, indent=1))

    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 5))
        ax[0].plot(vo_xyz[:, 0], vo_xyz[:, 2], "o-", ms=3, label="VO")
        ax[0].plot(xyz_orb[:, 0], xyz_orb[:, 2], ".-", ms=2,
                   label="ORB-SLAM3")
        ax[0].set_title("%s: top view (x vs z)" % args.take)
        ax[0].set_xlabel("x (m)")
        ax[0].set_ylabel("z (m)")
        ax[0].legend()
        ax[0].axis("equal")
        ax[1].plot(vo_xyz[:, 0], vo_xyz[:, 1], "o-", ms=3, label="VO")
        ax[1].plot(xyz_orb[:, 0], xyz_orb[:, 1], ".-", ms=2,
                   label="ORB-SLAM3")
        ax[1].set_title("%s: side view (x vs y)" % args.take)
        ax[1].set_xlabel("x (m)")
        ax[1].set_ylabel("y (m)")
        ax[1].legend()
        ax[1].axis("equal")
        fig.tight_layout()
        fig.savefig(args.out)
        print("plot:", args.out)


if __name__ == "__main__":
    main()
