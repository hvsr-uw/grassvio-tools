"""
CS566 Stage IV (Motion) — Monocular Visual Odometry with Lucas-Kanade optical flow.

Implements the Stage IV approach from the project proposal:
  "we will try first with 2D Optical Flow (Lucas-Kanade Differential Methods)"

Pipeline (per frame pair):
  1. Shi-Tomasi corners -> pyramidal Lucas-Kanade tracking (calcOpticalFlowPyrLK)
     with forward-backward consistency check.
  2. Essential matrix (5-point, RANSAC) -> recoverPose -> R, t (unit norm).
  3. Scale recovery from the known camera height: triangulate inliers,
     fit the dominant ground plane, scale = height / plane distance.
     (Monocular VO is scale-ambiguous; the proposal's VIO section notes the
     same need for an extra metric source — here the fixed camera mount.)
  4. Pose accumulation into a global trajectory.

Usage:
  python vo_lk.py --video data/synth_frames.mp4 --gt data/synth_groundtruth.npz
  python vo_lk.py --video my_lawn.mp4 --fx 520 --fy 520 --cx 320 --cy 240 --height 1.0

Outputs (results/): trajectory.npz, metrics.json, trajectory_topdown.png,
                    trajectory_3d.png, flow_viz.png
"""
import argparse
import json
import os
import sys

import numpy as np
import cv2

# ---------------- feature detection / tracking ----------------

def detect_features(gray, max_corners=2500):
    pts = cv2.goodFeaturesToTrack(
        gray, maxCorners=max_corners, qualityLevel=0.01,
        minDistance=7, blockSize=7, useHarrisDetector=False)
    if pts is not None:
        crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)
        cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), crit)
    return pts


def track_features(gray1, gray2, pts1):
    """Pyramidal Lucas-Kanade with forward-backward consistency check."""
    if pts1 is None or len(pts1) == 0:
        return np.zeros((0, 1, 2)), np.zeros((0, 1, 2))
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    pts2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, pts1, None, **lk)
    pts1_back, st_b, _ = cv2.calcOpticalFlowPyrLK(gray2, gray1, pts2, None, **lk)
    fb = np.linalg.norm(pts1 - pts1_back, axis=2).ravel()
    good = (st.ravel() == 1) & (st_b.ravel() == 1) & (fb < 1.0) & (err.ravel() < 30)
    return pts1[good], pts2[good]


# ---------------- two-view geometry ----------------

def _pick_homography_solution(H, K, p1, p2, inl):
    """Decompose H, keep the solution whose plane normal points toward the
    camera (+z, nadir prior) and passes cheirality. Returns (R, t_over_d, n)."""
    ret, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    idx = np.arange(inl.sum())
    idx = idx[::max(1, len(idx) // 40)]
    q1 = np.ascontiguousarray(p1[inl].reshape(-1, 2)[idx].T)
    q2 = np.ascontiguousarray(p2[inl].reshape(-1, 2)[idx].T)
    best = None
    for i in range(ret):
        n = Ns[i].ravel()
        if n[2] < 0.9:                      # want normal toward camera (nadir)
            continue
        R, t = Rs[i], Ts[i]
        P2 = K @ np.hstack([R, t])
        Xh = cv2.triangulatePoints(P1, P2, q1, q2)
        w = Xh[3]
        X = (Xh[:3] / np.where(np.abs(w) < 1e-12, 1e-12, w)).T
        if (X[:, 2] > 0).mean() < 0.9:      # cheirality
            continue
        if best is None or n[2] > best[0]:
            best = (float(n[2]), R, t, n)
    return None if best is None else best[1:]


def estimate_motion_h(p1, p2, K, height):
    """Planar ground-plane motion via homography decomposition.

    Returns (R, t_metric, inlier_mask) or None. t is metric because the
    plane distance is the known camera height (nadir camera over flat lawn).
    """
    if len(p1) < 20:
        return None
    H, mask = cv2.findHomography(p1, p2, cv2.RANSAC, 2.0)
    if H is None or mask is None:
        return None
    inl = mask.ravel().astype(bool)
    # strict: a reliable ground-plane fit needs many inliers; box views
    # give <50, clean lawn views give >2000.
    if inl.sum() < 200 or inl.mean() < 0.6:
        return None
    sol = _pick_homography_solution(H, K, p1, p2, inl)
    if sol is None:
        return None
    R, t_over_d, n = sol
    t_metric = t_over_d * height
    # sanity: triangulated ground should sit ~height below camera
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = K @ np.hstack([R, t_metric])
    idx = np.arange(inl.sum())[::max(1, inl.sum() // 60)]
    q1 = np.ascontiguousarray(p1[inl].reshape(-1, 2)[idx].T)
    q2 = np.ascontiguousarray(p2[inl].reshape(-1, 2)[idx].T)
    Xh = cv2.triangulatePoints(P1, P2, q1, q2)
    w = Xh[3]
    X = (Xh[:3] / np.where(np.abs(w) < 1e-12, 1e-12, w)).T
    X = X[X[:, 2] > 0.05]
    if len(X) < 10 or abs(np.median(X[:, 2]) - height) > 0.35 * height:
        return None                        # locked onto a non-ground plane
    return R, t_metric, inl


def estimate_motion_e(p1, p2, K):
    """Fallback: essential matrix for non-planar scenes. Returns (R, t_unit, inl)."""
    if len(p1) < 20:
        return None
    E, mask = cv2.findEssentialMat(p1, p2, K, method=cv2.RANSAC,
                                   prob=0.999, threshold=1.0)
    if E is None:
        return None
    inl = mask.ravel().astype(bool)
    if inl.sum() < 20:
        return None
    _, R, t, _ = cv2.recoverPose(E, p1[inl], p2[inl], K)
    return R, t, inl


def recover_scale(p1, p2, R, t, K, R_wc, n_world, height, prev_scale):
    """Scale the unit translation using the known camera height.

    Triangulate inliers in cam_{i-1} frame (unit scale), fit ground plane
    with expected normal n_world (expressed in cam frame), then
    scale = height / median plane distance.
    """
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = K @ np.hstack([R, t])
    q1 = np.ascontiguousarray(p1.reshape(-1, 2).T)
    q2 = np.ascontiguousarray(p2.reshape(-1, 2).T)
    Xh = cv2.triangulatePoints(P1, P2, q1, q2)
    w = Xh[3]
    X = (Xh[:3] / np.where(np.abs(w) < 1e-12, 1e-12, w)).T
    X = X[np.isfinite(X).all(axis=1) & (X[:, 2] > 0.05)]   # cheirality-ish guard
    if len(X) < 30:
        return prev_scale, 0.0
    n_cam = R_wc.T @ n_world                    # expected ground normal in cam frame
    d = X @ n_cam
    med = np.median(d)
    if abs(med) < 1e-6:
        return prev_scale, 0.0
    plane_inl = np.abs(d - med) < 0.15 * abs(med)
    ratio = plane_inl.mean()
    if ratio < 0.25:
        return prev_scale, ratio
    scale = height / (np.median(d[plane_inl]))
    if not np.isfinite(scale) or scale <= 0 or scale > 5 * prev_scale + 1e-9:
        return prev_scale, ratio
    # smooth against jumps
    scale = 0.7 * scale + 0.3 * prev_scale
    return scale, ratio


# ---------------- evaluation ----------------

def umeyama(src, dst):
    """Similarity transform src -> dst (positions Nx3). Returns s, R, t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    cov = (dc.T @ sc) / len(src)
    U, S, Vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1, 1, d])
    R = U @ D @ Vt
    var = (sc ** 2).sum() / len(src)
    s = (S * np.diag(D)).sum() / var
    t = mu_d - s * R @ mu_s
    return s, R, t


def ate(p_est, p_gt):
    s, R, t = umeyama(p_est, p_gt)
    aligned = s * (p_est @ R.T) + t
    return float(np.sqrt(((aligned - p_gt) ** 2).sum(1).mean())), aligned


# ---------------- main ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--gt", default=None, help="synthetic groundtruth npz (provides K)")
    ap.add_argument("--fx", type=float, default=520.0)
    ap.add_argument("--fy", type=float, default=520.0)
    ap.add_argument("--cx", type=float, default=320.0)
    ap.add_argument("--cy", type=float, default=240.0)
    ap.add_argument("--height", type=float, default=1.0,
                    help="camera height above ground in meters")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outdir = args.out or os.path.join(base, "results")
    os.makedirs(outdir, exist_ok=True)

    if args.gt:
        gt = np.load(args.gt)
        K = gt["K"]
        p_gt, R_gt = gt["positions"], gt["rotations"]
    else:
        K = np.array([[args.fx, 0, args.cx], [0, args.fy, args.cy], [0, 0, 1.0]])
        p_gt = R_gt = None

    if os.path.isdir(args.video):
        import glob
        frame_files = sorted(glob.glob(os.path.join(args.video, "*.png")))
        if not frame_files:
            sys.exit("no png frames found in " + args.video)
        _it = iter(frame_files)

        def _read():
            try:
                return True, cv2.imread(next(_it))
            except StopIteration:
                return False, None

        def _release():
            pass
    else:
        cap = cv2.VideoCapture(args.video)

        def _read():
            return cap.read()

        def _release():
            cap.release()

    ok, frame = _read()
    if not ok or frame is None:
        sys.exit("cannot read video/frames")
    gray_prev = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    H_img, W_img = gray_prev.shape

    # world frame := first camera frame
    R_wc = np.eye(3)
    C = np.zeros(3)
    traj = [C.copy()]
    rots = [R_wc.copy()]
    prev_R, prev_t = None, None
    inlier_hist, scale_hist, method_hist, yaw_hist = [], [], [], []
    viz_frame = None

    pts = detect_features(gray_prev)
    idx = 0
    while True:
        ok, frame = _read()
        if not ok or frame is None:
            break
        idx += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if pts is None or len(pts) < 120:
            pts = detect_features(gray_prev)
            if pts is None:
                gray_prev = gray
                traj.append(C.copy()); rots.append(R_wc.copy())
                continue
        p1, p2 = track_features(gray_prev, gray, pts)
        R = t_metric = inl = None
        method = None
        mh = estimate_motion_h(p1, p2, K, args.height)
        if mh is not None:
            R, t_metric, inl = mh
            # safety: reject physically implausible yaw jumps
            yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            if abs(np.degrees(yaw)) > 10.0:
                R = t_metric = inl = None
            else:
                method = "H"
        if method is None:
            # unreliable pair: constant-velocity dead reckoning from last good
            # motion (better than freezing during box occlusions).
            if prev_R is not None:
                R, t_metric = prev_R, prev_t
                inl = np.zeros(len(p1), dtype=bool)
                method = "CV"
            else:
                pts = detect_features(gray)
                gray_prev = gray
                traj.append(C.copy()); rots.append(R_wc.copy())
                continue
        inlier_hist.append(float(inl.mean()))
        method_hist.append(method)
        scale_hist.append(float(np.linalg.norm(t_metric)))
        yaw_hist.append(float(np.arctan2(R[1, 0], R[0, 0])))

        # integrate: X_w = R_wc X_c + C ; X_c2 = R X_c1 + t_metric
        C = C - (R_wc @ R.T @ t_metric.ravel())
        R_wc = R_wc @ R.T
        traj.append(C.copy()); rots.append(R_wc.copy())
        if method in ("H", "E"):
            prev_R, prev_t = R.copy(), t_metric.copy()

        if viz_frame is None and idx == 60:
            viz = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for a, b in zip(p1[inl][::3], p2[inl][::3]):
                cv2.line(viz, tuple(a.ravel().astype(int)),
                         tuple(b.ravel().astype(int)), (0, 255, 0), 1)
                cv2.circle(viz, tuple(b.ravel().astype(int)), 2, (0, 0, 255), -1)
            viz_frame = viz

        pts = detect_features(gray)
        gray_prev = gray

    _release()
    traj = np.array(traj)
    rots = np.array(rots)

    metrics = {
        "frames": int(idx + 1),
        "mean_inlier_ratio": float(np.mean(inlier_hist)) if inlier_hist else 0.0,
        "frac_homography": float(np.mean([m == "H" for m in method_hist])) if method_hist else 0.0,
        "mean_step_m": float(np.mean(scale_hist)) if scale_hist else 0.0,
        "est_path_length_m": float(np.linalg.norm(np.diff(traj, axis=0), axis=1).sum()),
    }
    if p_gt is not None:
        m = min(len(traj), len(p_gt))
        err, aligned = ate(traj[:m], p_gt[:m])
        metrics["ATE_m"] = err
        metrics["gt_path_length_m"] = float(
            np.linalg.norm(np.diff(p_gt[:m], axis=0), axis=1).sum())
        # relative pose error (translation / rotation per step)
        dt, dr = [], []
        for i in range(1, m):
            de = np.linalg.norm(np.diff(traj[:m], axis=0)[i - 1])
            dg = np.linalg.norm(np.diff(p_gt[:m], axis=0)[i - 1])
            dt.append(abs(de - dg))
            Re = rots[i - 1].T @ rots[i]
            Rg = R_gt[i - 1].T @ R_gt[i]
            dr.append(float(np.arccos(np.clip((np.trace(Re.T @ Rg) - 1) / 2, -1, 1))))
        metrics["RPE_trans_m_per_step"] = float(np.mean(dt))
        metrics["RPE_rot_deg_per_step"] = float(np.degrees(np.mean(dr)))
    else:
        aligned = None

    np.savez(os.path.join(outdir, "trajectory.npz"),
             est_positions=traj, est_rotations=rots,
             gt_positions=p_gt if p_gt is not None else np.zeros((0, 3)))
    with open(os.path.join(outdir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))

    # ---- plots ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    if aligned is not None:
        ax.plot(aligned[:, 0], aligned[:, 1], "b-", lw=1.5, label="VO estimate (aligned)")
    else:
        ax.plot(traj[:, 0], traj[:, 1], "b-", lw=1.5, label="VO estimate")
    if p_gt is not None:
        ax.plot(p_gt[:, 0], p_gt[:, 1], "g--", lw=1.5, label="ground truth")
    ax.set_aspect("equal"); ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_title("Stage IV Motion — top-down trajectory (world X-Y)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.savefig(os.path.join(outdir, "trajectory_topdown.png"), dpi=120)
    plt.close(fig)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    traj3d = aligned if aligned is not None else traj
    ax.plot(traj3d[:, 0], traj3d[:, 1], traj3d[:, 2], "b-", lw=1.2,
            label="VO estimate" + (" (aligned)" if aligned is not None else ""))
    if p_gt is not None:
        ax.plot(p_gt[:, 0], p_gt[:, 1], p_gt[:, 2], "g--", lw=1.2, label="ground truth")
    ax.legend(); ax.set_title("Stage IV Motion — 3D trajectory")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    fig.savefig(os.path.join(outdir, "trajectory_3d.png"), dpi=120)
    plt.close(fig)

    if viz_frame is not None:
        cv2.imwrite(os.path.join(outdir, "flow_viz.png"), viz_frame)
    print("saved results to", outdir)


if __name__ == "__main__":
    main()
