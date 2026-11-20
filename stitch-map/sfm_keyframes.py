#!/usr/bin/env python3
"""Choose SfM candidate pairs by rotation-compensated angular parallax.

This is a two-view conditioning/plane check, not a full SfM reconstruction.
It emits no metric translation. Calibration must match the input images.
Pure rotation is not a useful triangulation baseline, even at large angles.
"""
import argparse
import json

import cv2
import numpy as np

from geometry import apply_transform, fit_homography
from stitch_map import (load_frames_from_bag, load_frames_from_folder,
                        load_frames_from_video, make_detector)


def assess_two_view(src, dst, K, min_parallax_deg=2., min_points=20):
    src, dst = np.asarray(src, float).reshape(-1, 2), np.asarray(dst, float).reshape(-1, 2)
    K = np.asarray(K, float)
    if K.shape != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("need a finite camera calibration matrix with positive focal lengths")
    if len(src) != len(dst) or not np.isfinite([src, dst]).all():
        raise ValueError("need finite paired coordinates")
    result = {"matches": len(src), "eligible": False, "reason": "too_few_matches",
              "translation_scale": "unknown", "min_parallax_deg": min_parallax_deg}
    if len(src) < min_points:
        return result
    H, mask_h = fit_homography(src, dst, min_inliers=min_points)
    result["plane_inlier_fraction"] = float(mask_h.mean()) if mask_h is not None else 0.
    result["plane_check"] = "supported" if result["plane_inlier_fraction"] >= .8 else "weak_or_nonplanar"
    result["plane_homography"] = H.tolist() if H is not None else None
    if H is not None:
        result["plane_median_error_px"] = float(np.median(np.linalg.norm(apply_transform(H, src) - dst, axis=1)))
    invK = np.linalg.inv(K)
    rays_a = np.column_stack((src, np.ones(len(src)))) @ invK.T
    rays_b = np.column_stack((dst, np.ones(len(dst)))) @ invK.T
    rays_a /= np.linalg.norm(rays_a, axis=1, keepdims=True)
    rays_b /= np.linalg.norm(rays_b, axis=1, keepdims=True)
    if mask_h is not None:
        # Essential-matrix decomposition is degenerate under pure rotation
        # and can invent a translation with apparently positive depths.
        # Check a rotation-only fit before trusting that decomposition.
        consensus = mask_h.ravel() != 0
        U, _, Vt = np.linalg.svd(rays_b[consensus].T @ rays_a[consensus])
        correction = np.diag([1., 1., np.linalg.det(U @ Vt)])
        rotation = U @ correction @ Vt
        rotation_pixels = apply_transform(K @ rotation @ invK, src)
        errors = np.linalg.norm(rotation_pixels - dst, axis=1)
        support = np.isfinite(errors) & (errors <= 1.)
        result["rotation_only_inlier_fraction"] = float(support.mean())
        finite = errors[np.isfinite(errors)]
        result["rotation_only_median_error_px"] = float(np.median(finite)) if len(finite) else None
        if (support.sum() >= min_points and consensus.mean() >= .8 and
                support[consensus].mean() >= .9):
            result["reason"] = "rotation_only_explains_matches"
            result["R_source_to_target"] = rotation.tolist()
            result["rotation_deg"] = float(np.rad2deg(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
            return result
    E, mask = cv2.findEssentialMat(src, dst, K, method=cv2.RANSAC,
                                 prob=.999, threshold=1.)
    if E is None or mask is None:
        result["reason"] = "essential_fit_failed"
        return result
    best = None
    for candidate in np.asarray(E).reshape(-1, 3, 3):
        count, R, t, cheirality = cv2.recoverPose(candidate, src, dst, K, mask=mask.copy())
        good = cheirality.ravel() != 0
        if count < min_points:
            continue
        # Express both viewing rays in the SECOND camera, then measure
        # the angle that remains after removing camera rotation.
        rotated = rays_a @ R.T
        angles = np.rad2deg(np.arccos(np.clip(np.sum(rotated * rays_b, axis=1), -1, 1)))
        parallax = float(np.median(angles[good]))
        if best is None or count > best[0]:
            best = count, R, t, parallax
    if best is None:
        result["reason"] = "no_cheirality_baseline_or_pure_rotation"
        return result
    count, R, t, parallax = best
    result.update(cheirality_points=int(count), median_parallax_deg=parallax,
                  rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(R)-1)/2, -1, 1)))),
                  R_source_to_target=R.tolist(), t_unit_source_to_target=t.ravel().tolist(),
                  eligible=parallax >= min_parallax_deg,
                  reason="sufficient_angular_baseline" if parallax >= min_parallax_deg else "insufficient_parallax")
    if result["plane_check"] == "supported":
        result["caution"] = "planar scene: essential decomposition can be ambiguous; verify with plane model and independent poses"
    return result


def select_keyframes(frames, K, candidate_stride=5, min_parallax_deg=2.):
    if candidate_stride < 1 or min_parallax_deg <= 0:
        raise ValueError("positive candidate stride and parallax threshold required")
    detector, matcher = make_detector("sift"), cv2.BFMatcher()
    anchors, records = [0], []
    def features(index):
        return detector.detectAndCompute(cv2.cvtColor(frames[index], cv2.COLOR_BGR2GRAY), None)
    anchor_features = features(0)
    for target in range(candidate_stride, len(frames), candidate_stride):
        kp_a, da = anchor_features
        target_features = features(target)
        kp_b, db = target_features
        good = []
        if da is not None and db is not None and len(db) >= 2:
            reverse = {m.queryIdx: m.trainIdx for m in matcher.match(db, da)}
            good = [pair[0] for pair in matcher.knnMatch(da, db, k=2)
                    if len(pair) == 2 and pair[0].distance < .75 * pair[1].distance
                    and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
        src = np.array([kp_a[m.queryIdx].pt for m in good]).reshape(-1, 2)
        dst = np.array([kp_b[m.trainIdx].pt for m in good]).reshape(-1, 2)
        record = assess_two_view(src, dst, K, min_parallax_deg)
        record.update(source_frame=anchors[-1], target_frame=target)
        records.append(record)
        if record["eligible"]:
            anchors.append(target)
            anchor_features = target_features
    return {"keyframes": anchors, "pairs": records, "full_sfm_run": False,
            "metric_translation_available": False}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag")
    source.add_argument("--video")
    source.add_argument("--frames")
    ap.add_argument("--calibration", required=True, help='JSON {"K": [[fx,0,cx],[0,fy,cy],[0,0,1]]}; undistorted input')
    ap.add_argument("--out", required=True)
    ap.add_argument("--candidate-stride", type=int, default=5)
    ap.add_argument("--min-parallax-deg", type=float, default=2.)
    args = ap.parse_args()
    frames = (load_frames_from_bag(args.bag) if args.bag else
              load_frames_from_video(args.video) if args.video else
              load_frames_from_folder(args.frames))
    if len(frames) < 2 or any(f is None for f in frames):
        ap.error("need at least two readable input frames")
    with open(args.calibration) as f:
        K = np.array(json.load(f)["K"], float)
    result = select_keyframes(frames, K, args.candidate_stride, args.min_parallax_deg)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2, allow_nan=False)


if __name__ == "__main__":
    main()
