#!/usr/bin/env python3
"""Compare near and distant frame geometry against a plane model and SfM poses.

Uses recorded camera calibration and image-only SIFT matches. It never changes
the source recording and never calls RGB-D or IMU to fit a motion estimate.
"""
import argparse
import csv
import json
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

from recording_camera import COLOR, Recording, rays
from sfm_keyframes import assess_two_view


def spatial_holdout(uv, width, height):
    """Reserve disjoint checkerboard image regions for evaluation."""
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    if width <= 0 or height <= 0:
        raise ValueError("image width and height must be positive")
    x = np.clip((uv[:, 0] * 6 / width).astype(int), 0, 5)
    y = np.clip((uv[:, 1] * 5 / height).astype(int), 0, 4)
    return (x + 2 * y) % 5 == 0


def pixel_errors(homography, source, target):
    if homography is None or not np.isfinite(homography).all():
        return np.full(len(source), np.inf)
    predicted = cv2.perspectiveTransform(
        np.asarray(source, np.float64).reshape(1, -1, 2),
        np.asarray(homography, np.float64)).reshape(-1, 2)
    result = np.linalg.norm(predicted - target, axis=1)
    result[~np.isfinite(result)] = np.inf
    return result


def epipolar_errors(source, target, relative_pose, focal):
    """Sampson distance in rectified pixels, a ONE-dimensional residual."""
    pose = np.asarray(relative_pose, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("relative camera pose must be a finite 4x4 matrix")
    if focal <= 0:
        raise ValueError("positive focal length required")
    rotation, translation = pose[:3, :3], pose[:3, 3]
    if np.linalg.norm(translation) < 1e-8:
        return None
    x, y, z = translation
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    inv_k = np.diag([1 / focal, 1 / focal, 1.])
    fundamental = inv_k @ skew @ rotation @ inv_k
    a = np.column_stack([source, np.ones(len(source))])
    b = np.column_stack([target, np.ones(len(target))])
    fa = a @ fundamental.T
    ftb = b @ fundamental
    denom = fa[:, 0]**2 + fa[:, 1]**2 + ftb[:, 0]**2 + ftb[:, 1]**2
    numerator = np.sum(b * fa, axis=1)**2
    return np.sqrt(numerator / np.maximum(denom, 1e-18))


def compare_matches(source, target, source_raw, image_size, focal, min_matches=40,
                    min_parallax_deg=2., relative_pose=None):
    """Fit geometry on training tiles and score image regions kept out of fit."""
    source = np.asarray(source, float).reshape(-1, 2)
    target = np.asarray(target, float).reshape(-1, 2)
    if len(source) != len(target) or len(source) != len(source_raw):
        raise ValueError("matches need equal lengths")
    if min_matches < 8 or focal <= 0 or min_parallax_deg <= 0:
        raise ValueError("invalid geometry thresholds")
    if not (np.isfinite(source).all() and np.isfinite(target).all()):
        raise ValueError("all matched points must be finite")
    hold = spatial_holdout(source_raw, *image_size)
    train = ~hold
    result = {"matches": int(len(source)), "training_matches": int(train.sum()),
              "holdout_matches": int(hold.sum()), "eligible": False,
              "plane_check": "not_evaluated", "status": "insufficient_matches"}
    if train.sum() < min_matches or hold.sum() < 12:
        return result

    cv2.setRNGSeed(0)
    H, inliers = cv2.findHomography(source[train], target[train],
                                    cv2.RANSAC, 2.5,
                                    maxIters=3000, confidence=.995)
    result["plane_inlier_fraction"] = (
        float(np.mean(inliers)) if inliers is not None else None)
    if H is None or not np.isfinite(H).all():
        result["status"] = "plane_fit_failed"
        return result
    plane_errors = pixel_errors(H, source[hold], target[hold])
    result.update(plane_check=("strong" if result["plane_inlier_fraction"] >= .8
                               else "weak_or_nonplanar"),
                  plane_holdout_median_px=float(np.median(plane_errors)),
                  plane_holdout_p90_px=float(np.percentile(plane_errors, 90)))

    calibration = np.diag([focal, focal, 1.])
    try:
        cv2.setRNGSeed(0)
        geometry = assess_two_view(source[train], target[train], calibration,
                                   min_parallax_deg=min_parallax_deg,
                                   min_points=max(20, min_matches // 2))
        for field in ("eligible", "reason", "median_parallax_deg",
                      "rotation_deg", "cheirality_points",
                      "rotation_only_inlier_fraction"):
            if field in geometry:
                result[field] = geometry[field]
    except cv2.error:
        result["reason"] = "essential_estimation_failed"
    if relative_pose is not None:
        e = epipolar_errors(source[hold], target[hold], relative_pose, focal)
        if e is not None:
            result["sfm_holdout_sampson_median_px"] = float(np.median(e))
            result["sfm_holdout_sampson_p90_px"] = float(np.percentile(e, 90))
    result["status"] = "evaluated"
    return result


def match_frames(first, second):
    """Reciprocal SIFT ratio test; only accepted matches enter evaluation."""
    sift = cv2.SIFT_create(nfeatures=2500)
    a, da = sift.detectAndCompute(cv2.cvtColor(first, cv2.COLOR_BGR2GRAY), None)
    b, db = sift.detectAndCompute(cv2.cvtColor(second, cv2.COLOR_BGR2GRAY), None)
    if da is None or db is None or len(da) < 2 or len(db) < 2:
        return np.empty((0, 2)), np.empty((0, 2))
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(da, db, k=2)
    backward = matcher.knnMatch(db, da, k=2)
    reverse = {pair[0].queryIdx: pair[0].trainIdx
               for pair in backward if len(pair) == 2
               and pair[0].distance < .75 * pair[1].distance}
    matches = [pair[0] for pair in forward if len(pair) == 2
               and pair[0].distance < .75 * pair[1].distance
               and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    source = np.array([a[m.queryIdx].pt for m in matches], float).reshape(-1, 2)
    target = np.array([b[m.trainIdx].pt for m in matches], float).reshape(-1, 2)
    return source, target


def relative_sfm_pose(sfm_poses, first, second):
    if first not in sfm_poses or second not in sfm_poses:
        return None
    return np.linalg.inv(sfm_poses[second]) @ sfm_poses[first]


def summarize(rows, gaps):
    summary = {}
    for gap in gaps:
        records = [r for r in rows if r["gap_frames"] == gap]
        good = [r for r in records if r["status"] == "evaluated"]
        summary[str(gap)] = {
            "attempted_pairs": len(records), "evaluated_pairs": len(good),
            "parallax_eligible_pairs": sum(bool(r["eligible"]) for r in good),
            "strong_plane_pairs": sum(r["plane_check"] == "strong" for r in good),
            "median_plane_holdout_px": (float(np.median(
                [r["plane_holdout_median_px"] for r in good])) if good else None),
            "median_parallax_deg": (float(np.median(
                [r["median_parallax_deg"] for r in good
                 if "median_parallax_deg" in r]))
                if any("median_parallax_deg" in r for r in good) else None),
            "sfm_pose_pairs": sum("sfm_holdout_sampson_median_px" in r for r in good)}
    return summary


def run(bag, output, gaps, anchor_step, sfm_path=None, min_matches=40):
    if not gaps or any(g < 1 for g in gaps) or anchor_step < 1:
        raise ValueError("frame gaps and anchor step must be positive")
    if sfm_path is not None:
        sfm = json.loads(Path(sfm_path).read_text())
        if Path(sfm["recording"]).name != Path(bag).name:
            raise ValueError("SfM reconstruction is from a different recording")
        sfm_poses = {int(i): np.asarray(p, float)
                     for i, p in zip(sfm["registered_frames"], sfm["poses_camera_to_world"])}
    else:
        sfm_poses = {}
    recording = Recording(bag)
    try:
        count = len(recording.color_times)
        focal = float((recording.color["fx"] + recording.color["fy"]) / 2)
        width, height = recording.color["width"], recording.color["height"]

        @lru_cache(maxsize=24)
        def image(index):
            return recording.image(COLOR, index)

        rows = []
        for first in range(0, count - 1, anchor_step):
            for gap in gaps:
                second = first + gap
                if second >= count:
                    continue
                raw_a, raw_b = match_frames(image(first), image(second))
                row = {"source_frame": first, "target_frame": second,
                       "gap_frames": gap, "gap_seconds": round(
                           (int(recording.color_times[second]) -
                            int(recording.color_times[first])) / 1e9, 6)}
                if len(raw_a):
                    # Rectify using this recording's own distortion calibration.
                    a = rays(raw_a, recording.color)[:, :2] * focal
                    b = rays(raw_b, recording.color)[:, :2] * focal
                else:
                    a = b = np.empty((0, 2))
                row.update(compare_matches(
                    a, b, raw_a, (width, height), focal,
                    min_matches=min_matches,
                    relative_pose=relative_sfm_pose(sfm_poses, first, second)))
                rows.append(row)
                print(f"{Path(bag).name} {first}->{second}: "
                      f"{row['status']} {row['matches']} matches", flush=True)
        result = {
            "recording": Path(bag).name, "frame_count": count, "gaps_frames": gaps,
            "anchor_step_frames": anchor_step,
            "camera_calibration": "recorded_intrinsics_and_distortion_rectified",
            "matching": "reciprocal_sift_0.75_ratio",
            "fit": "2.5_pixel_homography_ransac_on_training_tiles",
            "holdout": "spatial_checkerboard_tiles_not_used_for_homography_fit",
            "sfm_pose_source": str(Path(sfm_path).name) if sfm_path else None,
            "interpretation": ("plane 2D transfer error and SfM 1D epipolar "
                               "Sampson residual have different dimensions; "
                               "do not rank them by numerical magnitude"),
            "metric_ground_truth_available": False,
            "summary": summarize(rows, gaps), "pairs": rows}
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        columns = ["source_frame", "target_frame", "gap_frames", "gap_seconds",
                   "matches", "training_matches", "holdout_matches", "status",
                   "eligible", "reason", "median_parallax_deg", "rotation_deg",
                   "plane_check", "plane_inlier_fraction",
                   "plane_holdout_median_px", "plane_holdout_p90_px",
                   "sfm_holdout_sampson_median_px", "sfm_holdout_sampson_p90_px"]
        with output.with_suffix(".csv").open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        return result
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gaps", type=int, nargs="+", default=[5, 25, 50])
    parser.add_argument("--anchor-step", type=int, default=50)
    parser.add_argument("--sfm", type=Path,
                        help="optional reconstruction.json from the same video")
    parser.add_argument("--min-matches", type=int, default=40)
    args = parser.parse_args()
    try:
        result = run(args.bag, args.out, sorted(set(args.gaps)),
                     args.anchor_step, args.sfm, args.min_matches)
        print(json.dumps(result["summary"], indent=2))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
