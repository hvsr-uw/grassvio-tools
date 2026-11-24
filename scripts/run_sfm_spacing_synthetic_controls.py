#!/usr/bin/env python3
"""Known-pose synthetic SfM baseline and flat-plane control.

This isolates triangulation conditioning. The camera poses are supplied as
known truth; this does not measure accuracy of an estimated SfM trajectory.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def run_case(half_height_m, seed=566):
    rng = np.random.default_rng(seed)
    count = 600
    xyz = np.column_stack((
        rng.uniform(-1.7, 1.7, count),
        rng.uniform(-1.1, 1.1, count),
        4 + rng.uniform(-half_height_m, half_height_m, count)))
    focal = 600.
    first = focal * xyz[:, :2] / xyz[:, 2:]
    tiles = (np.clip(((first[:, 0] + 320) * 6 / 640).astype(int), 0, 5)
             + 2 * np.clip(((first[:, 1] + 240) * 5 / 480).astype(int), 0, 4))
    holdout = tiles % 5 == 0
    results = []
    for gap in (5, 25, 50):
        baseline_m = .004 * gap
        angle_deg = .1 * gap
        rotation = cv2.Rodrigues(np.array([0., np.deg2rad(angle_deg), 0.]))[0]
        second_xyz = xyz @ rotation.T + [baseline_m, 0., 0.]
        second = focal * second_xyz[:, :2] / second_xyz[:, 2:]
        a = first + rng.normal(0, .2, first.shape)
        b = second + rng.normal(0, .2, second.shape)

        cv2.setRNGSeed(0)
        H, mask = cv2.findHomography(a[~holdout], b[~holdout],
                                      cv2.RANSAC, 1.5)
        predicted = cv2.perspectiveTransform(
            a[holdout].astype(np.float64)[None], H)[0]
        plane_error = float(np.median(np.linalg.norm(
            predicted - b[holdout], axis=1)))

        p1 = np.column_stack((a / focal, np.ones(len(a))))
        p2 = np.column_stack((b / focal, np.ones(len(b))))
        P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = np.hstack((rotation, np.array([[baseline_m], [0.], [0.]])))
        homogeneous = cv2.triangulatePoints(
            P1, P2, p1[:, :2].T, p2[:, :2].T)
        reconstructed = (homogeneous[:3] / homogeneous[3]).T
        error_3d = float(np.median(np.linalg.norm(
            reconstructed - xyz, axis=1)))

        ray1 = xyz / np.linalg.norm(xyz, axis=1)[:, None]
        ray2_in_first = second_xyz @ rotation
        ray2_in_first /= np.linalg.norm(ray2_in_first, axis=1)[:, None]
        parallax = float(np.degrees(np.median(np.arccos(
            np.clip(np.sum(ray1 * ray2_in_first, axis=1), -1, 1)))))
        results.append({
            "gap_frames": gap, "baseline_m": round(baseline_m, 3),
            "camera_rotation_deg": round(angle_deg, 3),
            "median_true_parallax_deg": round(parallax, 3),
            "median_plane_holdout_px": round(plane_error, 3),
            "median_known_pose_triangulation_error_m": round(error_3d, 3),
        })
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = {
        "experiment": "known_pose_synthetic_conditioning_not_real_video",
        "seed": 566, "points": 600,
        "image_noise_std_px": .2, "focal_px": 600.,
        "base_plane_depth_m": 4.,
        "translation_m_per_frame": .004,
        "camera_rotation_deg_per_frame": .1,
        "holdout": "spatial_tiles_excluded_from_homography_fit",
        "camera_poses_used_in_triangulation": "known_true_poses",
        "metric_ground_truth_available": True,
        "cases": {
            "exactly_planar": run_case(0.),
            "height_plus_minus_0_4m": run_case(.4),
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
