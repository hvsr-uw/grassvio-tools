#!/usr/bin/env python3
"""Check direct first-to-end matches beside each saved homography chain.

This is an image-loop consistency diagnostic, not metric pose ground truth.
The direct fit does not alter the maps or fill missing chained poses.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stitch-map"))
from geometry import apply_transform, fit_homography, projected_corners
from stitch_map import COLOR, parse_image, downscale


def evaluate_loop_points(source, target, chain_transform):
    source, target = np.asarray(source).reshape(-1, 2), np.asarray(target).reshape(-1, 2)
    direct, mask = fit_homography(source, target, min_inliers=20)
    output = {"matches": len(source), "direct_inliers": 0, "direct_fit_median_error_px": None,
              "chain_endpoint_available": chain_transform is not None,
              "chain_loop_median_error_px": None, "direct_homography": None}
    if direct is None:
        return output
    good = mask.ravel() != 0
    output.update(direct_inliers=int(good.sum()), direct_homography=direct.tolist(),
                  direct_fit_median_error_px=float(np.median(np.linalg.norm(
                      apply_transform(direct, source[good]) - target[good], axis=1))))
    if chain_transform is not None:
        errors = np.linalg.norm(apply_transform(chain_transform, source[good]) - target[good], axis=1)
        if np.isfinite(errors).all():
            output["chain_loop_median_error_px"] = float(np.median(errors))
    return output


def match_loop(first, last):
    sift, matcher = cv2.SIFT_create(nfeatures=10000), cv2.BFMatcher()
    a, da = sift.detectAndCompute(cv2.cvtColor(first, cv2.COLOR_BGR2GRAY), None)
    b, db = sift.detectAndCompute(cv2.cvtColor(last, cv2.COLOR_BGR2GRAY), None)
    if da is None or db is None or len(da) < 2 or len(db) < 2:
        return np.empty((0, 2)), np.empty((0, 2))
    reverse = {m.queryIdx: m.trainIdx for m in matcher.match(da, db)}
    good = [pair[0] for pair in matcher.knnMatch(db, da, k=2)
            if len(pair) == 2 and pair[0].distance < .75 * pair[1].distance
            and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    return (np.array([b[m.queryIdx].pt for m in good]).reshape(-1, 2),
            np.array([a[m.trainIdx].pt for m in good]).reshape(-1, 2))


def check_direct_footprint(result, shape):
    """Keep direct match support separate from full-frame geometry."""
    if result["direct_homography"] is None:
        return {"direct_full_frame_footprint_valid": None, "direct_footprint_error": None}
    try:
        projected_corners(np.asarray(result["direct_homography"]), shape)
        return {"direct_full_frame_footprint_valid": True, "direct_footprint_error": None}
    except ValueError as exc:
        return {"direct_full_frame_footprint_valid": False, "direct_footprint_error": str(exc)}


def load_selected_color(path, indices):
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        tid = db.execute("SELECT id FROM topics WHERE name=?", (COLOR,)).fetchone()[0]
        rowids = [row[0] for row in db.execute(
            "SELECT rowid FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))]
        frames = {}
        for index in set(indices):
            blob = db.execute("SELECT data FROM messages WHERE rowid=?", (rowids[index],)).fetchone()[0]
            p = parse_image(blob)
            if p["encoding"] != "rgb8":
                raise ValueError("loop check currently requires rgb8 color frames")
            image = np.frombuffer(p["data"], np.uint8).reshape(p["height"], p["width"], 3)
            frames[index] = downscale(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--results-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    inputs = sorted(args.input_dir.glob("*.db3"))
    if not inputs:
        parser.error("no .db3 recordings found")
    records = []
    for path in inputs:
        directory = args.results_dir / path.stem
        stats = {stage: json.loads((directory / (stage + ".json")).read_text(encoding="utf-8"))
                 for stage in ("baseline", "filtered")}
        endpoints = {stage: item["selected_frames"][-1] for stage, item in stats.items()}
        failed_window = next((w for w in stats["filtered"].get("windows", []) if not w.get("placed")), None)
        indices = [0, *endpoints.values()]
        if failed_window:
            indices.append(failed_window["end_frame"])
        frames = load_selected_color(path, indices)
        result = {"recording": path.name, "metric_ground_truth_available": False,
                  "direct_fit_used_to_change_chain": False, "methods": {}}
        matches = {}
        for stage, item in stats.items():
            endpoint = endpoints[stage]
            if endpoint not in matches:
                matches[endpoint] = match_loop(frames[0], frames[endpoint])
            measured = evaluate_loop_points(*matches[endpoint], item["transforms_to_first"][-1])
            measured["endpoint_frame"] = endpoint
            result["methods"][stage] = measured
        if failed_window:
            endpoint = failed_window["end_frame"]
            failure = evaluate_loop_points(*match_loop(frames[0], frames[endpoint]), None)
            failure.update(check_direct_footprint(failure, frames[endpoint].shape),
                           endpoint_frame=endpoint,
                           chained_placement_error=failed_window.get("placement_error"),
                           local_window_inliers=failed_window.get("inliers"))
            result["first_filtered_placement_failure"] = failure
        (directory / "loop_check.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        records.append(result)
        print(json.dumps({"recording": path.name, "methods": {
            stage: {key: value for key, value in item.items() if key != "direct_homography"}
            for stage, item in result["methods"].items()},
            "first_filtered_placement_failure": {key: value for key, value in result.get(
                "first_filtered_placement_failure", {}).items() if key != "direct_homography"}}), flush=True)
    (args.results_dir / "loop_summary.json").write_text(
        json.dumps(records, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
