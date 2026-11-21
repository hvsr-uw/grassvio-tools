#!/usr/bin/env python3
"""Audit SIFT matches before RANSAC on the same wind-video frame pairs."""
import argparse
from contextlib import closing
import csv
from functools import lru_cache
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stitch-map"))
from geometry import apply_transform, fit_homography
from recording_camera import COLOR, Recording
from stitch_map import LOWE_RATIO, MIN_INLIERS, RANSAC_THRESH_PX, make_detector
from window_motion import temporal_filter, track_window


def sift_matches(first, last, matcher):
    """Use the same one-way Lowe-ratio match set as the original inlier audit."""
    ka, da = first
    kb, db = last
    if da is None or db is None or len(da) < 2:
        return np.empty((2, 0, 2), np.float32), 0
    pairs = [pair for pair in matcher.knnMatch(db, da, k=2) if len(pair) == 2]
    good = [m for m, n in pairs if m.distance < LOWE_RATIO * n.distance]
    return np.float32([[ka[m.trainIdx].pt for m in good],
                       [kb[m.queryIdx].pt for m in good]]).reshape(2, -1, 2), len(pairs)


def fit(matches, selection):
    # Map end-frame pixels into the start frame in every comparison.
    cv2.setRNGSeed(0)
    return fit_homography(matches[1, selection], matches[0, selection],
                          threshold_px=RANSAC_THRESH_PX, min_inliers=MIN_INLIERS)


def score(H, matches, selection):
    if H is None or not selection.any():
        return None
    error = np.linalg.norm(apply_transform(H, matches[1, selection])
                           - matches[0, selection], axis=1)
    return float(np.median(error)) if np.isfinite(error).all() else None


def inspect_pair(grays, matches, candidate_count):
    """Separate flow corroboration from the temporal residual gate."""
    n = matches.shape[1]
    history, _, ids = track_window(grays, initial_points=matches[0], return_ids=True)
    endpoint_error = np.linalg.norm(history[-1] - matches[1, ids], axis=1)
    consistent = endpoint_error <= 2.0
    status = np.full(n, "flow_incomplete", dtype="<U24")
    status[ids] = "endpoint_disagrees"
    ids, history = ids[consistent], history[:, consistent]
    keep, residuals = temporal_filter(history)
    status[ids] = np.where(keep, "kept", "temporal_rejected")
    flow_supported = np.zeros(n, bool)
    flow_supported[ids] = True

    _, mask = fit(matches, np.ones(n, bool))
    inliers = np.zeros(n, bool) if mask is None else mask.ravel().astype(bool)
    baseline_count = int(inliers.sum())
    _, filtered_mask = fit(matches, status == "kept")
    after_count = int(filtered_mask.sum()) if filtered_mask is not None else 0
    survivors = int(keep.sum())

    # Held-out checkerboard tiles influence neither the temporal model nor RANSAC.
    held_out = np.floor(matches[0] / 40).astype(int).sum(axis=1) % 2 == 1
    train = ~held_out
    train_history = train[ids]
    train_keep, _ = temporal_filter(history[:, train_history])
    selected = np.zeros(n, bool)
    selected[ids[train_history][train_keep]] = True
    H0, _ = fit(matches, train)
    H1, _ = fit(matches, train & flow_supported)
    H2, _ = fit(matches, selected)
    shared = held_out & flow_supported
    result = dict(
        knn_pairs=candidate_count, sift_matches=n,
        raw_inliers=baseline_count,
        raw_inlier_fraction=baseline_count/n if n else None,
        flow_incomplete=int(np.sum(status == "flow_incomplete")),
        endpoint_disagrees=int(np.sum(status == "endpoint_disagrees")),
        temporal_rejected=int(np.sum(status == "temporal_rejected")),
        temporal_rejected_raw_inliers=int(np.sum(inliers & (status == "temporal_rejected"))),
        kept=survivors, after_inliers=after_count,
        after_inlier_fraction=after_count/survivors if survivors else None,
        holdout_candidates=int(held_out.sum()), holdout_flow_supported=int(shared.sum()),
        raw_median_px=score(H0, matches, shared),
        flow_only_median_px=score(H1, matches, shared),
        prefiltered_median_px=score(H2, matches, shared),
    )
    # Retain exact endpoints and labels for inspection, not just a plotted sample.
    residual = np.full(n, np.nan)
    residual[ids] = residuals
    return result, status, inliers, residual


def draw_matches(first, last, matches, status, inliers, recording, start, end):
    """Show a fixed-size sample from each class, with exact IDs at both ends."""
    classes = [
        ("kept", inliers & (status == "kept"), (65, 226, 165)),
        ("temporal reject", inliers & (status == "temporal_rejected"), (67, 100, 255)),
        ("flow check failed", inliers & np.isin(status, ["flow_incomplete", "endpoint_disagrees"]),
         (255, 200, 72)),
    ]
    h, w = first.shape[:2]
    panel = np.full((h+95, 2*w+12, 3), 248, np.uint8)
    panel[95:, :w] = first
    panel[95:, w+12:] = last

    def label(s, x, y, color=(35, 35, 35), size=.55):
        cv2.putText(panel, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)

    label(f"{recording} | baseline RANSAC inliers | numbered examples", 10, 23)
    for i, (name, mask, color) in enumerate(classes):
        left = 12 + 230*i
        cv2.circle(panel, (left, 48), 5, color, -1)
        label(f"{name}: {mask.sum()}", left+12, 53)
        indices = np.flatnonzero(mask)
        chosen = indices[np.linspace(0, len(indices)-1, min(7, len(indices)), dtype=int)]
        for j in chosen:
            for k, offset in enumerate((0, w+12)):
                x, y = np.rint(matches[k, j]).astype(int) + [offset, 95]
                cv2.circle(panel, (x, y), 6, (25, 25, 25), 3, cv2.LINE_AA)
                cv2.circle(panel, (x, y), 6, color, 2, cv2.LINE_AA)
                point = (min(x+7, offset+w-39), max(109, y-6))
                cv2.putText(panel, str(j), point, cv2.FONT_HERSHEY_SIMPLEX, .37,
                            (15, 15, 15), 3, cv2.LINE_AA)
                label(str(j), *point, color, .37)
    label(f"Frame {start}", 10, 82)
    label(f"Frame {end}", w+22, 82)
    return panel


def draw_linked_matches(first, last, matches, status, inliers, clip, start, end):
    """Join the saved SIFT endpoints on the original frames without refitting."""
    classes = (
        ("Passed both checks", "kept", (139, 241, 85)),
        ("Failed temporal check", "temporal_rejected", (52, 165, 255)),
        ("Failed flow check", ("flow_incomplete", "endpoint_disagrees"), (255, 208, 90)),
    )
    h, w = first.shape[:2]
    gap, top, bottom = 24, 111, 38
    panel = np.full((h + top + bottom, 2 * w + gap, 3), 250, np.uint8)
    panel[top:top+h, :w] = first
    panel[top:top+h, w+gap:] = last
    panel[top:top+h, w:w+gap] = 40

    def label(message, x, y, size=.55, weight=1):
        cv2.putText(panel, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    size, (36, 36, 36), weight, cv2.LINE_AA)

    label(f"Wind clip {clip} | Feature pairs before RANSAC", 14, 29, .76, 2)
    label(f"Source frame {start}", 14, top-11, .58, 2)
    label(f"Target frame {end}", w+gap+14, top-11, .58, 2)
    lines = panel.copy()
    endpoints = []
    for column, (name, decisions, color) in enumerate(classes):
        indices = np.flatnonzero(inliers & np.isin(status, decisions))
        # Spread the examples vertically so the links remain easy to follow.
        indices = indices[np.argsort(matches[0, indices, 1], kind="stable")]
        shown = min(7, len(indices))
        chosen = indices[np.round(np.linspace(0, len(indices)-1, shown)).astype(int)]
        x = 14 + column * ((panel.shape[1] - 24) // 3)
        cv2.circle(panel, (x+6, 59), 6, (35, 35, 35), -1, cv2.LINE_AA)
        cv2.circle(panel, (x+6, 59), 5, color, -1, cv2.LINE_AA)
        label(f"{name} ({shown} of {len(indices)} shown)", x+18, 64, .56)
        for j in chosen:
            a, b = np.rint(matches[:, j]).astype(int) + [[0, top], [w+gap, top]]
            cv2.line(lines, a, b, (20, 20, 20), 3, cv2.LINE_AA)
            cv2.line(lines, a, b, color, 1, cv2.LINE_AA)
            endpoints.append((a, b, color))

    panel[top:top+h] = cv2.addWeighted(lines[top:top+h], .82, panel[top:top+h], .18, 0)
    for a, b, color in endpoints:
        for point in (a, b):
            cv2.circle(panel, point, 4, (20, 20, 20), -1, cv2.LINE_AA)
            cv2.circle(panel, point, 3, color, -1, cv2.LINE_AA)
    label("Examples drawn from matches accepted by the original RANSAC fit.", 14, h+top+24)
    return panel


def render_saved_pair(bag_path, archive, output_dir, start, end):
    key = f"{start:03d}_{end:03d}"
    clip = bag_path.stem.rsplit("_", 1)[-1]
    with np.load(archive) as saved, closing(Recording(bag_path)) as bag:
        panel = draw_linked_matches(
            bag.image(COLOR, start), bag.image(COLOR, end), saved[f"{key}_endpoints"],
            saved[f"{key}_decision"], saved[f"{key}_raw_inlier"], clip, start, end,
        )
    path = output_dir / f"{clip}_linked_matches_frames_{key}.png"
    if not cv2.imwrite(str(path), panel):
        raise OSError(f"could not save {path}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--matches-npz", type=Path, help="render a saved pair without rerunning the audit")
    parser.add_argument("--pair", nargs=2, type=int, metavar=("START", "END"))
    args = parser.parse_args()
    if bool(args.matches_npz) != bool(args.pair):
        parser.error("--matches-npz and --pair must be used together")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.matches_npz:
        print(render_saved_pair(args.bag, args.matches_npz, args.output_dir, *args.pair))
        return
    cv2.setNumThreads(2)
    detector = make_detector("sift")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    rows, arrays = [], {}
    recording = args.bag.stem
    example_pairs = {(0, 5), (25, 30), (100, 105), (195, 200), (200, 225)}
    with closing(Recording(args.bag)) as bag:
        grays = [cv2.cvtColor(bag.image(COLOR, i), cv2.COLOR_BGR2GRAY)
                 for i in range(len(bag.color_times))]

        @lru_cache(maxsize=2)
        def features(index):
            return detector.detectAndCompute(grays[index], None)

        for step in (5, 25):
            for start in range(0, len(grays)-1, step):
                end = min(start+step, len(grays)-1)
                if step == 5 and end-start != step:
                    continue
                matches, count = sift_matches(features(start), features(end), matcher)
                stats, status, inliers, residual = inspect_pair(grays[start:end+1], matches, count)
                rows.append(dict(start=start, end=end, nominal_window=step, **stats))
                key = f"{start:03d}_{end:03d}"
                arrays[key+"_endpoints"] = matches
                arrays[key+"_decision"] = status
                arrays[key+"_raw_inlier"] = inliers
                arrays[key+"_residual_px"] = residual
                if (start, end) in example_pairs:
                    panel = draw_matches(bag.image(COLOR, start), bag.image(COLOR, end), matches,
                                         status, inliers, recording, start, end)
                    if not cv2.imwrite(str(args.output_dir / f"{recording}_matches_{key}.png"), panel):
                        raise OSError("could not save match panel")
                print(f"{start:03d}->{end:03d}  matches={count} kept={stats['kept']}", flush=True)

    (args.output_dir / f"{recording}_window_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    with (args.output_dir / f"{recording}_window_results.csv").open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(args.output_dir / f"{recording}_exact_matches.npz", **arrays)


if __name__ == "__main__":
    main()
