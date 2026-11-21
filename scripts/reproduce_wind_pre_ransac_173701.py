#!/usr/bin/env python3
"""Re-run the pre-RANSAC wind/flow comparison on a GrassVIO ROS2 recording.

This is a new implementation of the published 173701 procedure, not the
original analysis script. Results must be checked against the archived CSV
before claiming that the historical numerical results were reproduced.
"""
import argparse
import csv
from functools import lru_cache
import json
from pathlib import Path
import sqlite3
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bag-exploration"))
from explore_bag import COLOR, parse_image  # noqa: E402

COLUMNS = ("start", "end", "nominal_window", "knn_pairs", "sift_matches",
           "raw_inliers", "raw_inlier_fraction", "flow_incomplete",
           "endpoint_disagrees", "temporal_rejected", "temporal_rejected_raw_inliers",
           "kept", "after_inliers", "after_inlier_fraction", "holdout_candidates",
           "holdout_flow_supported", "raw_median_px", "flow_only_median_px",
           "prefiltered_median_px")


class ColorFrames:
    """Lazy, read-only access to the color stream in a ROS2 SQLite bag."""

    def __init__(self, path):
        self.db = sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)
        self.db.execute("PRAGMA query_only=ON")
        topic = self.db.execute("SELECT id FROM topics WHERE name=?", (COLOR,)).fetchone()
        if topic is None:
            raise ValueError("recording has no RGB color topic")
        self.ids = [r[0] for r in self.db.execute(
            "SELECT rowid FROM messages WHERE topic_id=? ORDER BY timestamp", topic)]

    def __len__(self):
        return len(self.ids)

    @lru_cache(maxsize=60)
    def gray(self, index):
        blob = self.db.execute("SELECT data FROM messages WHERE rowid=?",
                               (self.ids[index],)).fetchone()[0]
        frame = parse_image(blob)
        h, w = frame["height"], frame["width"]
        raw = np.frombuffer(frame["data"], np.uint8).reshape(h, frame["step"])
        if frame["encoding"] == "mono8":
            return raw[:, :w].copy()
        if frame["encoding"] not in ("rgb8", "bgr8"):
            raise ValueError(f"unsupported color encoding: {frame['encoding']}")
        rgb = raw[:, :w * 3].reshape(h, w, 3)
        conversion = (cv2.COLOR_RGB2GRAY if frame["encoding"] == "rgb8"
                      else cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(rgb, conversion)

    def close(self):
        self.gray.cache_clear()
        self.db.close()


def correspondences(first, last, detector, matcher):
    ka, da = detector.detectAndCompute(first, None)
    kb, db = detector.detectAndCompute(last, None)
    if da is None or db is None or len(db) < 2:
        empty = np.empty((0, 2), np.float32)
        return 0, empty, empty.copy()
    knn = matcher.knnMatch(da, db, k=2)
    good = [m for pair in knn if len(pair) == 2
            for m, n in [pair] if m.distance < 0.75 * n.distance]
    a = np.array([ka[m.queryIdx].pt for m in good], np.float32).reshape(-1, 2)
    b = np.array([kb[m.trainIdx].pt for m in good], np.float32).reshape(-1, 2)
    return len(knn), a, b


def fit(a, b, minimum=20):
    """Fit 3-pixel RANSAC, returning an explicit failure for weak matches."""
    if len(a) < max(4, minimum):
        return None, np.zeros(len(a), bool)
    cv2.setRNGSeed(566)
    h, mask = cv2.findHomography(a, b, cv2.RANSAC, 3.0,
                                 maxIters=5000, confidence=0.999)
    if h is None or mask is None or not np.isfinite(h).all():
        return None, np.zeros(len(a), bool)
    return h, mask.ravel().astype(bool)


def errors(h, a, b):
    if h is None or not len(a):
        return np.full(len(a), np.nan)
    predicted = cv2.perspectiveTransform(a.astype(np.float32).reshape(-1, 1, 2), h)
    return np.linalg.norm(predicted.reshape(-1, 2) - b, axis=1)


def track(grays, starts):
    """Lucas-Kanade tracking through every frame with a forward/backward check."""
    history = np.full((len(grays), len(starts), 2), np.nan, np.float32)
    history[0] = starts
    live = np.ones(len(starts), bool)
    points = starts.copy()
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01))
    for step in range(1, len(grays)):
        ids = np.flatnonzero(live)
        if not len(ids):
            break
        previous = points[ids].reshape(-1, 1, 2)
        forward, status, _ = cv2.calcOpticalFlowPyrLK(
            grays[step - 1], grays[step], previous, None, **lk)
        if forward is None or status is None:
            break
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(
            grays[step], grays[step - 1], forward, None, **lk)
        if backward is None or back_status is None:
            break
        f = forward.reshape(-1, 2)
        back = backward.reshape(-1, 2)
        height, width = grays[step].shape
        keep = (status.ravel().astype(bool) & back_status.ravel().astype(bool)
                & np.isfinite(f).all(axis=1) & np.isfinite(back).all(axis=1)
                & (np.linalg.norm(back - previous.reshape(-1, 2), axis=1) <= 1.0)
                & (f[:, 0] >= 0) & (f[:, 0] < width)
                & (f[:, 1] >= 0) & (f[:, 1] < height))
        live[ids[~keep]] = False
        history[step, ids[keep]] = f[keep]
        points[ids[keep]] = f[keep]
    return history, np.isfinite(history).all(axis=(0, 2))


def temporal_scores(history, training, valid):
    """Model each intermediate view using ONLY the non-held-out tile tracks."""
    scores = np.full(history.shape[1], np.inf)
    supported = training & valid
    if supported.sum() < 4:
        return scores
    residuals = []
    for step in range(1, len(history)):
        cv2.setRNGSeed(566)
        h, _ = cv2.findHomography(
            history[0, supported], history[step, supported], cv2.LMEDS)
        if h is None or not np.isfinite(h).all():
            return scores
        residuals.append(errors(h, history[0, valid], history[step, valid]))
    scores[valid] = np.median(residuals, axis=0)
    return scores


def heldout_error(h, a, b, supported):
    if h is None or supported.sum() < 4:
        return None
    distances = errors(h, a[supported], b[supported])
    return float(np.median(distances)) if np.isfinite(distances).all() else None


def evaluate(grays, start, end, detector, matcher, minimum=20):
    knn, a, b = correspondences(grays[0], grays[-1], detector, matcher)
    n = len(a)
    result = dict(start=start, end=end, nominal_window=end - start,
                  knn_pairs=knn, sift_matches=n, raw_inliers=0,
                  raw_inlier_fraction=0.0, flow_incomplete=0,
                  endpoint_disagrees=0, temporal_rejected=0,
                  temporal_rejected_raw_inliers=0, kept=0, after_inliers=0,
                  after_inlier_fraction=0.0, holdout_candidates=0,
                  holdout_flow_supported=0, raw_median_px=None,
                  flow_only_median_px=None, prefiltered_median_px=None)
    if n < minimum:
        return result, None
    # Checkerboard holdout uses 40-pixel tiles in the FIRST image.
    train = ((np.floor(a[:, 0] / 40).astype(int) +
              np.floor(a[:, 1] / 40).astype(int)) % 2) == 0
    test = ~train
    result["holdout_candidates"] = int(test.sum())

    # Descriptive inlier fractions use full candidate populations.
    _, raw_inliers = fit(a, b, minimum)
    result["raw_inliers"] = int(raw_inliers.sum())
    result["raw_inlier_fraction"] = float(raw_inliers.mean())

    path, complete = track(grays, a)
    endpoint = complete & (np.linalg.norm(path[-1] - b, axis=1) <= 2.0)
    temporal = temporal_scores(path, train, endpoint)
    retained = endpoint & (temporal <= 1.5)

    result["flow_incomplete"] = int((~complete).sum())
    result["endpoint_disagrees"] = int((complete & ~endpoint).sum())
    result["temporal_rejected"] = int((endpoint & ~retained).sum())
    result["temporal_rejected_raw_inliers"] = int(
        (raw_inliers & endpoint & ~retained).sum())
    result["kept"] = int(retained.sum())
    _, accepted = fit(a[retained], b[retained], minimum)
    result["after_inliers"] = int(accepted.sum())
    result["after_inlier_fraction"] = (
        float(accepted.mean()) if len(accepted) else 0.0)

    # Score ALL three methods on identical, flow-supported HELD-OUT points.
    heldout = test & endpoint
    result["holdout_flow_supported"] = int(heldout.sum())
    comparisons = (("raw_median_px", train),
                   ("flow_only_median_px", train & endpoint),
                   ("prefiltered_median_px", train & retained))
    for key, fitting in comparisons:
        h, _ = fit(a[fitting], b[fitting], minimum)
        result[key] = heldout_error(h, a, b, heldout)

    details = dict(points=a, raw_inliers=raw_inliers, complete=complete,
                   endpoint=endpoint, retained=retained)
    return result, details


def median(rows, key):
    numbers = [r[key] for r in rows if r[key] is not None]
    return float(np.median(numbers)) if numbers else None


def plots(rows, example, output):
    five = [r for r in rows if r["nominal_window"] == 5]
    if five:
        x = [r["start"] for r in five]
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for field, title in (("raw_inlier_fraction", "SIFT only"),
                             ("after_inlier_fraction", "Flow + temporal filter")):
            axes[0].plot(x, [100 * r[field] for r in five], label=title)
        axes[0].set_ylabel("RANSAC inlier rate (%)")
        axes[0].legend()
        for field, title in (("raw_median_px", "SIFT only"),
                             ("flow_only_median_px", "Flow only"),
                             ("prefiltered_median_px", "Flow + temporal filter")):
            axes[1].plot(x, [r[field] if r[field] is not None else np.nan
                             for r in five], label=title)
        axes[1].set(xlabel="First frame of pair", ylabel="Held-out alignment error (px)")
        axes[1].legend()
        fig.tight_layout()
        fig.savefig(output / "173701_inlier_rate_and_alignment.png", dpi=160)
        plt.close(fig)
    if example is not None:
        gray, details = example
        ix = np.flatnonzero(details["raw_inliers"])
        points = details["points"][ix]
        colors = np.where(~details["complete"][ix], "red",
                          np.where(~details["endpoint"][ix], "orange",
                                   np.where(details["retained"][ix], "green", "purple")))
        fig, ax = plt.subplots(figsize=(9, 7))
        ax.imshow(gray, cmap="gray")
        ax.scatter(points[:, 0], points[:, 1], c=colors, s=5, alpha=0.7)
        ax.set_title("Frames 195–200: raw inliers after flow/temporal checks")
        ax.set_axis_off()
        fig.tight_layout()
        fig.savefig(output / "173701_match_decisions_frames_195_200.png", dpi=160)
        plt.close(fig)


def run(bag, outdir, gaps=(5, 25)):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    recording = ColorFrames(bag)
    detector = cv2.SIFT_create(nfeatures=10000)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    rows, example = [], None
    try:
        for gap in gaps:
            for start in range(0, len(recording) - 1, gap):
                end = min(start + gap, len(recording) - 1)
                # The archived 5-frame table omits a final incomplete interval.
                if gap == 5 and end - start < gap:
                    continue
                grays = [recording.gray(i) for i in range(start, end + 1)]
                row, detail = evaluate(grays, start, end, detector, matcher)
                row["nominal_window"] = gap
                rows.append(row)
                print(f"{start}–{end}: {row['sift_matches']} matches; "
                      f"{row['raw_inlier_fraction']:.1%} -> "
                      f"{row['after_inlier_fraction']:.1%}", flush=True)
                if gap == 5 and start == 195 and detail is not None:
                    example = (grays[0].copy(), detail)
    finally:
        recording.close()

    with (outdir / "173701_window_results.csv").open(
            "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    plots(rows, example, outdir)
    summary = {"recording": Path(bag).name,
               "method": "independent reimplementation, not the original script",
               "archived_result_reproduced": False,
               "by_window": {}}
    for gap in gaps:
        group = [r for r in rows if r["nominal_window"] == gap]
        scored = [r for r in group if r["prefiltered_median_px"] is not None]
        summary["by_window"][str(gap)] = {
            "pairs": len(group), "fully_scored": len(scored),
            "raw_inlier_rate_median": median(group, "raw_inlier_fraction"),
            "filtered_inlier_rate_median": median(group, "after_inlier_fraction"),
            "raw_alignment_px_median": median(scored, "raw_median_px"),
            "flow_alignment_px_median": median(scored, "flow_only_median_px"),
            "filtered_alignment_px_median": median(scored, "prefiltered_median_px")}
    (outdir / "173701_reproduction_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--gaps", type=int, nargs="+", default=[5, 25])
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error("recording not found; provide the actual .db3 path")
    if not args.gaps or any(g < 2 for g in args.gaps):
        parser.error("frame gaps must be at least 2")
    print(json.dumps(run(args.bag, args.outdir, args.gaps), indent=2))


if __name__ == "__main__":
    main()
