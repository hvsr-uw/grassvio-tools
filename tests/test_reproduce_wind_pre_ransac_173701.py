"""Synthetic checks for the real-recording pre-RANSAC evaluation runner."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np


SPEC = importlib.util.spec_from_file_location(
    "wind_reproduction",
    Path(__file__).resolve().parents[1] / "scripts" / "reproduce_wind_pre_ransac_173701.py")
wind = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wind)


def test_low_match_count_is_explicit_failure():
    h, mask = wind.fit(np.empty((0, 2), np.float32),
                       np.empty((0, 2), np.float32))
    assert h is None
    assert mask.size == 0


def test_temporal_filter_detects_nonrigid_oscillation():
    rng = np.random.default_rng(566)
    start = rng.uniform(20, 200, size=(120, 2)).astype(np.float32)
    tracks = np.repeat(start[None], 6, axis=0)
    for k in range(6):
        tracks[k, 90:, 0] += 7 * np.sin(np.pi * k / 5)
    scores = wind.temporal_scores(tracks, np.ones(120, bool),
                                  np.ones(120, bool))
    assert np.median(scores[:90]) < 1.5
    assert np.median(scores[90:]) > 1.5


def test_pair_evaluation_on_known_rigid_motion():
    rng = np.random.default_rng(20)
    original = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    frames = [cv2.warpAffine(
        original, np.float32([[1, 0, k * .2], [0, 1, k * .1]]),
        (320, 240)) for k in range(6)]
    row, detail = wind.evaluate(
        frames, 0, 5, cv2.SIFT_create(nfeatures=2000),
        cv2.BFMatcher(cv2.NORM_L2))
    assert detail is not None
    assert row["sift_matches"] > 30
    assert row["raw_inlier_fraction"] > .9
    assert row["holdout_flow_supported"] > 10
    assert row["raw_median_px"] is not None
    assert row["prefiltered_median_px"] is not None
    assert set(row) == set(wind.COLUMNS)
