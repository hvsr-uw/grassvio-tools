import cv2
import numpy as np
import pytest
from geometry import apply_transform
from window_motion import temporal_filter, flow_pairs, validate_camera_motion


def test_window_catches_flutter_even_with_identical_endpoints():
    rng = np.random.default_rng(3)
    start = rng.uniform(30, 150, (100, 2))
    history = np.repeat(start[None], 26, axis=0)
    for k in range(26):
        history[k, 60:, 0] += 8 * np.sin(2*np.pi*k/25)
    keep, scores = temporal_filter(history, np.repeat(np.eye(3)[None], 26, axis=0))
    assert np.all(keep[:60])
    assert not np.any(keep[60:])
    np.testing.assert_allclose(history[-1], history[0])
    assert np.median(scores[60:]) > 4


def test_complete_camera_prior_preserves_rigid_motion_and_rejects_local_motion():
    rng = np.random.default_rng(4)
    start = rng.uniform(20, 150, (100, 2))
    models = np.array([[[1., 0, k*.8], [0, 1, k*.3], [0, 0, 1]] for k in range(26)])
    history = np.array([apply_transform(H, start) for H in models])
    history[:, 80:, 1] += (5*np.sin(np.arange(26)/4))[:, None]
    keep, _ = temporal_filter(history, models)
    assert np.all(keep[:80])
    assert not np.any(keep[80:])


def test_visual_only_fallback_is_not_a_stationary_camera_claim():
    rng = np.random.default_rng(5)
    gray = cv2.GaussianBlur(rng.integers(0, 256, (160, 220), dtype=np.uint8), (3, 3), 0)
    result = list(flow_pairs([gray]*8, window=5))
    assert [(r[0], r[1]) for r in result] == [(0, 5), (5, 7)]
    assert all(r[2] is not None for r in result)
    assert all(r[3]['camera_evidence'] == 'visual_plane_only' for r in result)
    assert all(not r[3]['wind_cause_verified'] for r in result)


def test_empty_window_fails_without_unfiltered_retry():
    results = list(flow_pairs([np.zeros((100, 100), np.uint8)] * 26, camera_still=True))
    assert results[0][2] is None
    assert results[0][3]['prefilter_kept'] == 0


def test_rotation_only_prior_is_rejected():
    with pytest.raises(ValueError, match='translation'):
        validate_camera_motion({'translation_compensated': False, 'image_transforms': [np.eye(3).tolist()]}, 1)


def test_stationary_mode_tracks_real_pixels():
    rng = np.random.default_rng(6)
    gray = rng.integers(0, 256, (180, 240), dtype=np.uint8)
    result = list(flow_pairs([gray]*26, camera_still=True))[0]
    assert result[3]['complete_tracks'] >= 100
    assert result[3]['prefilter_rejected'] == 0
    np.testing.assert_allclose(result[2], np.eye(3), atol=1e-5)


def test_flow_pairs_can_apply_semantic_cues_without_claiming_wind_causality():
    rng = np.random.default_rng(8)
    gray = rng.integers(0, 256, (120, 160), dtype=np.uint8)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    result = list(flow_pairs([gray] * 8, window=5, semantic_images=[bgr] * 8))
    assert len(result) == 2
    assert all("semantic_rejected" in row[3] for row in result)
    assert all(not row[3]["wind_cause_verified"] for row in result)


def test_flow_pairs_rejects_mismatched_semantic_sequence():
    gray = np.zeros((40, 40), dtype=np.uint8)
    with pytest.raises(ValueError, match="semantic image sequence"):
        list(flow_pairs([gray] * 3, semantic_images=[cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)]))


def test_semantic_filter_can_actually_change_flow_selection(monkeypatch):
    import window_motion
    # One point moves 1.3 px: passes the ordinary 1.5 px gate but should
    # fail the vegetation-specific 1.125 px gate. A rigid green point stays.
    xy = np.array([[10., 10.], [20., 10.], [30., 10.], [10., 20.],
                   [20., 20.], [30., 20.]])
    history = np.repeat(xy[None], 3, axis=0)
    history[1:, 0, 0] += 1.3
    monkeypatch.setattr(window_motion, 'track_window', lambda _: (history, len(xy)))
    gray = np.zeros((40, 40), np.uint8)
    green = np.full((40, 40, 3), (0, 255, 0), np.uint8)
    ordinary = list(flow_pairs([gray]*3, camera_still=True, min_inliers=4))[0][3]
    semantic = list(flow_pairs([gray]*3, camera_still=True, min_inliers=4,
        semantic_images=[green]*3))[0][3]
    assert ordinary['prefilter_kept'] == 6
    assert semantic['prefilter_kept'] == 5
    assert semantic['semantic_rejected'] == 1
