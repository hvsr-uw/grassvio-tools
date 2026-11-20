import importlib.util
from pathlib import Path
import cv2
import numpy as np

P = Path(__file__).resolve().parents[1] / 'stitch-map' / 'heldout_diagnostics.py'
spec = importlib.util.spec_from_file_location('heldout_diagnostics', P)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def scorer(history, models, threshold):
    residual = np.linalg.norm(history[-1] - history[0] - np.array([5, 2]), axis=1)
    return residual < threshold, residual


def fitter(src, dst, min_inliers):
    if len(src) < min_inliers:
        return None, None
    matrix, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    return matrix, mask


def project(H, points):
    p = np.c_[points, np.ones(len(points))] @ H.T
    return p[:, :2] / p[:, 2:]


def history_for_test():
    rng = np.random.default_rng(4)
    pts = rng.uniform([0, 0], [400, 300], (400, 2))
    final = pts + [5, 2]
    # A local moving patch creates a second motion field and cannot be
    # treated as an independently measured camera pose.
    patch = (pts[:, 0] > 100) & (pts[:, 0] < 300) & (pts[:, 1] > 50) & (pts[:, 1] < 250)
    final[patch] += [8, -3]
    return np.stack([pts, final])


def test_spatial_heldout_uses_only_training_tracks():
    h = history_for_test()
    seen = []
    def spy(tracks, models, threshold):
        seen.append(tracks.shape[1])
        return scorer(tracks, models, threshold)
    r = mod.evaluate_heldout_window(h, None, 1.5, spy, fitter, project)
    assert r['heldout_status'] == 'scored'
    assert r['heldout_test_tracks'] >= 20
    assert seen == [r['heldout_train_tracks']]
    assert r['heldout_train_tracks'] + r['heldout_test_tracks'] == len(h[0])
    assert r['heldout_train_kept'] < r['heldout_train_tracks']
    assert r['heldout_median_error_px'] is not None


def test_too_few_tracks_does_not_claim_improvement():
    h=np.zeros((3, 8, 2))
    r=mod.evaluate_heldout_window(h, None, 1.5, scorer, fitter, project)
    assert r['heldout_status'].startswith('too_few')
    assert r['heldout_median_error_px'] is None


def test_nonfinite_history_is_rejected():
    h=np.zeros((3, 50, 2));h[1, 10] = [np.nan, 1]
    try:
        mod.evaluate_heldout_window(h, None, 1.5, scorer, fitter, project)
        assert False
    except ValueError:
        pass
