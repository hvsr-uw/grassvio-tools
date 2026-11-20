import importlib.util
from pathlib import Path

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'check_recording_loops.py'
spec = importlib.util.spec_from_file_location('check_recording_loops', PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def matches():
    source = np.random.default_rng(31).uniform([0, 0], [640, 480], (100, 2))
    return source, source + [5., -3.]


def test_direct_loop_does_not_hide_a_missing_chained_endpoint():
    result = mod.evaluate_loop_points(*matches(), None)
    assert result['direct_inliers'] == 100
    assert result['direct_fit_median_error_px'] < 1e-4
    assert result['chain_endpoint_available'] is False
    assert result['chain_loop_median_error_px'] is None


def test_loop_reports_chained_drift_separately_from_direct_fit():
    shifted = np.array([[1.,0,15.],[0,1,-3.],[0,0,1.]])
    result = mod.evaluate_loop_points(*matches(), shifted)
    assert result['direct_fit_median_error_px'] < 1e-4
    assert result['chain_endpoint_available'] is True
    assert abs(result['chain_loop_median_error_px'] - 10.) < 1e-6


def test_good_direct_matches_do_not_imply_renderable_full_frame():
    source = np.random.default_rng(32).uniform([50,300],[590,470],(100,2))
    h = np.array([[1.,0,0],[0,1,0],[0,-1./200,1.]])
    target = mod.apply_transform(h, source)
    result = mod.evaluate_loop_points(source, target, None)
    assert result['direct_inliers'] == 100
    assert result['direct_fit_median_error_px'] < 1e-4
    footprint = mod.check_direct_footprint(result, (480,640,3))
    assert footprint['direct_full_frame_footprint_valid'] is False
    assert footprint['direct_footprint_error'] == 'projective horizon crosses the frame'
