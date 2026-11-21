import importlib.util
from pathlib import Path
import cv2
import numpy as np
import pytest
import stitch_map
import stable_homography


def frames():
    rng = np.random.default_rng(8)
    image = cv2.GaussianBlur(rng.integers(0, 256, (180, 240, 3), dtype=np.uint8), (3, 3), 0)
    return [image.copy() for _ in range(3)]


def test_descriptor_chain_skips_bad_frame_and_recovers_from_last_placed(monkeypatch):
    calls = []
    def match(*args, **kwargs):
        calls.append(1)
        return (None, 0) if len(calls) == 1 else (np.eye(3), 100)
    monkeypatch.setattr(stitch_map, 'pair_homography', match)
    image, stats = stitch_map.stitch(frames(), 1, 'sift')
    assert stats['frames_unplaced'] == [1]
    assert stats['frames_used'] == 2
    assert stats['pairs'][1]['from_frame'] == 0
    assert stats['transforms_to_first'][1] is None


def test_failed_flow_window_does_not_invent_an_absolute_bridge(monkeypatch):
    def pairs(*args, **kwargs):
        yield 0, 1, None, {'inliers': 0}
        yield 1, 2, np.eye(3), {'inliers': 100}
    monkeypatch.setattr(stitch_map, 'flow_pairs', pairs)
    with pytest.raises(ValueError, match='fewer than two'):
        stitch_map.stitch(frames(), 1, 'sift', motion_filter=True)


def test_both_entrypoints_run_same_implementation():
    path = Path(__file__).resolve().parents[1] / 'dataset-tools/stitch-map/stitch_map.py'
    spec = importlib.util.spec_from_file_location('compat_stitch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    a, sa = stitch_map.stitch(frames(), 1, 'sift')
    b, sb = module.stitch(frames(), 1, 'sift')
    np.testing.assert_array_equal(a, b)
    assert sa == sb


def test_checkpoint_cannot_be_reused_for_different_content(tmp_path):
    path = str(tmp_path / 'pairs.json')
    _, first = stitch_map.stitch(frames(), 1, 'sift', place='tree', pairs_checkpoint=path)
    _, second = stitch_map.stitch(frames(), 1, 'sift', place='tree', pairs_checkpoint=path)
    assert first['pairs_from_checkpoint'] == 0
    assert second['pairs_from_checkpoint'] == 3
    altered = [np.flip(f, 1).copy() for f in frames()]
    _, third = stitch_map.stitch(altered, 1, 'sift', place='tree', pairs_checkpoint=path)
    assert third['pairs_from_checkpoint'] == 0


def test_window_mode_writes_originals_beside_map(tmp_path):
    debug = tmp_path / 'debug.png'
    image, stats = stitch_map.stitch(frames()*10, 5, 'sift', motion_filter=True,
                                    camera_still=True, debug_out=str(debug), blend='best')
    assert stats['selected_frames'] == [0, 25, 29]
    assert stats['frames_used'] == 3
    assert debug.exists()
    assert cv2.imread(str(debug)).shape[1] > 320


def test_stable_normalization_does_not_accept_pixel_outliers():
    # Directly exercise the entry point that previously used 3 normalized
    # units (hundreds of pixels) rather than a 3-pixel threshold.
    rng = np.random.default_rng(9)
    pts = rng.uniform([0, 0], [640, 480], (140, 2))
    target = pts + [10, -5]
    target[90:] = rng.uniform([0, 0], [640, 480], (50, 2))
    kp_b = [cv2.KeyPoint(float(x), float(y), 1) for x, y in pts]
    kp_a = [cv2.KeyPoint(float(x), float(y), 1) for x, y in target]
    class Matcher:
        def knnMatch(self, *args, **kwargs):
            return [[cv2.DMatch(i, i, 0.), cv2.DMatch(i, (i+1)%140, 10.)] for i in range(140)]
    H, inliers = stable_homography.match_h(kp_a, np.ones((140, 1)), kp_b, np.ones((140, 1)), Matcher(), True)
    assert H is not None
    assert inliers == 90


def test_endpoint_reanchor_composes_actual_matched_frames():
    relative = [np.array([[1., 0, k], [0, 1, 0], [0, 0, 1]]) for k in (10, 20, 30, 40)]
    poses = [np.eye(3)]
    for H in relative:
        poses.append(poses[-1] @ H)
    measured = np.array([[1., 0, 7], [0, 1, 0], [0, 0, 1]])
    corrected = stable_homography.reanchor_chain(poses, relative, 1, 3, measured)
    assert corrected[3][0, 2] == 17  # frame 1 offset + measured 3->1
    assert corrected[4][0, 2] == 57  # use the actual pair after frame 3
    assert corrected[2][0, 2] == 30  # no fabricated intermediate correction


def test_short_sequence_without_loop_writes_normalized_only(monkeypatch, tmp_path):
    import json
    import sys
    fs = frames()
    monkeypatch.setattr(stable_homography, 'load_frames', lambda *args: (fs, [(None, None)]*3))
    monkeypatch.setattr(stable_homography, 'match_h', lambda *args: (np.eye(3), 30))
    prefix = str(tmp_path/'short')
    monkeypatch.setattr(sys, 'argv', ['stable_homography.py', '--bag', 'unused.db3', '--out', prefix])
    stable_homography.main()
    record = json.loads((tmp_path/'short_stable.json').read_text())
    assert record['best_loop'] is None
    assert record['global_drift_corrected'] is False
    assert (tmp_path/'short_norm_only.png').exists()
    assert not (tmp_path/'short_loop_reanchored.png').exists()


def test_stable_failed_pair_returns_failure_status(monkeypatch, tmp_path):
    import sys
    monkeypatch.setattr(stable_homography, 'load_frames',
                        lambda *args: (frames(), [(None, None)]*3))
    monkeypatch.setattr(stable_homography, 'match_h', lambda *args: (None, 0))
    monkeypatch.setattr(sys, 'argv', ['stable_homography.py', '--bag', 'unused.db3',
                                     '--out', str(tmp_path/'failed')])
    assert stable_homography.main() == 1
    assert not (tmp_path/'failed_norm_only.png').exists()
