import cv2
import numpy as np
import pytest
from sfm_keyframes import assess_two_view

K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])


def pair(tx=0., rotation_deg=0.):
    rng = np.random.default_rng(13)
    world = rng.uniform([-2, -1.5, 4], [2, 1.5, 8], (250, 3))
    R = cv2.Rodrigues(np.array([0., np.deg2rad(rotation_deg), 0.]))[0]
    def project(points):
        pixels = points @ K.T
        return pixels[:, :2] / pixels[:, 2:]
    return project(world), project(world @ R.T + [tx, 0, 0])


@pytest.mark.parametrize('rotation_deg', [0., 8., 25.])
def test_nonzero_rotation_alone_is_not_a_triangulation_baseline(rotation_deg):
    src, dst = pair(rotation_deg=rotation_deg)
    result = assess_two_view(src, dst, K)
    assert not result['eligible']
    assert result['plane_check'] == 'supported'
    assert result['reason'] == 'rotation_only_explains_matches'


def test_rotation_only_with_small_measurement_noise_is_rejected():
    src, dst = pair(rotation_deg=8)
    rng = np.random.default_rng(21)
    result = assess_two_view(src + rng.normal(0, .15, src.shape),
                             dst + rng.normal(0, .15, dst.shape), K)
    assert not result['eligible']
    assert result['reason'] == 'rotation_only_explains_matches'


def test_rotation_only_with_mismatches_is_rejected():
    src, dst = pair(rotation_deg=8)
    dst[:30] = np.random.default_rng(12).uniform([0, 0], [640, 480], (30, 2))
    result = assess_two_view(src, dst, K)
    assert not result['eligible']
    assert result['reason'] == 'rotation_only_explains_matches'


def test_narrow_baseline_is_held_until_frames_are_farther_apart():
    src, dst = pair(tx=.01)
    assert not assess_two_view(src, dst, K)['eligible']
    src, dst = pair(tx=.6, rotation_deg=3)
    result = assess_two_view(src, dst, K)
    assert result['eligible']
    assert result['median_parallax_deg'] > 2
    assert result['translation_scale'] == 'unknown'
