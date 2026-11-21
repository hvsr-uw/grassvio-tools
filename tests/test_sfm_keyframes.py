import cv2
import numpy as np
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


def test_nonzero_rotation_alone_is_not_a_triangulation_baseline():
    src, dst = pair(rotation_deg=8)
    result = assess_two_view(src, dst, K)
    assert not result['eligible']
    assert result['plane_check'] == 'supported'


def test_narrow_baseline_is_held_until_frames_are_farther_apart():
    src, dst = pair(tx=.01)
    assert not assess_two_view(src, dst, K)['eligible']
    src, dst = pair(tx=.6, rotation_deg=3)
    result = assess_two_view(src, dst, K)
    assert result['eligible']
    assert result['median_parallax_deg'] > 2
    assert result['translation_scale'] == 'unknown'
