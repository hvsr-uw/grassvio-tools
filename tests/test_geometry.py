import cv2
import numpy as np
import pytest
from geometry import apply_transform, fit_homography, projected_corners, render_mosaic


def test_normalized_threshold_remains_three_pixels():
    rng = np.random.default_rng(42)
    src = rng.uniform([0, 0], [640, 480], (160, 2))
    H = np.array([[1.01, .02, 13], [-.01, .99, -8], [1e-5, -2e-5, 1.]])
    dst = apply_transform(H, src) + rng.normal(0, .15, src.shape)
    dst[100:] = rng.uniform([0, 0], [640, 480], (60, 2))
    for normalize in (True, False):
        cv2.setRNGSeed(0)
        estimate, mask = fit_homography(src, dst, normalize=normalize, min_inliers=20)
        assert 98 <= mask.sum() <= 102
        assert np.linalg.norm(apply_transform(estimate, src[:100]) - apply_transform(H, src[:100]), axis=1).mean() < .2
        errors = np.linalg.norm(apply_transform(estimate, src) - dst, axis=1)
        assert np.all(errors[mask.ravel().astype(bool)] <= 3)


@pytest.mark.parametrize('points', [np.zeros((25, 2)), np.column_stack((np.arange(25), np.arange(25)))])
def test_degenerate_points_are_rejected(points):
    assert fit_homography(points, points) == (None, None)


def test_horizon_inside_image_is_rejected():
    H = np.array([[1., 0, 0], [0, 1, 0], [-.01, 0, 1]])
    with pytest.raises(ValueError, match='horizon'):
        projected_corners(H, (200, 200))


def test_canvas_budget_checked_before_warp(monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError('allocation/warp happened before canvas check')
    monkeypatch.setattr(cv2, 'warpPerspective', unexpected)
    frame = np.zeros((20, 20, 3), np.uint8)
    with pytest.raises(ValueError, match='budget'):
        render_mosaic([frame, frame], [np.eye(3), np.diag([1e5, 1e5, 1])])


def test_unplaced_images_are_omitted_and_black_is_valid():
    black = np.zeros((20, 20, 3), np.uint8)
    white = np.full_like(black, 200)
    result, stats = render_mosaic([black, white, white], [np.eye(3), np.eye(3), None])
    assert np.all(result == 100)
    assert stats['coverage_fraction'] == 1


def test_fractional_border_does_not_double_multiply_mask():
    frame = np.full((20, 20, 3), 120, np.uint8)
    H = np.array([[1., 0, .5], [0, 1, .5], [0, 0, 1.]])
    image, _ = render_mosaic([frame], [H])
    assert np.all(image == 120)


def test_best_blending_preserves_texture_instead_of_averaging():
    a = np.full((20, 20, 3), 20, np.uint8)
    b = np.full_like(a, 200)
    best, _ = render_mosaic([a, b], [np.eye(3), np.eye(3)], blend='best')
    assert np.all(best == 20)
