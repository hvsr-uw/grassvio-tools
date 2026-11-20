import numpy as np
import pytest

from semantic_cues import sample_likelihood, semantic_motion_keep, vegetation_likelihood


def test_vegetation_likelihood_is_high_for_green_pixels_and_low_for_gray():
    image = np.zeros((20, 30, 3), np.uint8)
    image[:, :15] = (30, 150, 50)  # BGR green
    image[:, 15:] = (100, 100, 100)
    score = vegetation_likelihood(image, blur_radius=1)
    assert float(score[:, :15].mean()) > 0.2
    assert float(score[:, 15:].max()) == 0.0


def test_semantic_cue_rejects_dynamic_vegetation_but_keeps_rigid_green_points():
    keep, dynamic = semantic_motion_keep(
        [0.9, 0.9, 0.1], [4.0, 0.2, 4.0], residual_threshold_px=1.5
    )
    np.testing.assert_array_equal(keep, [False, True, True])
    assert dynamic[0] > dynamic[1]


def test_semantic_cue_keeps_nonvegetation_even_with_large_residual():
    keep, _ = semantic_motion_keep([0.0], [100.0])
    assert keep.tolist() == [True]


def test_sample_likelihood_handles_out_of_bounds_points():
    likelihood = np.arange(9, dtype=np.float32).reshape(3, 3)
    result = sample_likelihood(likelihood, [[0, 0], [2, 2], [-1, 1], [4, 1]])
    np.testing.assert_allclose(result, [0, 8, 0, 0])


def test_vegetation_likelihood_validates_inputs():
    with pytest.raises(ValueError, match="HxW"):
        vegetation_likelihood(np.zeros((5, 5), np.uint8))
    with pytest.raises(ValueError, match="odd"):
        vegetation_likelihood(np.zeros((5, 5, 3), np.uint8), blur_radius=2)
