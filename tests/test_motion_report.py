import cv2
import numpy as np

from motion_report import relative_motion, temporal_mean, rotation_vectors_for_plot


def test_relative_translation_has_pnp_direction_and_target_coordinates():
    rotation = cv2.Rodrigues(np.array([0., 0., np.pi/2]))[0]
    r, t = relative_motion((np.eye(3), np.zeros(3)), (rotation, np.array([1., 0., 0.])))
    np.testing.assert_allclose(r, rotation.T)
    np.testing.assert_allclose(t, [0., 1., 0.], atol=1e-12)
    # A fixed world landmark must map to exactly its target camera coordinates.
    point = np.array([2., 1., 4.])
    np.testing.assert_allclose(r@point+t, rotation.T@(point-[1., 0., 0.]))


def test_imu_mean_is_temporal_and_does_not_mix_sensor_axes():
    values = np.column_stack([np.arange(9, dtype=float), np.full(9, 10.), np.zeros(9)])
    mean = temporal_mean(values)
    np.testing.assert_allclose(mean[2:-2, 0], values[2:-2, 0])
    np.testing.assert_allclose(mean[:, 1], 10.)
    np.testing.assert_allclose(mean[:, 2], 0.)


def test_rotation_plot_does_not_turn_the_pi_branch_cut_into_a_motion_jump():
    angles = np.linspace(0., 2*np.pi, 101)
    rotations = np.array([cv2.Rodrigues(np.array([0., 0., a]))[0] for a in angles])
    displayed = rotation_vectors_for_plot(rotations)
    assert np.isnan(displayed).any()
    finite_pairs = np.isfinite(displayed[:-1, 2]) & np.isfinite(displayed[1:, 2])
    assert np.max(np.abs(np.diff(displayed[:, 2])[finite_pairs])) < 4.
    for vector, rotation in zip(displayed, rotations):
        if np.isfinite(vector).all():
            np.testing.assert_allclose(cv2.Rodrigues(np.radians(vector))[0], rotation, atol=1e-7)
