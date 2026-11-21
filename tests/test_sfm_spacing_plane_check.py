import cv2
import numpy as np

from sfm_spacing_plane_check import (compare_matches, epipolar_errors,
                                     relative_sfm_pose, spatial_holdout,
                                     summarize)


def synthetic_matches(depth_range=(4., 4.), tx=.5, rotation_deg=3.):
    rng = np.random.default_rng(566)
    xyz = rng.uniform([-1.7, -1.1, depth_range[0]],
                      [1.7, 1.1, depth_range[1]], (300, 3))
    rotation = cv2.Rodrigues(np.array(
        [0., np.deg2rad(rotation_deg), 0.]))[0]
    camera = xyz @ rotation.T + [tx, 0., 0.]
    focal = 600.
    source = focal * xyz[:, :2] / xyz[:, 2:]
    target = focal * camera[:, :2] / camera[:, 2:]
    raw = source + [320., 240.]
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = [tx, 0., 0.]
    return source, target, raw, focal, pose


def test_spatial_split_is_deterministic_and_keeps_separate_tiles():
    source, _, raw, _, _ = synthetic_matches()
    selected = spatial_holdout(raw, 640, 480)
    np.testing.assert_array_equal(selected, spatial_holdout(raw, 640, 480))
    assert 25 < selected.sum() < 100


def test_flat_grass_plane_homography_predicts_reserved_matches():
    source, target, raw, focal, pose = synthetic_matches()
    row = compare_matches(source, target, raw, (640, 480), focal,
                          relative_pose=pose)
    assert row["status"] == "evaluated"
    assert row["plane_check"] == "strong"
    assert row["plane_inlier_fraction"] > .99
    assert row["plane_holdout_median_px"] < 1e-5
    assert row["sfm_holdout_sampson_median_px"] < 1e-6


def test_nonplanar_scene_does_not_fit_one_plane_even_with_correct_sfm_pose():
    source, target, raw, focal, pose = synthetic_matches((3., 7.), tx=.65)
    row = compare_matches(source, target, raw, (640, 480), focal,
                          relative_pose=pose)
    assert row["status"] == "evaluated"
    assert row["plane_holdout_median_px"] > 1.
    assert row["sfm_holdout_sampson_median_px"] < 1e-6


def test_short_translation_is_not_relabelled_as_triangulation_baseline():
    source, target, raw, focal, _ = synthetic_matches((3., 7.), tx=.005,
                                                        rotation_deg=0.)
    row = compare_matches(source, target, raw, (640, 480), focal)
    assert row["status"] == "evaluated"
    assert not row["eligible"]


def test_pure_camera_rotation_does_not_count_as_baseline():
    source, target, raw, focal, _ = synthetic_matches(
        (3., 7.), tx=0., rotation_deg=9.)
    row = compare_matches(source, target, raw, (640, 480), focal)
    assert row["status"] == "evaluated"
    assert not row["eligible"]
    assert row["reason"] == "rotation_only_explains_matches"


def test_insufficient_matches_stay_unscored():
    source, target, raw, focal, _ = synthetic_matches()
    row = compare_matches(source[:15], target[:15], raw[:15],
                          (640, 480), focal)
    assert row["status"] == "insufficient_matches"
    assert "plane_holdout_median_px" not in row


def test_sfm_relative_camera_pose_and_pure_rotation_error():
    a = np.eye(4)
    b = np.eye(4)
    b[0, 3] = 2
    relative = relative_sfm_pose({10: a, 25: b}, 10, 25)
    np.testing.assert_allclose(relative[:3, 3], [-2., 0., 0.])
    assert relative_sfm_pose({10: a}, 10, 25) is None
    points = np.array([[1., 2.], [3., 4.]])
    assert epipolar_errors(points, points, a, 600.) is None


def test_summary_counts_supported_pairs_without_inventing_missing_results():
    rows = [
        {"gap_frames": 5, "status": "insufficient_matches"},
        {"gap_frames": 5, "status": "evaluated", "eligible": False,
         "plane_check": "strong", "plane_holdout_median_px": .5},
        {"gap_frames": 25, "status": "evaluated", "eligible": True,
         "plane_check": "weak_or_nonplanar", "plane_holdout_median_px": 3.,
         "median_parallax_deg": 3.2, "sfm_holdout_sampson_median_px": .4},
    ]
    result = summarize(rows, [5, 25, 50])
    assert result["5"]["attempted_pairs"] == 2
    assert result["5"]["evaluated_pairs"] == 1
    assert result["5"]["median_parallax_deg"] is None
    assert result["25"]["parallax_eligible_pairs"] == 1
    assert result["25"]["sfm_pose_pairs"] == 1
    assert result["50"]["evaluated_pairs"] == 0
    assert result["50"]["median_plane_holdout_px"] is None
