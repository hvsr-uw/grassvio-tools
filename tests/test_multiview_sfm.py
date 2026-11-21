import cv2
import numpy as np

from multiview_sfm import TrackUnion, bundle_adjust, reconstruct, triangulate


def synthetic_scene():
    rng = np.random.default_rng(566)
    xyz = rng.uniform([-1., -.7, 3.], [1., .7, 6.], (100, 3))
    poses = {}
    features = []
    for i in range(5):
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(np.array([.01*i, -.015*i, .003*i]))[0]
        pose[:3, 3] = [-.17*i, -.03*i, .01*i]
        poses[i] = pose
        camera = xyz@pose[:3, :3].T+pose[:3, 3]
        features.append({'xy': camera[:, :2]/camera[:, 2:]})
    tracks = [{i: tid for i in poses} for tid in range(len(xyz))]
    return xyz, poses, features, tracks


def test_track_union_rejects_conflicting_observations_in_one_camera():
    union = TrackUnion()
    assert union.merge((0, 0), (1, 0))
    assert union.merge((1, 0), (2, 0))
    assert not union.merge((2, 0), (0, 1))
    assert len(union.members[union.find((0, 0))]) == 3


def test_triangulation_rejects_pure_rotation_and_recovers_baseline():
    xyz, poses, features, tracks = synthetic_scene()
    point = triangulate(tracks[0], features, poses, 600.)
    np.testing.assert_allclose(point, xyz[0], atol=1e-8)
    still = {0: np.eye(4), 1: np.eye(4)}
    stationary_features = [{'xy': features[0]['xy']}, {'xy': features[0]['xy']}]
    assert triangulate({0: 0, 1: 0}, stationary_features, still, 600.) is None


def test_bundle_adjustment_reduces_error_and_preserves_world_and_scale_gauges():
    xyz, poses, features, tracks = synthetic_scene()
    rng = np.random.default_rng(9)
    perturbed = {i: pose.copy() for i, pose in poses.items()}
    for i in range(2, 5):
        perturbed[i][:3, 3] += rng.normal(0, .015, 3)
    points = {i: p+rng.normal(0, .025, 3) for i, p in enumerate(xyz)}
    refined, _, info = bundle_adjust(perturbed, points, tracks, features, 1, 600., max_nfev=80)
    assert info['after_median_px'] < info['before_median_px']*.05
    np.testing.assert_allclose(refined[0], poses[0], atol=1e-10)
    assert abs(refined[1][0, 3]-poses[1][0, 3]) < 1e-12


def test_incremental_reconstruction_uses_multiple_views_and_scores_reserved_landmarks():
    _, true_poses, features, tracks = synthetic_scene()
    poses, points, info = reconstruct(features, tracks, 600., max_points=200)
    assert len(poses) == 5
    assert len(points) >= 50
    assert info['validation_observations'] >= 20
    assert info['validation_median_px'] < .05
    # Translation is determined only up to one common visual scale.
    scale = np.linalg.norm(poses[4][:3, 3])/np.linalg.norm(true_poses[4][:3, 3])
    np.testing.assert_allclose(poses[2][:3, 3]/scale, true_poses[2][:3, 3], atol=.002)
