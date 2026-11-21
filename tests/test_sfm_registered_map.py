import numpy as np
import pytest

from sfm_registered_map import poses_for_map


def sample_reconstruction():
    pose = np.eye(4)
    pose[0, 3] = .2
    return {'pose_units': 'metres_from_initial_depth', 'metric_scale': {'usable': True},
            'selected_frames': [0, 25, 50], 'registered_frames': [0, 50],
            'poses_camera_to_world': [np.eye(4).tolist(), pose.tolist()]}


def test_visual_map_preserves_supported_poses_and_marks_missing_views_unconnected():
    channels = poses_for_map(sample_reconstruction())
    assert channels['rgbd_component_ids'] == [0, -1, 0]
    assert channels['rgbd_poses_to_component'][2][0][3] == .2
    assert channels['camera_pose_source'] == 'image_only_multiview_SfM_initial_depth_scale'
    assert channels['metric_ground_truth_available'] is False


def test_visual_map_rejects_unknown_scale_or_wrong_world_anchor():
    sfm = sample_reconstruction()
    sfm['metric_scale']['usable'] = False
    with pytest.raises(ValueError, match='metric scale'):
        poses_for_map(sfm)
    sfm = sample_reconstruction()
    sfm['poses_camera_to_world'][0][0][3] = 1.
    with pytest.raises(ValueError, match='fixed world frame'):
        poses_for_map(sfm)
