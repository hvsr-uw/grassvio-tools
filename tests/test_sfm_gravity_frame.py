import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from export_sfm_gravity_frame import export, world_from_camera0


def test_gravity_basis_height_and_relative_poses():
    gravity = np.array([0.0, 0.6, 0.8])
    world_from_camera = world_from_camera0(gravity, 0.75)
    np.testing.assert_allclose(world_from_camera @ [0, 0, 0, 1], [0, 0, .70, 1])
    np.testing.assert_allclose(world_from_camera @ [0, 0, .75, 1], [.45, 0, .1, 1])
    np.testing.assert_allclose(world_from_camera @ [*gravity, 0], [0, 0, -1, 0])
    np.testing.assert_allclose(np.linalg.det(world_from_camera[:3, :3]), 1)
    first = np.eye(4)
    later = np.array([[0, -1, 0, .02], [1, 0, 0, .03], [0, 0, 1, .04], [0, 0, 0, 1]])
    np.testing.assert_allclose(
        np.linalg.solve(world_from_camera @ later, world_from_camera @ first),
        np.linalg.solve(later, first),
        atol=1e-12,
    )


def test_rejects_undefined_optical_heading():
    with pytest.raises(ValueError, match="heading is undefined"):
        world_from_camera0([0, 0, 1], .75)


def test_export_skips_rgbd_poses_in_disconnected_component(tmp_path):
    identity = np.eye(4).tolist()
    recording = "clip.db3"
    sfm = tmp_path / "sfm.json"
    sfm.write_text(json.dumps({
        "recording": recording, "registered_frames": [0, 25, 50],
        "poses_camera_to_world": [identity] * 3, "pose_units": "metres_from_initial_depth",
        "metric_scale": {"usable": True},
    }), encoding="utf-8")
    rgbd = tmp_path / "channels.json"
    rgbd.write_text(json.dumps({
        "recording": recording, "rgbd_pose_indices": [0, 25, 50],
        "rgbd_component_ids": [0, 1, 0], "rgbd_poses_to_component": [identity] * 3,
    }), encoding="utf-8")
    structure = tmp_path / "structure.npz"
    np.savez(structure, points=[[0, 0, .75]])
    inputs = {"recording": recording, "gravity_down_camera0": [0, .6, .8],
              "mean_feature_depth_m": .75}
    export(sfm, structure, rgbd, inputs, tmp_path, rgbd_channels=True)
    result = json.loads((tmp_path / "world-frame.json").read_text(encoding="utf-8"))
    assert result["rgbd_poses_camera_to_world"][1] is None
    np.testing.assert_allclose(result["rgbd_poses_camera_to_world"][2], result["world_from_first_rgb_camera"])
    assert (tmp_path / "world-poses.csv").read_text(encoding="utf-8").splitlines()[2].endswith(",,,")
    assert (tmp_path / "world-points.ply").read_text(encoding="ascii").splitlines()[-1].startswith(
        "0.450000000 0.000000000 0.100000000"
    )


def test_export_uses_later_shared_reference_for_poses_and_points(tmp_path):
    sfm_anchor = np.array([[0, -1, 0, .02], [1, 0, 0, .03],
                           [0, 0, 1, .04], [0, 0, 0, 1]])
    rgbd_anchor = np.eye(4)
    rgbd_anchor[:3, 3] = [.01, .02, .03]
    identity = np.eye(4).tolist()
    sfm = tmp_path / "sfm.json"
    sfm.write_text(json.dumps({
        "recording": "clip.db3", "registered_frames": [0, 25],
        "poses_camera_to_world": [identity, sfm_anchor.tolist()],
        "pose_units": "metres_from_initial_depth", "metric_scale": {"usable": True},
    }), encoding="utf-8")
    rgbd = tmp_path / "channels.json"
    rgbd.write_text(json.dumps({
        "recording": "clip.db3", "rgbd_pose_indices": [0, 25],
        "rgbd_component_ids": [0, 0], "rgbd_poses_to_component": [identity, rgbd_anchor.tolist()],
    }), encoding="utf-8")
    structure = tmp_path / "structure.npz"
    point = (sfm_anchor @ [0, 0, .75, 1])[:3]
    np.savez(structure, points=[point])
    shared_rotation = [[0, -.8, .6], [-1, 0, 0], [0, -.6, -.8]]
    inputs = {
        "format": "grassvio-world-frame-v1", "recording": "clip", "position_unit": "m",
        "reference": {"frame": 25}, "gravity_down_camera": [0, .6, .8],
        "grass_depth_m": .75, "grass_height_m": .12,
        "rotation_world_from_camera": shared_rotation,
        "translation_world_from_camera_m": [0, 0, .72],
    }

    export(sfm, structure, rgbd, inputs, tmp_path, rgbd_channels=True)
    result = json.loads((tmp_path / "world-frame.json").read_text(encoding="utf-8"))
    assert result["reference_frame"] == 25
    shared = np.eye(4)
    shared[:3, :3] = shared_rotation
    shared[2, 3] = .72
    np.testing.assert_allclose(result["sfm_poses_camera_to_world"][1], shared, atol=1e-12)
    np.testing.assert_allclose(result["rgbd_poses_camera_to_world"][1], shared, atol=1e-12)
    np.testing.assert_allclose(result["sfm_poses_camera_to_world"][0] @ sfm_anchor, shared, atol=1e-12)
    np.testing.assert_allclose(result["rgbd_poses_camera_to_world"][0] @ rgbd_anchor, shared, atol=1e-12)
    vertex = (tmp_path / "world-points.ply").read_text(encoding="ascii").splitlines()[-1]
    np.testing.assert_allclose([float(value) for value in vertex.split()[:3]], [.45, 0, .12])

    with pytest.raises(ValueError, match="no registered pose for the shared reference frame 24"):
        export(sfm, structure, None, {**inputs, "reference": {"frame": 24}}, tmp_path)
    with pytest.raises(ValueError, match="does not match its gravity"):
        export(sfm, structure, None, {**inputs, "rotation_world_from_camera": np.eye(3)}, tmp_path)
