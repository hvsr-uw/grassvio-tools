import cv2
import numpy as np
import pytest
from motion_channels import (estimate_pnp, inertial_path, angle, reconnect_component,
                             fit_anchor_loop, estimate_static_window,
                             velocity_from_visual_displacement)


def test_pnp_recovers_target_from_source_metric_pose():
    points=np.random.default_rng(50).uniform([-1,-1,1],[1,1,4],(100,3))
    r=cv2.Rodrigues(np.array([.05,.08,-.02]))[0];t=np.array([.1,.03,.02])
    target=points@r.T+t
    rays=target/target[:,2:]
    transform,stats=estimate_pnp(points,rays,400.)
    assert stats['status']=='estimated'
    assert angle(transform[:3,:3],r)<1e-5
    np.testing.assert_allclose(transform[:3,3],t,atol=1e-6)


def test_static_imu_does_not_invent_translation():
    times=np.arange(401,dtype=np.int64)*5_000_000
    gyro=np.zeros((len(times),3));accel=np.tile([0,-9.81,0],(len(times),1))
    path,init=inertial_path(times,gyro,times,accel,times[[0,200,400]],np.eye(3),np.eye(3),np.array([-.03,0,0]),np.zeros(3))
    np.testing.assert_allclose(path[int(times[-1])][1],0,atol=1e-10)
    assert init['initial_camera_velocity']=='assumed_zero_not_verified'
    assert init['accelerometer_bias_separately_estimated'] is False


def test_inertial_comparison_never_clamps_outside_sensor_coverage():
    times=np.arange(201,dtype=np.int64)*5_000_000
    data=np.zeros((len(times),3))
    with pytest.raises(ValueError,match='coverage'):
        inertial_path(times,data,times,data,np.array([0,2_000_000_000]),np.eye(3),np.eye(3),np.zeros(3),np.zeros(3))


def test_relocalization_rebases_only_the_component_with_measured_links():
    poses=[np.eye(4) for _ in range(3)]
    ids=[0,1,2];sources=['initial','gap','gap']
    alignment=np.eye(4);alignment[0,3]=2.
    recovered=reconnect_component(poses,ids,sources,2,alignment)
    assert recovered==[2]
    assert ids==[0,1,0]
    assert poses[1][0,3]==0.
    assert poses[2][0,3]==2.


def test_loop_fit_checks_spatially_unfitted_tracks():
    rng=np.random.default_rng(75)
    points=rng.uniform([-1,-1,2],[1,1,4],(500,3))
    pixels=points[:,:2]/points[:,2:]*400+[320,240]
    target=points+[.1,0,0]
    target=target/target[:,2:]
    fitted,stats=fit_anchor_loop(points,target,pixels,400.)
    assert fitted is not None
    assert stats['loop_heldout_tracks']>20
    assert stats['loop_heldout_median_error_px']<1e-4
    cells=np.floor(pixels/40).astype(int)
    test=(cells[:,0]+cells[:,1])%2==1
    corrupted=target.copy();corrupted[test,0]+=.1
    fitted,stats=fit_anchor_loop(points,corrupted,pixels,400.)
    assert fitted is None
    assert stats['loop_heldout_median_error_px']>30


def test_static_window_does_not_assume_the_recording_starts_at_rest():
    times = np.arange(601, dtype=np.int64) * 5_000_000
    gyro = np.zeros((len(times), 3))
    accel = np.tile([0., -9.81, 0.], (len(times), 1))
    # The first 0.5 s is moving. A quiet interval occurs later.
    gyro[:100, 2] = 1.0
    accel[:100, 0] = 0.8
    window = estimate_static_window(times, gyro, times, accel, window_seconds=0.5)
    assert window["start_ns"] >= times[100]
    assert window["median_gyro_norm"] == 0.0


def test_inertial_path_accepts_a_visual_initial_velocity():
    times = np.arange(401, dtype=np.int64) * 5_000_000
    gyro = np.zeros((len(times), 3))
    accel = np.tile([0., -9.81, 0.], (len(times), 1))
    path, init = inertial_path(
        times, gyro, times, accel, times[[0, 200, 400]], np.eye(3), np.eye(3),
        np.zeros(3), np.zeros(3),
        gravity_window={"start_ns": 0, "end_ns": 500_000_000},
        initial_velocity=np.array([1., 0., 0.]),
    )
    np.testing.assert_allclose(path[int(times[-1])][1], [2., 0., 0.], atol=1e-10)
    assert init["initial_camera_velocity"] == "provided_visual_reference"


def test_later_static_gravity_is_transported_to_start_orientation():
    times = np.arange(401, dtype=np.int64)*5_000_000
    seconds = times/1e9
    # Rotate for one second, then stop. There is no linear acceleration.
    speed = np.where(seconds < 1., .6, 0.)
    theta = np.r_[0., np.cumsum((speed[:-1]+speed[1:])*.5*.005)]
    gyro = np.column_stack([np.zeros(len(times)), np.zeros(len(times)), speed])
    gravity = np.array([0., 9.81, 0.])
    accel = np.array([-cv2.Rodrigues(np.array([0., 0., a]))[0].T@gravity for a in theta])
    path, init = inertial_path(times, gyro, times, accel, times[[0, 200, 400]],
        np.eye(3), np.eye(3), np.zeros(3), np.zeros(3),
        gravity_window={'start_ns': 1_200_000_000, 'end_ns': 1_800_000_000})
    np.testing.assert_allclose(init['gravity_vector'], gravity, atol=1e-10)
    np.testing.assert_allclose(path[int(times[-1])][1], 0., atol=1e-10)


def test_visual_velocity_subtracts_acceleration_and_lever_contribution():
    # At t=2, v0=1 and acceleration=2 give displacement 6, not v0=3.
    np.testing.assert_allclose(velocity_from_visual_displacement([6., 0., 0.],
        [4., 0., 0.], 2.), [1., 0., 0.])


def test_continuously_rotating_recording_is_not_a_static_bias_candidate():
    times = np.arange(201, dtype=np.int64)*5_000_000
    gyro = np.tile([0., 0., .3], (len(times), 1))
    accel = np.tile([0., -9.81, 0.], (len(times), 1))
    window = estimate_static_window(times, gyro, times, accel)
    assert not window['low_motion_candidate']
    assert not window['stationary_verified']
