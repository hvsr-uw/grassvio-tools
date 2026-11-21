import cv2
import numpy as np
import pytest
from recording_camera import camera,rays,project,extrinsic,Recording,DepthPairingError


def calibration(model='Brown Conrady'):
    return camera('width=640;height=480;fx=386;fy=385;ppx=323;ppy=250;model='+model+';coeffs=-0.0566,0.062,-0.0002345,0.0005963,-0.01954')


def test_brown_deprojection_agrees_with_opencv():
    cal=calibration()
    pixels=np.array([[50.,60.],[320,240],[580,430]])
    reference=cv2.undistortPoints(pixels[:,None],cal['K'],np.array(cal['coeffs'])).reshape(-1,2)
    np.testing.assert_allclose(rays(pixels,cal)[:,:2],reference,atol=1e-8)


def test_inverse_brown_roundtrip_over_image_grid():
    cal=calibration('Inverse Brown Conrady')
    yy,xx=np.mgrid[10:480:50,10:640:50]
    pixels=np.column_stack((xx.ravel(),yy.ravel()))
    np.testing.assert_allclose(project(rays(pixels,cal),cal),pixels,atol=1e-5)


def test_sdk_extrinsics_are_column_major_and_reference_to_stream():
    rotation,translation=extrinsic('rotation=0,1,0,-1,0,0,0,0,1;translation=1,2,3')
    np.testing.assert_allclose(rotation@[1,0,0]+translation,[1,3,3])


def test_unpaired_start_frame_does_not_relax_depth_limit_for_later_frames():
    recording=Recording.__new__(Recording)
    recording.color_times=np.array([2_631_900,35_987_000])
    recording.depth_times=np.array([29_310_100,62_667_000])
    with pytest.raises(DepthPairingError,match='pairing limit'):
        recording.depth_pair_for_color(0)
    index,offset=recording.depth_pair_for_color(1)
    assert index==0
    assert offset==pytest.approx(6.6769)


def test_depth_pairing_checks_empty_stream_and_threshold():
    recording=Recording.__new__(Recording)
    recording.color_times=np.array([0])
    recording.depth_times=np.array([],dtype=np.int64)
    with pytest.raises(DepthPairingError,match='no depth'):
        recording.depth_pair_for_color(0)
    with pytest.raises(ValueError,match='finite'):
        recording.depth_pair_for_color(0,max_offset_ms=float('inf'))
