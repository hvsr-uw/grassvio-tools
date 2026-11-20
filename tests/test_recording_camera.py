import cv2
import numpy as np
from recording_camera import camera,rays,project,extrinsic


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
