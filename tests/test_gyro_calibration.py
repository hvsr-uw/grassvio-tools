import numpy as np
from gyro_calibration import samples_between,integrate_rotation,fit_bias


def test_visual_rotation_prefix_identifies_constant_gyro_bias():
    times=np.arange(201,dtype=np.int64)*5_000_000
    bias=np.array([.004,-.007,.003])
    clean=np.column_stack((.3*np.sin(times/1e9*3),.2*np.cos(times/1e9*4),.1+times/1e10))
    references=[];measurements=[]
    for start,end in [(0,300_000_000),(300_000_000,650_000_000),(650_000_000,1_000_000_000)]:
        references.append(integrate_rotation(samples_between(times,clean,start,end),np.zeros(3),np.eye(3)))
        measurements.append(samples_between(times,clean+bias,start,end))
    recovered,error=fit_bias(measurements,references,np.eye(3))
    np.testing.assert_allclose(recovered,bias,atol=1e-6)
    assert error<1e-7
