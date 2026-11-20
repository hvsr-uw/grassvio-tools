import numpy as np
from inertial_initialization import solve_initialization,design_matrix


def test_fixed_orientation_cannot_separate_accel_bias_and_gravity():
    times=np.arange(1.,7.)
    b=np.array([np.eye(3)*.5*t*t for t in times])
    raw=np.zeros((len(times),3));positions=np.zeros_like(raw)
    _,fit=solve_initialization(times,raw,b,positions)
    assert fit['rank']<9
    assert fit['observable_by_linear_gate'] is False


def test_full_rank_initialized_fit_recovers_known_parameters():
    rng=np.random.default_rng(70)
    times=np.arange(1.,12.)
    b=np.array([np.eye(3)*.5*t*t+rng.normal(0,.1,(3,3))*t*t for t in times])
    parameters=np.array([.1,.2,-.1,.03,-.05,.02,0,9.81,0])
    positions=(design_matrix(times,b)@parameters).reshape(-1,3)
    fitted,report=solve_initialization(times,np.zeros_like(positions),b,positions)
    np.testing.assert_allclose(fitted,parameters,atol=1e-8)
    assert report['observable_by_linear_gate'] is True
    assert report['physically_plausible'] is True
