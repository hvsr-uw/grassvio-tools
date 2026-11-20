import numpy as np
from registered_map import fit_plane,plane_basis


def test_initial_plane_is_measured_robustly_and_basis_is_metric():
    rng=np.random.default_rng(61)
    ground=rng.uniform([-1,.4,1],[1,.4,3],(1500,3))
    ground[:,1]+=rng.normal(0,.003,len(ground))
    outliers=rng.uniform([-1,-1,1],[1,1,3],(300,3))
    normal,distance,stats=fit_plane(np.vstack((ground,outliers)))
    assert normal[1]>.999
    assert abs(distance-.4)<.003
    basis=plane_basis(normal)
    np.testing.assert_allclose(basis.T@basis,np.eye(2),atol=1e-10)
    np.testing.assert_allclose(normal@basis,0,atol=1e-10)
