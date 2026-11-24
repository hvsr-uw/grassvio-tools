import numpy as np

from constrained_inertial_initialization import GRAVITY, fit_prefix, gravity_vector, positions


def test_bias_prior_does_not_make_static_gravity_bias_separation_observable():
    t = np.arange(1., 7.)
    b = np.array([np.eye(3)*.5*x*x for x in t])
    raw = -.5*t[:, None]**2*np.array([0., GRAVITY, 0.])
    parameters, diagnostics = fit_prefix(t, raw, b, np.zeros_like(raw))
    assert diagnostics['optimizer_converged']
    assert diagnostics['data_rank'] < 8
    assert not diagnostics['observable_from_data']
    np.testing.assert_allclose(np.linalg.norm(gravity_vector(parameters[6:])), GRAVITY, atol=1e-10)


def test_constrained_fit_recovers_well_excited_known_motion():
    rng = np.random.default_rng(81)
    t = np.arange(1., 10.)
    b = np.array([np.eye(3)*.5*x*x+rng.normal(0, .2, (3, 3))*x*x for x in t])
    raw = np.zeros((len(t), 3))
    true_parameters = np.array([.1, -.05, .03, 0., 0., 0., .2, 1.])
    reference = positions(true_parameters, t, raw, b)
    estimated, diagnostics = fit_prefix(t, raw, b, reference)
    np.testing.assert_allclose(estimated, true_parameters, atol=1e-6)
    assert diagnostics['data_rank'] == 8
    assert diagnostics['observable_from_data']
