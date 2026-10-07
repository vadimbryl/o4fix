import numpy as np

from o4fix import quat


def test_rotvec_roundtrip():
    rng = np.random.default_rng(1)
    r = rng.normal(size=(100, 3))
    r *= rng.uniform(0.0, 3.1, size=(100, 1)) / np.linalg.norm(r, axis=1, keepdims=True)  # angles below pi are unique
    np.testing.assert_allclose(quat.to_rotvec(quat.from_rotvec(r)), r, atol=1e-12)


def test_integrate_is_inverse_of_body_rates():
    rng = np.random.default_rng(2)
    rates = rng.normal(scale=200.0, size=(2000, 3))
    q = quat.integrate(np.array([1.0, 0.0, 0.0, 0.0]), rates, 1000.0)
    np.testing.assert_allclose(quat.body_rates(q, 1000.0), rates, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-12)


def test_relative_attitude_of_known_rotation():
    rates = np.zeros((1000, 3))
    rates[:, 2] = 90.0  # 90 deg/s about Z for 1 s
    q = quat.integrate(np.array([1.0, 0.0, 0.0, 0.0]), rates, 1000.0)
    np.testing.assert_allclose(quat.relative_attitude_deg(q)[-1], [0.0, 0.0, 90.0], atol=1e-9)
