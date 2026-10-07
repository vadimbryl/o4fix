import numpy as np

from o4fix import quat
from o4fix.correct import Settings, correct, remove_impulses, remove_pullback

RATE = 1000.0


def real_motion(seconds=6.0):
    """Smooth flight-like rates (deg/s): slow turns plus a fast 360 deg roll flip at 3 s."""
    t = np.arange(int(seconds * RATE)) / RATE
    rates = np.stack([20 * np.sin(2 * np.pi * 0.4 * t), 15 * np.sin(2 * np.pi * 0.7 * t + 1), 10 * np.cos(t)], axis=1)
    flip = np.exp(-0.5 * ((t - 3.0) / 0.12) ** 2)
    rates[:, 0] += 360.0 * flip / (flip.sum() / RATE)  # integrates to exactly 360 deg
    return rates


def false_burst(n, start, axis, area_deg):
    """A cluster of ~6 ms bell-shaped impulses on one axis, like the ones the ICM-40609-D produces."""
    out = np.zeros((n, 3))
    k = np.arange(-8, 9)
    bell = np.exp(-0.5 * (k / 2.0) ** 2)
    for j, centre in enumerate((start, start + 30, start + 70)):
        pulse = bell / bell.sum() * RATE * area_deg / 3 * (1 if j != 1 else 1.2)
        out[centre - 8 : centre + 9, axis] += pulse
    return out


def test_real_motion_including_a_flip_is_untouched():
    rates = real_motion()
    cleaned, bursts = remove_impulses(rates, RATE)
    assert bursts == []
    np.testing.assert_array_equal(cleaned, rates)


def test_false_burst_is_removed_and_area_recovered():
    rates = real_motion()
    burst = false_burst(len(rates), 1500, axis=1, area_deg=6.0)
    cleaned, bursts = remove_impulses(rates + burst, RATE)
    assert len(bursts) == 1 and bursts[0].axis == 1
    assert abs(bursts[0].area_deg - burst[:, 1].sum() / RATE) < 0.2
    leftover = (cleaned - rates)[:, 1].sum() / RATE
    assert abs(leftover) < 0.2


def test_pullback_removal_cancels_exponential_recovery():
    tau, n = 12.5, int(40 * RATE)
    t = np.arange(n) / RATE
    step = np.zeros((n, 3))
    step[5000, 0] = -10.0 * RATE  # false step of -10 deg at 5 s
    recovery = np.where(t[:, None] > 5.0, 10.0 / tau * np.exp(-(t[:, None] - 5.0) / tau), 0.0) * [1, 0, 0]
    observed_after_step_removal = recovery  # what is left once the step itself is cut out
    restored = remove_pullback(observed_after_step_removal, step, RATE, tau)
    assert np.abs(restored.sum(axis=0) / RATE).max() < 0.05


def test_correct_removes_false_step_from_attitude():
    rates = real_motion() + false_burst(6000, 1500, axis=0, area_deg=8.0)
    q = quat.integrate(np.array([1.0, 0, 0, 0]), rates, RATE)
    truth = quat.integrate(np.array([1.0, 0, 0, 0]), real_motion(), RATE)
    result = correct(q, RATE, Settings(pullback_tau_s=0))
    err = quat.relative_attitude_deg(quat.multiply(quat.conjugate(truth), result.quats))
    raw = quat.relative_attitude_deg(quat.multiply(quat.conjugate(truth), q))
    assert np.abs(raw[-1]).max() > 7.0
    assert np.abs(err[-1]).max() < 0.5
