"""Removal of false rotations from the O4 attitude track.

Units with the ICM-40609-D gyro record false single-axis impulse bursts under motor vibration,
and the camera's attitude fusion then pulls each false step back exponentially. See
docs/REPORT.md for the measurements behind the defaults.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d
from scipy.signal import butter, lfilter, sosfiltfilt

from . import quat


@dataclass(frozen=True)
class Settings:
    """Correction parameters. A value of 0 disables the corresponding step."""

    impulse_threshold_dps: float = 60.0
    lowpass_hz: float = 20.0
    busy_lowpass_hz: float = 3.0
    pullback_tau_s: float = 12.5


@dataclass(frozen=True)
class Burst:
    """A removed false burst: inclusive sample range, camera axis (0, 1, 2 = X, Y, Z) and rotation removed."""

    start_sample: int
    end_sample: int
    axis: int
    area_deg: float


@dataclass
class Result:
    quats: np.ndarray
    bursts: list[Burst] = field(default_factory=list)
    busy_weight: np.ndarray | None = None


def remove_impulses(
    rates_dps: np.ndarray,
    rate_hz: float,
    threshold_dps: float = 60.0,
    purity: float = 0.6,
    median_window_ms: float = 61,
    padding_samples: int = 3,
    min_width_ms: float = 2,
    max_width_ms: float = 25,
    min_area_deg: float = 0.25,
    merge_gap_ms: float = 80,
    noisy_merge_gap_ms: float = 120,
    edge_window_ms: float = 15,
) -> tuple[np.ndarray, list[Burst]]:
    """Remove false single-axis impulse bursts from body rates of shape (N, 3).

    An impulse is a run where one axis departs from its running median by more than
    `threshold_dps` while the other two stay below `purity` of that deviation. Impulses on one
    axis form a burst when they are closer than `merge_gap_ms`, or closer than
    `noisy_merge_gap_ms` with jittery false content in between. The whole burst, including the
    false shelves between impulses, is replaced by a straight line between the clean rates around it.
    """
    window = int(median_window_ms * rate_hz / 1000) | 1
    baseline = median_filter(rates_dps, size=(window, 1), mode="nearest")
    deviation = np.abs(rates_dps - baseline)
    cleaned = rates_dps.copy()
    bursts: list[Burst] = []
    merge_gap = int(merge_gap_ms * rate_hz / 1000)
    noisy_merge_gap = int(noisy_merge_gap_ms * rate_hz / 1000)
    edge = max(int(edge_window_ms * rate_hz / 1000), 1)
    last = len(rates_dps) - 1

    for axis in range(3):
        other_axes = np.max(np.delete(deviation, axis, axis=1), axis=1)
        is_impulse = (deviation[:, axis] > threshold_dps) & (other_axes < purity * deviation[:, axis])
        impulse_samples = np.flatnonzero(is_impulse)
        spans: list[list[int]] = []
        for run in np.split(impulse_samples, np.flatnonzero(np.diff(impulse_samples) > 2) + 1):
            if not len(run):
                continue
            start, end = int(run[0]), int(run[-1])
            while start > 0 and deviation[start - 1, axis] > threshold_dps / 5:
                start -= 1
            while end < last and deviation[end + 1, axis] > threshold_dps / 5:
                end += 1
            width_ms = (end - start + 1) * 1000 / rate_hz
            area_deg = (rates_dps[start : end + 1, axis] - baseline[start : end + 1, axis]).sum() / rate_hz
            if min_width_ms <= width_ms <= max_width_ms and abs(area_deg) >= min_area_deg:
                spans.append([max(start - padding_samples, 0), min(end + padding_samples, last)])

        merged: list[list[int]] = []
        for start, end in sorted(spans):
            if merged and _mergeable(
                rates_dps[merged[-1][1] : start + 1, axis], merge_gap, noisy_merge_gap, threshold_dps
            ):
                merged[-1][1] = max(merged[-1][1], end)
            else:
                merged.append([start, end])

        for start, end in merged:
            before = rates_dps[max(start - edge, 0) : start, axis]
            after = rates_dps[end + 1 : end + 1 + edge, axis]
            rate_before = float(np.median(before)) if len(before) else None
            rate_after = float(np.median(after)) if len(after) else None
            if rate_before is None and rate_after is None:
                continue
            rate_before = rate_after if rate_before is None else rate_before
            rate_after = rate_before if rate_after is None else rate_after
            replacement = np.linspace(rate_before, rate_after, end - start + 1)
            removed_deg = float((rates_dps[start : end + 1, axis] - replacement).sum() / rate_hz)
            bursts.append(Burst(start, end, axis, removed_deg))
            cleaned[start : end + 1, axis] = replacement
    return cleaned, bursts


def _mergeable(gap_rates: np.ndarray, merge_gap: int, noisy_merge_gap: int, threshold_dps: float) -> bool:
    """Bursts merge across short gaps, and across longer ones only when the gap itself jitters
    sample to sample like false content does; a real manoeuvre is smooth at 1 kHz."""
    gap = len(gap_rates) - 1
    if gap <= merge_gap:
        return True
    return gap <= noisy_merge_gap and np.abs(np.diff(gap_rates)).mean() > threshold_dps / 8


def remove_pullback(rates_dps: np.ndarray, removed_dps: np.ndarray, rate_hz: float, tau_s: float) -> np.ndarray:
    """Remove the fusion's exponential pull-back of the false steps that were cut out.

    The camera pulls every false step back with a first-order decay. Once the step itself is
    removed, that pull-back would remain as an opposite drift, so its rate (-false_step / tau)
    is added back."""
    dt = 1.0 / rate_hz
    false_step_deg = lfilter([dt], [1.0, -np.exp(-dt / tau_s)], removed_dps, axis=0)
    return rates_dps + false_step_deg / tau_s


def lowpass(rates_dps: np.ndarray, rate_hz: float, cutoff_hz: float, order: int = 4) -> np.ndarray:
    """Zero-phase Butterworth low-pass; no-op when cutoff_hz is 0."""
    if not cutoff_hz:
        return rates_dps
    return sosfiltfilt(butter(order, cutoff_hz / (rate_hz / 2), output="sos"), rates_dps, axis=0)


def adaptive_lowpass(
    rates_dps: np.ndarray,
    removed_dps: np.ndarray,
    rate_hz: float,
    cutoff_hz: float,
    busy_cutoff_hz: float,
    busy_level_dps: float = 20.0,
    smoothing_s: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Low-pass at cutoff_hz, blending towards busy_cutoff_hz where impulse removal took out a lot."""
    window = max(int(smoothing_s * rate_hz), 1)
    removal_level = uniform_filter1d(np.linalg.norm(removed_dps, axis=1), window, mode="nearest")
    busy_weight = np.clip((removal_level - busy_level_dps / 4) / (busy_level_dps * 3 / 4), 0.0, 1.0)
    busy_weight = uniform_filter1d(busy_weight, window, mode="nearest")
    calm = lowpass(rates_dps, rate_hz, cutoff_hz)
    busy = lowpass(rates_dps, rate_hz, busy_cutoff_hz)
    return (1 - busy_weight[:, None]) * calm + busy_weight[:, None] * busy, busy_weight


def correct(quats: np.ndarray, rate_hz: float, settings: Settings = Settings()) -> Result:
    """Return corrected unique attitude samples of shape (N, 4) and what was removed."""
    if len(quats) < 2:
        raise ValueError("need at least two attitude samples")
    rates = quat.body_rates(quats, rate_hz)
    cleaned, bursts = remove_impulses(rates, rate_hz, threshold_dps=settings.impulse_threshold_dps)
    removed = rates - cleaned
    if settings.pullback_tau_s:
        cleaned = remove_pullback(cleaned, removed, rate_hz, settings.pullback_tau_s)
    busy_weight = None
    if settings.lowpass_hz and settings.busy_lowpass_hz:
        cleaned, busy_weight = adaptive_lowpass(
            cleaned, removed, rate_hz, settings.lowpass_hz, settings.busy_lowpass_hz
        )
    else:
        cleaned = lowpass(cleaned, rate_hz, settings.lowpass_hz)
    return Result(quat.integrate(quats[0], cleaned, rate_hz), bursts, busy_weight)
