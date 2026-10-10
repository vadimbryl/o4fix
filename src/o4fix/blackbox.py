"""Compare the camera's recorded attitude with a Betaflight blackbox of the same flight.

The flight-controller gyro is the reference for the real motion of the airframe. The camera
stream is synchronised to it, the fixed camera-to-FC rotation is fitted, and the false rotation
(camera minus FC) is reported before and after correction. Requires the `tools` extra and a
blackbox with raw gyro (gyroUnfilt or debug_mode GYRO_SCALED; gyroADC as a fallback).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, sosfiltfilt

from . import quat
from .correction import Settings, correct
from .dji import QUAT_RATE_HZ, read_track

VIDEO_FRAME_RATE_HZ = 59.94
REPORT_BANDS_HZ = ((0.1, 0.3), (0.3, 1.0), (1.0, 3.0), (3.0, 10.0))
DEBUG_MODE_GYRO_SCALED = "6"


@dataclass
class Blackbox:
    time_s: np.ndarray
    gyro_dps: np.ndarray
    gyro_field: str


@dataclass
class Sync:
    """Blackbox time = offset_s + clock_ratio * camera time; camera rates ~= FC rates @ rotation."""

    offset_s: float
    clock_ratio: float
    rotation: np.ndarray
    correlation: float

    def blackbox_time(self, camera_time_s: np.ndarray) -> np.ndarray:
        return self.offset_s + self.clock_ratio * camera_time_s


@dataclass
class FalseRotation:
    per_frame_p99_deg: float
    per_frame_max_deg: float
    band_rms_deg: dict[tuple[float, float], np.ndarray]


def _lowpass(signal: np.ndarray, rate_hz: float, cutoff_hz: float, order: int = 2) -> np.ndarray:
    return sosfiltfilt(butter(order, cutoff_hz / (rate_hz / 2), output="sos"), signal, axis=0)


def load_blackbox(path: str | os.PathLike, log_index: int = 1) -> Blackbox:
    from orangebox import Parser

    parser = Parser.load(os.fspath(path), log_index)
    names = parser.field_names
    if "gyroUnfilt[0]" in names:
        gyro_field = "gyroUnfilt"
    elif str(parser.headers.get("debug_mode")) == DEBUG_MODE_GYRO_SCALED and "debug[0]" in names:
        gyro_field = "debug"
    else:
        gyro_field = "gyroADC"
    columns = [names.index("time")] + [names.index(f"{gyro_field}[{axis}]") for axis in range(3)]
    rows = [[frame.data[c] for c in columns] for frame in parser.frames() if len(frame.data) > max(columns)]
    data = np.asarray(rows, dtype=float)
    time_s = (data[:, 0] - data[0, 0]) / 1e6
    rate_hz = 1.0 / np.median(np.diff(time_s))
    gyro_dps = _lowpass(data[:, 1:4], rate_hz, min(250.0, 0.4 * rate_hz), order=4)
    return Blackbox(time_s, gyro_dps, gyro_field)


def _best_orthogonal_map(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Orthogonal matrix R minimising |source @ R - target|."""
    u, _, vt = np.linalg.svd(source.T @ target)
    return u @ vt


def _rates_at(blackbox: Blackbox, rates_dps: np.ndarray, times_s: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(times_s, blackbox.time_s, rates_dps[:, axis]) for axis in range(3)], axis=1)


def _coarse_offset(camera_dps: np.ndarray, camera_time: np.ndarray, blackbox: Blackbox, fc_dps: np.ndarray):
    """Best offset of the camera clip inside the whole log, from 20 Hz rate-magnitude envelopes."""
    step_s = 0.05
    grid = np.arange(0.0, blackbox.time_s[-1], step_s)
    fc_envelope = np.interp(grid, blackbox.time_s, np.linalg.norm(fc_dps, axis=1))
    camera_envelope = np.interp(
        np.arange(0.0, camera_time[-1], step_s), camera_time, np.linalg.norm(camera_dps, axis=1)
    )
    best_correlation, best_offset = -np.inf, None
    for shift in range(-len(camera_envelope) // 2, len(grid) - len(camera_envelope) // 2):
        first, last = max(0, -shift), min(len(camera_envelope), len(grid) - shift)
        if last - first < min(0.5 * len(camera_envelope), 0.9 * len(grid)):
            continue
        correlation = np.corrcoef(camera_envelope[first:last], fc_envelope[first + shift : last + shift])[0, 1]
        if correlation > best_correlation:
            best_correlation, best_offset = correlation, shift * step_s
    if best_offset is None:
        raise ValueError("camera clip and blackbox do not overlap")
    return best_offset, float(best_correlation)


def _residual(camera: np.ndarray, fc: np.ndarray) -> tuple[float, np.ndarray]:
    rotation = _best_orthogonal_map(fc, camera)
    return float(np.sqrt(((camera - fc @ rotation) ** 2).sum() / (camera**2).sum())), rotation


def synchronise(camera_rates_dps: np.ndarray, blackbox: Blackbox, window_s: float = 20.0) -> Sync:
    """Map camera time to blackbox time, including the drift between the two clocks (~0.1 %).

    A coarse offset comes from the whole log; local offsets are then refined in windows along
    the clip and fitted with a straight line; the camera-to-FC rotation is fitted last."""
    camera_time = np.arange(len(camera_rates_dps)) / QUAT_RATE_HZ
    camera = _lowpass(camera_rates_dps, QUAT_RATE_HZ, 20.0)
    fc = _lowpass(blackbox.gyro_dps, 1.0 / np.median(np.diff(blackbox.time_s)), 20.0)
    coarse_offset, correlation = _coarse_offset(
        _lowpass(camera_rates_dps, QUAT_RATE_HZ, 5.0),
        camera_time,
        blackbox,
        _lowpass(blackbox.gyro_dps, 1.0 / np.median(np.diff(blackbox.time_s)), 5.0),
    )
    centres, offsets = [], []
    for start in np.arange(0.0, max(camera_time[-1] - window_s, 0.0) + 1e-9, window_s):
        in_window = (camera_time >= start) & (camera_time < start + window_s)
        if np.linalg.norm(camera[in_window], axis=1).mean() < 15.0:
            continue
        best_residual, best_offset = 0.5, None
        for offset in np.arange(coarse_offset - 0.3, coarse_offset + 0.3, 0.002):
            times = camera_time[in_window] + offset
            if times[0] < 0 or times[-1] > blackbox.time_s[-1]:
                continue
            residual, _ = _residual(camera[in_window], _rates_at(blackbox, fc, times))
            if residual < best_residual:
                best_residual, best_offset = residual, offset
        if best_offset is not None:
            centres.append(start + window_s / 2)
            offsets.append(best_offset)
    if len(centres) >= 2:
        slope, offset_s = np.polyfit(centres, offsets, 1)
    elif centres:
        slope, offset_s = 0.0, offsets[0]
    else:
        slope, offset_s = 0.0, coarse_offset
    times = offset_s + (1.0 + slope) * camera_time
    overlap = (times > 0.5) & (times < blackbox.time_s[-1] - 0.5)
    _, rotation = _residual(camera[overlap], _rates_at(blackbox, fc, times[overlap]))
    return Sync(float(offset_s), float(1.0 + slope), rotation, correlation)


def false_rotation(camera_rates_dps: np.ndarray, reference_dps: np.ndarray, valid: np.ndarray) -> FalseRotation:
    """False rotation of the camera against the reference, per video frame and per frequency band."""
    error_deg = np.cumsum(np.where(valid[:, None], camera_rates_dps - reference_dps, 0.0), axis=0) / QUAT_RATE_HZ
    frame = int(QUAT_RATE_HZ / VIDEO_FRAME_RATE_HZ)
    per_frame = np.linalg.norm(error_deg[frame:] - error_deg[:-frame], axis=1)[valid[frame:]]
    band_rms = {}
    for low, high in REPORT_BANDS_HZ:
        bandpass = butter(2, [low / (QUAT_RATE_HZ / 2), high / (QUAT_RATE_HZ / 2)], "band", output="sos")
        band_rms[(low, high)] = np.sqrt((sosfiltfilt(bandpass, error_deg, axis=0)[valid] ** 2).mean(axis=0))
    return FalseRotation(float(np.percentile(per_frame, 99)), float(per_frame.max()), band_rms)


def compare_blackbox(
    clip: str | os.PathLike, log: str | os.PathLike, log_index: int = 1, settings: Settings = Settings()
) -> None:
    samples, _ = read_track(clip).unique()
    recorded = quat.body_rates(samples, QUAT_RATE_HZ)
    blackbox = load_blackbox(log, log_index)
    sync = synchronise(recorded, blackbox)
    times = sync.blackbox_time((np.arange(len(recorded)) + 0.5) / QUAT_RATE_HZ)
    valid = (times > 0.5) & (times < blackbox.time_s[-1] - 0.5)
    fc_in_camera_axes = _rates_at(blackbox, blackbox.gyro_dps, times) @ sync.rotation
    print(
        f"blackbox field {blackbox.gyro_field}; offset {sync.offset_s:+.3f} s; "
        f"clock drift {(sync.clock_ratio - 1) * 1000:+.2f} ms/s; sync correlation {sync.correlation:.2f}"
    )
    corrected = quat.body_rates(correct(samples, QUAT_RATE_HZ, settings).quats, QUAT_RATE_HZ)
    for label, rates in (("as recorded", recorded), ("o4fix", corrected)):
        result = false_rotation(rates, fc_in_camera_axes, valid)
        print(
            f"\n{label}: false rotation per video frame "
            f"p99 {result.per_frame_p99_deg:.2f} deg, max {result.per_frame_max_deg:.2f} deg"
        )
        for (low, high), rms in result.band_rms_deg.items():
            print(f"  {low:>4}-{high:<4} Hz: rms per camera axis X/Y/Z {rms.round(2)} deg")
