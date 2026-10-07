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
from .correct import Settings, correct
from .dji import QUAT_RATE_HZ, read_track

CAMERA_CLOCK_HZ = 1000.2
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
    """Camera time + offset_s = blackbox time; camera rates ~= FC rates @ rotation."""

    offset_s: float
    rotation: np.ndarray
    correlation: float


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


def synchronise(camera_rates_dps: np.ndarray, blackbox: Blackbox, search_s: float = 10.0) -> Sync:
    """Find the time offset by correlating rate magnitudes, then refine it with the axis-resolved fit."""
    camera_time = (np.arange(len(camera_rates_dps)) + 0.5) / CAMERA_CLOCK_HZ
    camera = _lowpass(camera_rates_dps, CAMERA_CLOCK_HZ, 20.0)
    fc = _lowpass(blackbox.gyro_dps, 1.0 / np.median(np.diff(blackbox.time_s)), 20.0)
    end_s = blackbox.time_s[-1]

    def fc_rates_at(times):
        return np.stack([np.interp(times, blackbox.time_s, fc[:, axis]) for axis in range(3)], axis=1)

    best_correlation, coarse_offset = -np.inf, None
    for offset in np.arange(-search_s, search_s, 0.01):
        times = camera_time[::10] + offset
        overlap = (times > 0) & (times < end_s)
        if overlap.sum() < 0.5 * len(times):
            continue
        correlation = np.corrcoef(
            np.linalg.norm(fc_rates_at(times[overlap]), axis=1), np.linalg.norm(camera[::10][overlap], axis=1)
        )[0, 1]
        if correlation > best_correlation:
            best_correlation, coarse_offset = correlation, offset
    if coarse_offset is None:
        raise ValueError("camera clip and blackbox do not overlap")

    best_residual, best = np.inf, None
    for offset in np.arange(coarse_offset - 0.05, coarse_offset + 0.05, 0.0005):
        times = camera_time + offset
        overlap = (times > 0.5) & (times < end_s - 0.5)
        fc_rates = fc_rates_at(times[overlap])
        rotation = _best_orthogonal_map(fc_rates, camera[overlap])
        residual = np.sqrt(((camera[overlap] - fc_rates @ rotation) ** 2).sum() / (camera[overlap] ** 2).sum())
        if residual < best_residual:
            best_residual, best = residual, (offset, rotation)
    return Sync(best[0], best[1], float(best_correlation))


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
    times = (np.arange(len(recorded)) + 0.5) / CAMERA_CLOCK_HZ + sync.offset_s
    valid = (times > 0.5) & (times < blackbox.time_s[-1] - 0.5)
    fc_in_camera_axes = (
        np.stack([np.interp(times, blackbox.time_s, blackbox.gyro_dps[:, axis]) for axis in range(3)], axis=1)
        @ sync.rotation
    )
    print(
        f"blackbox field {blackbox.gyro_field}; offset {sync.offset_s:+.3f} s; "
        f"sync correlation {sync.correlation:.2f}; camera-to-FC rotation det {np.linalg.det(sync.rotation):+.0f}"
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
