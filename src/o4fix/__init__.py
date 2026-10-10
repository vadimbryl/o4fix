"""Repair the gyro track of DJI O4 Air Unit (Pro) videos recorded with an ICM-40609-D gyro."""

from .correction import Burst, Result, Settings, correct
from .dji import QUAT_RATE_HZ, AttitudeTrack, TrackError, read_focal_px, read_track, write_track

__version__ = "0.5.0"

__all__ = [
    "QUAT_RATE_HZ",
    "AttitudeTrack",
    "Burst",
    "Result",
    "Settings",
    "TrackError",
    "correct",
    "read_focal_px",
    "read_track",
    "write_track",
    "__version__",
]
