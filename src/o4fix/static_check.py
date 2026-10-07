"""Static-camera check: compare the attitude the gyro recorded with the motion seen in the video.

Record with RockSteady/HorizonSteady OFF while the camera does not move, for example with the
quad on a table, propellers removed and the motors spun up from the Betaflight Motors tab. Any
attitude change that the video does not show is a false gyro reading. Requires the `tools` extra.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from . import quat
from .dji import QUAT_RATE_HZ, REFERENCE_WIDTH_PX, read_focal_px, read_track

REPORTED_JUMP_DEG = 0.3


@dataclass
class VideoMotion:
    """Image rotation about the optical axis and image shift in full-resolution pixels,
    both relative to the first frame, sampled at `time_s`."""

    time_s: np.ndarray
    rotation_deg: np.ndarray
    shift_px: np.ndarray
    focal_px: float

    @property
    def motion_deg(self) -> np.ndarray:
        """Upper bound of the camera rotation implied by the image rotation and shift."""
        shift_deg = np.degrees(np.hypot(self.shift_px[:, 0], self.shift_px[:, 1]) / self.focal_px)
        return np.hypot(self.rotation_deg, shift_deg)


def video_motion(path: str | os.PathLike, step_s: float = 0.1, scale: float = 0.5) -> VideoMotion:
    """Track features of the first frame through the clip and fit a similarity transform per step."""
    import cv2

    video = cv2.VideoCapture(os.fspath(path))
    if not video.isOpened():
        raise OSError(f"cannot open video {path}")
    fps = video.get(cv2.CAP_PROP_FPS)
    width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_step = max(round(step_s * fps), 1)

    def to_gray(frame):
        small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    decoded, frame = video.read()
    if not decoded:
        raise OSError(f"cannot decode video {path}")
    first_frame = to_gray(frame)
    center = np.array([first_frame.shape[1] / 2, first_frame.shape[0] / 2])
    features = cv2.goodFeaturesToTrack(first_frame, 800, 0.005, 15)
    samples = [(0.0, 0.0, 0.0, 0.0)]
    frame_index = 0
    while True:
        for _ in range(frame_step):
            decoded = video.grab()
            frame_index += 1
        if not decoded:
            break
        decoded, frame = video.retrieve()
        if not decoded:
            break
        tracked, found, _ = cv2.calcOpticalFlowPyrLK(
            first_frame, to_gray(frame), features, None, winSize=(31, 31), maxLevel=4
        )
        found = found[:, 0] == 1
        transform, _ = cv2.estimateAffinePartial2D(
            features[found, 0] - center, tracked[found, 0] - center, method=cv2.RANSAC, ransacReprojThreshold=1.0
        )
        if transform is not None:
            rotation_deg = np.degrees(np.arctan2(transform[1, 0], transform[0, 0]))
            samples.append((frame_index / fps, rotation_deg, transform[0, 2] / scale, transform[1, 2] / scale))
    video.release()
    samples = np.asarray(samples)
    focal_px = read_focal_px(path) * width / REFERENCE_WIDTH_PX
    return VideoMotion(samples[:, 0], samples[:, 1], samples[:, 2:4], focal_px)


def recorded_attitude_deg(path: str | os.PathLike) -> np.ndarray:
    """Attitude the camera recorded, relative to its first sample, at QUAT_RATE_HZ."""
    samples, _ = read_track(path).unique()
    return quat.relative_attitude_deg(samples)


def check_static(path: str | os.PathLike, step_s: float = 0.1, csv_path: str | None = None) -> None:
    attitude = recorded_attitude_deg(path)
    video = video_motion(path, step_s)
    gyro = attitude[np.minimum((video.time_s * QUAT_RATE_HZ).astype(int), len(attitude) - 1)]
    largest_shift_px = np.hypot(video.shift_px[:, 0], video.shift_px[:, 1]).max()
    print(f"{os.path.basename(os.fspath(path))}: {video.time_s[-1]:.1f} s, focal {video.focal_px:.0f} px")
    print(
        f"  attitude change recorded by the gyro: max {np.linalg.norm(gyro, axis=1).max():.2f} deg "
        f"(per axis {np.abs(gyro).max(axis=0).round(2)})"
    )
    print(
        f"  camera motion seen in the video:      max {video.motion_deg.max():.2f} deg "
        f"(rotation {np.abs(video.rotation_deg).max():.3f} deg, shift {largest_shift_px:.1f} px)"
    )
    gyro_steps = np.linalg.norm(np.diff(gyro, axis=0), axis=1)
    for step in np.flatnonzero(gyro_steps > REPORTED_JUMP_DEG):
        video_step = abs(video.motion_deg[step + 1] - video.motion_deg[step])
        print(f"  gyro jump at {video.time_s[step + 1]:6.2f} s: {gyro_steps[step]:.2f} deg, video {video_step:.3f} deg")
    if csv_path:
        np.savetxt(
            csv_path,
            np.c_[video.time_s, gyro, video.rotation_deg, video.shift_px, video.motion_deg],
            delimiter=",",
            fmt="%.5f",
            header="time_s,gyro_x_deg,gyro_y_deg,gyro_z_deg,image_rotation_deg,image_dx_px,image_dy_px,video_motion_deg",
            comments="",
        )
        print(f"  written {csv_path}")
