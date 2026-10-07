"""Unit-quaternion helpers. Quaternions are arrays of [w, x, y, z]; rotations act on body vectors."""

from __future__ import annotations

import numpy as np


def multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of (N, 4) arrays."""
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=1,
    )


def conjugate(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def to_rotvec(q: np.ndarray) -> np.ndarray:
    """Rotation vectors (rad) of unit quaternions (N, 4) -> (N, 3)."""
    q = q * np.sign(q[:, :1] + 1e-30)
    v = q[:, 1:]
    s = np.linalg.norm(v, axis=1)
    angle = 2.0 * np.arctan2(s, q[:, 0])
    return v / np.maximum(s, 1e-15)[:, None] * angle[:, None]


def from_rotvec(r: np.ndarray) -> np.ndarray:
    """Unit quaternions from rotation vectors (rad) (N, 3) -> (N, 4)."""
    angle = np.linalg.norm(r, axis=1)
    k = np.where(angle > 1e-15, np.sin(angle / 2) / np.maximum(angle, 1e-15), 0.5)
    return np.concatenate([np.cos(angle / 2)[:, None], r * k[:, None]], axis=1)


def body_rates(q: np.ndarray, rate_hz: float) -> np.ndarray:
    """Angular rate (deg/s) in the body frame between consecutive samples: (N, 4) -> (N-1, 3)."""
    return np.degrees(to_rotvec(multiply(conjugate(q[:-1]), q[1:]))) * rate_hz


def integrate(q0: np.ndarray, rates: np.ndarray, rate_hz: float) -> np.ndarray:
    """Integrate body rates (deg/s, (N, 3)) from attitude q0 -> (N+1, 4). Inverse of body_rates."""
    steps = from_rotvec(np.radians(rates) / rate_hz)
    out = np.empty((len(rates) + 1, 4))
    out[0] = q = q0 / np.linalg.norm(q0)
    for i, step in enumerate(steps, start=1):
        q = multiply(q[None], step[None])[0]
        q /= np.linalg.norm(q)
        out[i] = q
    return out


def relative_attitude_deg(q: np.ndarray) -> np.ndarray:
    """Attitude relative to the first sample as rotation vectors in degrees (N, 3)."""
    return np.degrees(to_rotvec(multiply(np.repeat(conjugate(q[:1]), len(q), axis=0), q)))
