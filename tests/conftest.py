"""Builds a minimal MP4 with a DJI `djmd` attitude track, so the parser and writer can be tested
without real recordings."""

from __future__ import annotations

import struct

import numpy as np
import pytest


def _varint(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def _len_field(number: int, payload: bytes) -> bytes:
    return _varint(number << 3 | 2) + _varint(len(payload)) + payload


def _float_field(number: int, value: float) -> bytes:
    return _varint(number << 3 | 5) + struct.pack("<f", value)


def _djmd_sample(quats: np.ndarray, timestamp: int) -> bytes:
    """One frame message: 3 -> 3 -> 2 -> repeated 3 {1: w, 2: x, 3: y, 4: z}, plus unrelated fields."""
    block = _varint(1 << 3 | 0) + _varint(timestamp)
    for q in quats:
        block += _len_field(3, b"".join(_float_field(i + 1, float(v)) for i, v in enumerate(q)))
    imu = _len_field(2, block)
    frame = _len_field(2, _len_field(4, b"DJI TEST")) + _len_field(3, imu)
    return _len_field(1, _len_field(8, _float_field(1, 1457.07))) + _len_field(3, frame)


def _box(kind: str, payload: bytes) -> bytes:
    return struct.pack(">I4s", 8 + len(payload), kind.encode()) + payload


def build_mp4(samples: list[bytes]) -> bytes:
    """ftyp + mdat (the samples, one per chunk) + moov with a single djmd track."""
    ftyp = _box("ftyp", b"isom\x00\x00\x02\x00isom")
    mdat_payload = b"".join(samples)
    mdat_start = len(ftyp) + 8
    offsets, pos = [], mdat_start
    for s in samples:
        offsets.append(pos)
        pos += len(s)
    n = len(samples)
    stsd = _box("stsd", struct.pack(">II", 0, 1) + struct.pack(">I4s", 16, b"djmd") + b"\x00" * 8)
    stsz = _box("stsz", struct.pack(">III", 0, 0, n) + b"".join(struct.pack(">I", len(s)) for s in samples))
    stco = _box("stco", struct.pack(">II", 0, n) + b"".join(struct.pack(">I", o) for o in offsets))
    stsc = _box("stsc", struct.pack(">II", 0, 1) + struct.pack(">III", 1, 1, 1))
    stts = _box("stts", struct.pack(">II", 0, 1) + struct.pack(">II", n, 1001))
    stbl = _box("stbl", stsd + stts + stsc + stsz + stco)
    trak = _box("trak", _box("mdia", _box("minf", stbl)))
    return ftyp + _box("mdat", mdat_payload) + _box("moov", trak)


@pytest.fixture
def synthetic_clip(tmp_path):
    """Path to a synthetic clip with 3 frames x 4 unique samples (each written twice), and the samples."""
    rng = np.random.default_rng(0)
    q = rng.normal(size=(12, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    q = q.astype(np.float32).astype(float)  # values exactly representable in the file
    frames = [np.repeat(q[i * 4 : (i + 1) * 4], 2, axis=0) for i in range(3)]
    path = tmp_path / "DJI_TEST.MP4"
    path.write_bytes(build_mp4([_djmd_sample(f, 1000 + i) for i, f in enumerate(frames)]))
    return path, q
