"""Read and rewrite the attitude track that DJI O4 cameras embed in their MP4 files.

With RockSteady and HorizonSteady off, every sample of the `djmd` metadata track holds the
camera attitude as unit quaternions at 1 kHz, each value written twice.
"""

from __future__ import annotations

import os
import shutil
import struct
import tempfile
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np

QUAT_RATE_HZ = 1000.0
O4_PRO_FOCAL_PX = 1457.07
REFERENCE_WIDTH_PX = 3840

METADATA_SAMPLE_ENTRY = "djmd"

WIRE_VARINT, WIRE_FIXED64, WIRE_BYTES, WIRE_FIXED32 = 0, 1, 2, 5

FRAME_FIELD = 3
IMU_FIELD = 3
ATTITUDE_BLOCK_FIELD = 2
QUATERNION_FIELD = 3
QUATERNION_COMPONENT_FIELDS = (1, 2, 3, 4)

HEADER_FIELD = 1
LENS_FIELD = 8
FOCAL_LENGTH_FIELD = 1


class TrackError(ValueError):
    """The file does not contain a usable DJI attitude track."""


def _iter_boxes(file, start: int, end: int) -> Iterator[tuple[str, int, int]]:
    """Yield (box_type, payload_start, box_end) for the MP4 boxes in [start, end)."""
    position = start
    while position < end:
        file.seek(position)
        header = file.read(8)
        if len(header) < 8:
            return
        size, box_type = struct.unpack(">I4s", header)
        header_size = 8
        if size == 1:
            size = struct.unpack(">Q", file.read(8))[0]
            header_size = 16
        elif size == 0:
            size = end - position
        if size < header_size:
            raise TrackError(f"corrupt MP4 box at offset {position}")
        yield box_type.decode("latin1"), position + header_size, position + size
        position += size


def _read_sample_tables(file, file_size: int, sample_entry: str) -> dict[str, bytes]:
    for box_type, start, end in _iter_boxes(file, 0, file_size):
        if box_type != "moov":
            continue
        for trak_type, trak_start, trak_end in _iter_boxes(file, start, end):
            if trak_type != "trak":
                continue
            tables: dict[str, bytes] = {}
            containers = [(trak_start, trak_end)]
            while containers:
                for child_type, child_start, child_end in _iter_boxes(file, *containers.pop()):
                    if child_type in ("mdia", "minf", "stbl"):
                        containers.append((child_start, child_end))
                    else:
                        file.seek(child_start)
                        tables[child_type] = file.read(child_end - child_start)
            if tables.get("stsd", b"")[12:16].decode("latin1") == sample_entry:
                return tables
    raise TrackError(f'no "{sample_entry}" track in this file (is it an original DJI O4 recording?)')


def iter_samples(file, file_size: int, sample_entry: str) -> Iterator[tuple[int, int]]:
    """Yield (file_offset, size) of every sample in the track with the given sample entry."""
    tables = _read_sample_tables(file, file_size, sample_entry)
    fixed_size, sample_count = struct.unpack(">II", tables["stsz"][4:12])
    if fixed_size:
        sizes = [fixed_size] * sample_count
    else:
        sizes = struct.unpack(f">{sample_count}I", tables["stsz"][12 : 12 + 4 * sample_count])
    if "co64" in tables:
        count = struct.unpack(">I", tables["co64"][4:8])[0]
        chunk_offsets = struct.unpack(f">{count}Q", tables["co64"][8 : 8 + 8 * count])
    else:
        count = struct.unpack(">I", tables["stco"][4:8])[0]
        chunk_offsets = struct.unpack(f">{count}I", tables["stco"][8 : 8 + 4 * count])
    count = struct.unpack(">I", tables["stsc"][4:8])[0]
    sample_to_chunk = [struct.unpack(">III", tables["stsc"][8 + 12 * i : 20 + 12 * i]) for i in range(count)]
    sample = 0
    for chunk_number, offset in enumerate(chunk_offsets, start=1):
        samples_in_chunk = [n for first_chunk, n, _ in sample_to_chunk if first_chunk <= chunk_number][-1]
        for _ in range(samples_in_chunk):
            if sample >= sample_count:
                return
            yield offset, sizes[sample]
            offset += sizes[sample]
            sample += 1


def _read_varint(buffer: bytes, position: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        byte = buffer[position]
        position += 1
        value |= (byte & 0x7F) << shift
        shift += 7
        if byte < 0x80:
            return value, position


def _iter_fields(buffer: bytes, base_offset: int = 0) -> Iterator[tuple[int, int, object, int]]:
    """Yield (field_number, wire_type, value, file_offset_of_value) for one protobuf message level."""
    position = 0
    while position < len(buffer):
        key, position = _read_varint(buffer, position)
        field_number, wire_type = key >> 3, key & 7
        value_offset = base_offset + position
        if wire_type == WIRE_VARINT:
            value, position = _read_varint(buffer, position)
        elif wire_type == WIRE_FIXED64:
            value, position = buffer[position : position + 8], position + 8
        elif wire_type == WIRE_BYTES:
            length, position = _read_varint(buffer, position)
            value_offset = base_offset + position
            value, position = buffer[position : position + length], position + length
        elif wire_type == WIRE_FIXED32:
            value, position = buffer[position : position + 4], position + 4
        else:
            raise TrackError(f"unsupported protobuf wire type {wire_type}")
        yield field_number, wire_type, value, value_offset


def _submessages(buffer: bytes, base_offset: int, field_number: int) -> list[tuple[bytes, int]]:
    return [
        (value, offset)
        for number, wire_type, value, offset in _iter_fields(buffer, base_offset)
        if number == field_number and wire_type == WIRE_BYTES
    ]


def _read_first_metadata_sample(path: str | os.PathLike) -> tuple[bytes, int]:
    with open(path, "rb") as file:
        offset, size = next(iter_samples(file, os.path.getsize(path), METADATA_SAMPLE_ENTRY))
        file.seek(offset)
        return file.read(size), offset


@dataclass
class AttitudeTrack:
    """Attitude quaternions [w, x, y, z] as stored in the file, duplicates included,
    with the file offsets of their float32 values and the video frame of each entry."""

    quats: np.ndarray
    offsets: np.ndarray
    frames: np.ndarray

    def unique(self) -> tuple[np.ndarray, np.ndarray]:
        """Unique 1 kHz samples, and for every stored entry the index of its unique sample."""
        is_new = np.r_[True, ~np.all(self.quats[1:] == self.quats[:-1], axis=1)]
        return self.quats[is_new], np.cumsum(is_new) - 1


def read_track(path: str | os.PathLike) -> AttitudeTrack:
    """Read the 1 kHz attitude quaternions embedded in a DJI O4 MP4."""
    quats, offsets, frames = [], [], []
    with open(path, "rb") as file:
        for frame, (sample_offset, size) in enumerate(iter_samples(file, os.path.getsize(path), METADATA_SAMPLE_ENTRY)):
            file.seek(sample_offset)
            sample = file.read(size)
            for frame_msg, frame_offset in _submessages(sample, sample_offset, FRAME_FIELD):
                for imu_msg, imu_offset in _submessages(frame_msg, frame_offset, IMU_FIELD):
                    for block_msg, block_offset in _submessages(imu_msg, imu_offset, ATTITUDE_BLOCK_FIELD):
                        for quat_msg, quat_offset in _submessages(block_msg, block_offset, QUATERNION_FIELD):
                            components = {
                                number: (struct.unpack("<f", value)[0], offset)
                                for number, wire_type, value, offset in _iter_fields(quat_msg, quat_offset)
                                if wire_type == WIRE_FIXED32 and number in QUATERNION_COMPONENT_FIELDS
                            }
                            if len(components) == 4:
                                quats.append([components[n][0] for n in QUATERNION_COMPONENT_FIELDS])
                                offsets.append([components[n][1] for n in QUATERNION_COMPONENT_FIELDS])
                                frames.append(frame)
    if not quats:
        raise TrackError("no 1 kHz attitude track in this file: record with RockSteady and HorizonSteady OFF")
    quats = np.asarray(quats, dtype=float)
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    if not np.all(np.isfinite(quats)) or np.any(norms < 0.5):
        raise TrackError("attitude track contains invalid quaternions")
    return AttitudeTrack(quats / norms, np.asarray(offsets, dtype=np.int64), np.asarray(frames))


def read_focal_px(path: str | os.PathLike) -> float:
    """Lens focal length in pixels at 3840 px width, from the clip metadata."""
    sample, sample_offset = _read_first_metadata_sample(path)
    for header, header_offset in _submessages(sample, sample_offset, HEADER_FIELD):
        for lens, lens_offset in _submessages(header, header_offset, LENS_FIELD):
            for number, wire_type, value, _ in _iter_fields(lens, lens_offset):
                if number == FOCAL_LENGTH_FIELD and wire_type == WIRE_FIXED32:
                    focal = struct.unpack("<f", value)[0]
                    if 300 < focal < 5000:
                        return focal
    return O4_PRO_FOCAL_PX


def write_track(
    src: str | os.PathLike, dst: str | os.PathLike, track: AttitudeTrack, quats: np.ndarray, overwrite: bool = False
) -> None:
    """Copy `src` to `dst` with the stored quaternions replaced by `quats`, keeping the file layout.

    The copy is built in a temporary file next to `dst` and renamed only when complete."""
    src, dst = os.fspath(src), os.fspath(dst)
    if os.path.abspath(src) == os.path.abspath(dst):
        raise ValueError("refusing to overwrite the input file; choose a different output path")
    if os.path.exists(dst) and not overwrite:
        raise FileExistsError(f"{dst} exists (use --force to overwrite)")
    if quats.shape != track.quats.shape:
        raise ValueError(f"expected quaternions of shape {track.quats.shape}, got {quats.shape}")
    handle, temp_path = tempfile.mkstemp(prefix=".o4fix-", suffix=".tmp", dir=os.path.dirname(os.path.abspath(dst)))
    os.close(handle)
    try:
        shutil.copyfile(src, temp_path)
        values = quats.astype("<f4")
        with open(temp_path, "r+b") as file:
            for entry in np.argsort(track.offsets[:, 0]):
                for component in range(4):
                    file.seek(int(track.offsets[entry, component]))
                    file.write(values[entry, component].tobytes())
        os.replace(temp_path, dst)
    except BaseException:
        if os.path.exists(temp_path):
            os.remove(temp_path)
        raise
