import numpy as np
import pytest

from o4fix import TrackError, read_focal_px, read_track, write_track


def test_read_track_returns_unique_samples_in_order(synthetic_clip):
    path, expected = synthetic_clip
    track = read_track(path)
    assert track.quats.shape == (24, 4)
    unique, index = track.unique()
    np.testing.assert_allclose(unique, expected / np.linalg.norm(expected, axis=1, keepdims=True), atol=1e-7)
    np.testing.assert_array_equal(index, np.repeat(np.arange(12), 2))
    np.testing.assert_array_equal(track.frames, np.repeat([0, 1, 2], 8))


def test_read_focal_px(synthetic_clip):
    path, _ = synthetic_clip
    assert read_focal_px(path) == pytest.approx(1457.07, rel=1e-6)


def test_write_track_patches_values_and_keeps_layout(synthetic_clip, tmp_path):
    path, _ = synthetic_clip
    track = read_track(path)
    new = np.tile([1.0, 0.0, 0.0, 0.0], (len(track.quats), 1))
    new[::3] = [0.0, 1.0, 0.0, 0.0]
    out = tmp_path / "out.MP4"
    write_track(path, out, track, new)
    assert out.stat().st_size == path.stat().st_size
    again = read_track(out)
    np.testing.assert_allclose(again.quats, new, atol=1e-7)
    np.testing.assert_array_equal(again.offsets, track.offsets)
    original, patched = path.read_bytes(), out.read_bytes()
    changed = {i for i, (a, b) in enumerate(zip(original, patched)) if a != b}
    allowed = {int(o) + k for o in track.offsets.ravel() for k in range(4)}
    assert changed <= allowed


def test_write_track_refuses_input_and_existing_output(synthetic_clip, tmp_path):
    path, _ = synthetic_clip
    track = read_track(path)
    with pytest.raises(ValueError):
        write_track(path, path, track, track.quats)
    out = tmp_path / "exists.MP4"
    out.write_bytes(b"keep")
    with pytest.raises(FileExistsError):
        write_track(path, out, track, track.quats)
    assert out.read_bytes() == b"keep"
    write_track(path, out, track, track.quats, overwrite=True)
    assert out.stat().st_size == path.stat().st_size
    assert not list(tmp_path.glob(".o4fix-*"))


def test_file_without_track_raises(tmp_path):
    from conftest import build_mp4

    path = tmp_path / "empty.MP4"
    path.write_bytes(build_mp4([b"\x08\x01"]))
    with pytest.raises(TrackError):
        read_track(path)
