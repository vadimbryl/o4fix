#!/usr/bin/env python3
"""Rebuild the figures in docs/figures from the recordings described in docs/REPORT.md.

Needs the tools extra (`pip install -e ".[tools]"`). Annotation positions are tuned for
those specific recordings.

    python scripts/make_figures.py --bench-clip CLIP.MP4 --bench-log LOG.BFL \
                                   --flight-clip CLIP.MP4 --flight-log LOG.BBL
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.ndimage import maximum_filter1d  # noqa: E402
from scipy.signal import butter, sosfiltfilt  # noqa: E402

from o4fix import QUAT_RATE_HZ, correct, quat, read_track  # noqa: E402
from o4fix.blackbox import load_blackbox, synchronise  # noqa: E402
from o4fix.static_check import video_motion  # noqa: E402

# Light-mode reference palette: series slots 1-3, then ink, grid and surface tokens.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"
OUT = Path(__file__).resolve().parent.parent / "docs" / "figures"


def style(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=9, length=3)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.yaxis.label.set_color(INK2)
    ax.xaxis.label.set_color(INK2)


def label_line_end(ax, x, y, text, color, dy=0) -> None:
    ax.plot([x], [y], "o", ms=4, color=color, zorder=5)
    ax.annotate(text, (x, y), xytext=(6, dy), textcoords="offset points", va="center", fontsize=9, color=INK)


def two_panels(height):
    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(9, height), sharex=True, gridspec_kw={"height_ratios": [3, 1.2], "hspace": 0.12}
    )
    fig.patch.set_facecolor(SURFACE)
    style(top)
    style(bottom)
    return fig, top, bottom


def fastest_motor_hz(log: Path, poles: int = 14) -> tuple[np.ndarray, np.ndarray]:
    from orangebox import Parser

    parser = Parser.load(str(log))
    names = parser.field_names
    cols = [names.index("time")] + [names.index(f"eRPM[{k}]") for k in range(4)]
    data = np.asarray([[f.data[c] for c in cols] for f in parser.frames()], dtype=float)
    return (data[:, 0] - data[0, 0]) / 1e6, (data[:, 1:] * 100 / (poles / 2) / 60).max(axis=1)


def bench_figure(clip: Path, log: Path, out: Path, show_fixed: bool = True) -> None:
    samples, _ = read_track(clip).unique()
    recorded = quat.relative_attitude_deg(samples)
    fixed = quat.relative_attitude_deg(correct(samples, QUAT_RATE_HZ).quats)
    t = np.arange(len(recorded)) / QUAT_RATE_HZ
    video = video_motion(clip, 0.1)
    t_motor, motor = fastest_motor_hz(log)
    video_color = AQUA if show_fixed else ORANGE

    fig, top, bottom = two_panels(5.6)
    every_10th = slice(None, None, 10)
    top.plot(t[every_10th], recorded[every_10th, 0], color=BLUE, lw=1.6, label="Camera gyro, as recorded")
    if show_fixed:
        top.plot(t[every_10th], fixed[every_10th, 0], color=ORANGE, lw=1.6, label="Camera gyro, after o4fix")
    top.plot(video.time_s, video.motion_deg, color=video_color, lw=1.6, label="Camera motion measured in the video")
    top.set_ylabel("Rotation about camera X (deg)", fontsize=9)
    top.set_ylim(-15, 4.5)
    top.annotate(
        "false 12° in 0.2 s",
        (20.05, recorded[int(20.2 * QUAT_RATE_HZ), 0]),
        xytext=(9.5, -12.6),
        fontsize=9,
        color=INK,
        arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8},
    )
    top.annotate(
        "fusion pulls it back, τ ≈ 12.5 s",
        (32, recorded[int(32 * QUAT_RATE_HZ), 0]),
        xytext=(30.5, -10.6),
        fontsize=9,
        color=INK,
        arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8},
    )
    label_line_end(top, t[-1], recorded[-1, 0], "as recorded", BLUE, dy=-6)
    if show_fixed:
        label_line_end(top, t[-1], fixed[-1, 0], "after o4fix", ORANGE, dy=6)
    label_line_end(top, video.time_s[-1], video.motion_deg[-1], "video", video_color)
    top.legend(loc="upper center", ncol=3 if show_fixed else 2, fontsize=9, frameon=False, labelcolor=INK)
    top.set_title(
        "Static camera, propellers removed, motors only: the gyro reports 12° that never happened",
        loc="left",
        fontsize=11,
        color=INK,
        pad=10,
    )
    bottom.plot(t_motor, motor, color=BLUE, lw=0.9)
    bottom.set_ylabel("Fastest motor (Hz)", fontsize=9)
    bottom.set_xlabel("Time (s)", fontsize=9)
    bottom.set_xlim(0, t[-1] + 6)
    fig.text(
        0.01,
        0.005,
        "Data: O4 Air Unit Pro (ICM-40609-D), RockSteady off; video motion from feature tracking; "
        "motor frequency from Betaflight eRPM (log start aligned to clip start, ±1 s).",
        fontsize=7.5,
        color=INK2,
    )
    fig.savefig(out, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(
        f"written {out} (gyro max {np.abs(recorded[:, 0]).max():.2f} deg, video max {video.motion_deg.max():.3f} deg)"
    )


def flight_figure(clip: Path, log: Path, out: Path) -> None:
    samples, _ = read_track(clip).unique()
    rates = quat.body_rates(samples, QUAT_RATE_HZ)
    fixed = quat.body_rates(correct(samples, QUAT_RATE_HZ).quats, QUAT_RATE_HZ)
    blackbox = load_blackbox(log)
    sync = synchronise(rates, blackbox)
    t = (np.arange(len(rates)) + 0.5) / QUAT_RATE_HZ
    t_fc = sync.blackbox_time(t)
    valid = (t_fc > 0.5) & (t_fc < blackbox.time_s[-1] - 0.5)
    fc_in_camera_axes = (
        np.stack([np.interp(t_fc, blackbox.time_s, blackbox.gyro_dps[:, k]) for k in range(3)], axis=1) @ sync.rotation
    )
    highpass = butter(2, 0.1 / (QUAT_RATE_HZ / 2), "high", output="sos")

    def false_rotation_y_deg(rates):
        return sosfiltfilt(
            highpass, np.cumsum(np.where(valid, rates[:, 1] - fc_in_camera_axes[:, 1], 0.0)) / QUAT_RATE_HZ
        )

    before, after = false_rotation_y_deg(rates), false_rotation_y_deg(fixed)
    from orangebox import Parser

    parser = Parser.load(str(log), 1)
    names = parser.field_names
    cols = [names.index("time")] + [names.index(f"accSmooth[{k}]") for k in range(3)]
    acc = np.asarray([[f.data[c] for c in cols] for f in parser.frames()], dtype=float)
    t_acc = ((acc[:, 0] - acc[0, 0]) / 1e6 - sync.offset_s) / sync.clock_ratio
    load = maximum_filter1d(np.linalg.norm(acc[:, 1:], axis=1) / float(parser.headers.get("acc_1G", 2048)), 200)

    fig, top, bottom = two_panels(5.2)
    every_20th = slice(None, None, 20)
    keep = valid[every_20th]
    top.plot(t[every_20th][keep], before[every_20th][keep], color=BLUE, lw=1.2, label="as recorded")
    top.plot(t[every_20th][keep], after[every_20th][keep], color=ORANGE, lw=1.4, label="after o4fix")
    top.axhline(0, color=MUTED, lw=0.8)
    top.set_ylabel("False rotation, camera Y (deg)", fontsize=9)
    top.legend(loc="upper right", fontsize=9, frameon=False, labelcolor=INK, ncol=2)
    top.set_title(
        "Flight: camera rotation the airframe never made (camera minus blackbox gyro, > 0.1 Hz)",
        loc="left",
        fontsize=11,
        color=INK,
        pad=10,
    )
    bottom.plot(t_acc, load, color=BLUE, lw=1.2)
    bottom.set_ylabel("Peak load (g)", fontsize=9)
    bottom.set_xlabel("Video time (s)", fontsize=9)
    bottom.set_xlim(0, t[valid][-1])
    fig.text(
        0.01,
        0.005,
        "Data: public O4 Air Unit Pro sample (Feb 2026 batch) with Betaflight blackbox, gyro 8 kHz, "
        f"RockSteady off; sync correlation {sync.correlation:.2f}.",
        fontsize=7.5,
        color=INK2,
    )
    fig.savefig(out, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    rms_before, rms_after = (np.sqrt((x[valid] ** 2).mean()) for x in (before, after))
    print(f"written {out} (rms as recorded {rms_before:.2f} deg, after {rms_after:.2f} deg)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bench-clip", type=Path, required=True)
    parser.add_argument("--bench-log", type=Path, required=True)
    parser.add_argument("--flight-clip", type=Path, required=True)
    parser.add_argument("--flight-log", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    bench_figure(args.bench_clip, args.bench_log, args.out / "bench_static_camera.png")
    bench_figure(args.bench_clip, args.bench_log, args.out / "bench_static_camera_gyro_only.png", show_fixed=False)
    flight_figure(args.flight_clip, args.flight_log, args.out / "flight_false_rotation.png")


if __name__ == "__main__":
    main()
