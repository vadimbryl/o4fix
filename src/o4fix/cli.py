"""Command line: `o4fix fix | check-static | compare-blackbox`."""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

from . import __version__
from .correction import Settings, correct
from .dji import QUAT_RATE_HZ, TrackError, read_track, write_track


def _add_correction_options(parser: argparse.ArgumentParser) -> None:
    defaults = Settings()
    parser.add_argument(
        "--impulse-thr",
        type=float,
        default=defaults.impulse_threshold_dps,
        help=f"impulse threshold above the running median, deg/s (default {defaults.impulse_threshold_dps:g})",
    )
    parser.add_argument(
        "--lpf", type=float, default=defaults.lowpass_hz, help=f"low-pass cutoff, Hz (default {defaults.lowpass_hz:g})"
    )
    parser.add_argument(
        "--busy-lpf",
        type=float,
        default=defaults.busy_lowpass_hz,
        help=f"low-pass cutoff where false content is dense, Hz (default {defaults.busy_lowpass_hz:g})",
    )
    parser.add_argument(
        "--pullback-tau",
        type=float,
        default=defaults.pullback_tau_s,
        help=f"time constant of the fusion pull-back to remove, s (default {defaults.pullback_tau_s:g})",
    )


def _settings_from(args: argparse.Namespace) -> Settings:
    return Settings(args.impulse_thr, args.lpf, args.busy_lpf, args.pullback_tau)


def _default_output(path: str) -> str:
    root, extension = os.path.splitext(path)
    return f"{root}_o4fix{extension}"


def _fix(args: argparse.Namespace) -> int:
    output = args.output or _default_output(args.input)
    if os.path.abspath(output) == os.path.abspath(args.input):
        raise ValueError("refusing to overwrite the input file; choose a different output path")
    track = read_track(args.input)
    samples, sample_of_entry = track.unique()
    duration_s = len(samples) / QUAT_RATE_HZ
    print(f"{os.path.basename(args.input)}: {duration_s:.1f} s, {len(samples)} attitude samples")

    result = correct(samples, QUAT_RATE_HZ, _settings_from(args))
    removed_deg = np.abs([burst.area_deg for burst in result.bursts]) if result.bursts else np.zeros(1)
    per_axis = [sum(burst.axis == axis for burst in result.bursts) for axis in range(3)]
    print(
        f"false bursts removed: {len(result.bursts)}, per axis X/Y/Z {per_axis}, "
        f"total {removed_deg.sum():.1f} deg, largest {removed_deg.max():.2f} deg"
    )
    if args.dry_run:
        return 0
    print(f"writing {output} ...", flush=True)
    write_track(args.input, output, track, result.quats[sample_of_entry], overwrite=args.force)
    print("done")
    return 0


def _check_static(args: argparse.Namespace) -> int:
    from .static_check import check_static

    check_static(args.clip, args.step, args.csv)
    return 0


def _compare_blackbox(args: argparse.Namespace) -> int:
    from .blackbox import compare_blackbox

    compare_blackbox(args.clip, args.blackbox, args.log_index, _settings_from(args))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="o4fix",
        description="Repair and check the gyro track of DJI O4 Air Unit (Pro) videos with the ICM-40609-D gyro. "
        "Clips must be recorded with RockSteady and HorizonSteady OFF.",
    )
    parser.add_argument("--version", action="version", version=f"o4fix {__version__}")
    commands = parser.add_subparsers(dest="command", required=True)

    fix = commands.add_parser("fix", help="write a copy of the clip with the false rotations removed")
    fix.add_argument("input", help="original DJI O4 MP4")
    fix.add_argument("-o", "--output", help="output MP4 (default: <input>_o4fix.MP4)")
    fix.add_argument("-f", "--force", action="store_true", help="overwrite the output file if it exists")
    fix.add_argument("--dry-run", action="store_true", help="analyse only, write nothing")
    _add_correction_options(fix)
    fix.set_defaults(handler=_fix)

    static = commands.add_parser("check-static", help="static camera: recorded attitude against motion in the video")
    static.add_argument("clip")
    static.add_argument("--step", type=float, default=0.1, help="video sampling step, s (default 0.1)")
    static.add_argument("--csv", help="also write the time series to this CSV file")
    static.set_defaults(handler=_check_static)

    blackbox = commands.add_parser("compare-blackbox", help="flight: false rotation against a Betaflight blackbox")
    blackbox.add_argument("clip")
    blackbox.add_argument("blackbox", help="Betaflight .bbl/.bfl log")
    blackbox.add_argument("--log-index", type=int, default=1, help="log number inside the file (default 1)")
    _add_correction_options(blackbox)
    blackbox.set_defaults(handler=_compare_blackbox)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.handler(args)
    except ImportError as error:
        missing = error.name or error
        print(
            f"error: {missing} is not installed; install the tools extra: pip install 'o4fix[tools]'", file=sys.stderr
        )
    except (TrackError, FileExistsError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
