#!/usr/bin/env python3
"""
Plot ESP32 vibration stream data for threshold tuning.

Supports input from:
1) Clean CSV lines:
   sig_ms,adc,delta,base,td,rd,armed
   8925,1327,560,767,4095,1433,1
2) Raw serial logs that contain those CSV lines (e.g. stream_raw.log)
3) PlatformIO monitor style lines:
   19:18:34.287 > 8925,1327,560,767,4095,1433,1
"""

from __future__ import annotations

import argparse
import os
import re
from dataclasses import dataclass
from typing import List, Optional


STREAM_LINE = re.compile(r"(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),([01])\s*$")
ALIVE_LINE = re.compile(
    r"ms=(\d+),adc=(\d+),delta=(\d+),base=(\d+),td=(\d+),rd=(\d+),.*?armed=(YES|NO)"
)


@dataclass
class Samples:
    sig_ms: List[int]
    adc: List[int]
    delta: List[int]
    base: List[int]
    td: List[int]
    rd: List[int]
    armed: List[int]

    def __len__(self) -> int:
        return len(self.sig_ms)


def parse_stream_file(path: str) -> Samples:
    sig_ms: List[int] = []
    adc: List[int] = []
    delta: List[int] = []
    base: List[int] = []
    td: List[int] = []
    rd: List[int] = []
    armed: List[int] = []

    stream_count = 0
    alive_count = 0

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            # Strip monitor prefix like "19:18:34.287 > ".
            if ">" in line:
                line = line.split(">", 1)[1].strip()

            if line.startswith("sig_ms"):
                continue

            m = STREAM_LINE.fullmatch(line) or STREAM_LINE.search(line)
            if m:
                sig_ms.append(int(m.group(1)))
                adc.append(int(m.group(2)))
                delta.append(int(m.group(3)))
                base.append(int(m.group(4)))
                td.append(int(m.group(5)))
                rd.append(int(m.group(6)))
                armed.append(int(m.group(7)))
                stream_count += 1
                continue

            am = ALIVE_LINE.search(line)
            if am:
                sig_ms.append(int(am.group(1)))
                adc.append(int(am.group(2)))
                delta.append(int(am.group(3)))
                base.append(int(am.group(4)))
                td.append(int(am.group(5)))
                rd.append(int(am.group(6)))
                armed.append(1 if am.group(7) == "YES" else 0)
                alive_count += 1
                continue

    if not sig_ms:
        raise ValueError(
            f"No stream samples found in {path}. "
            "Make sure file contains either:\n"
            "  8925,1327,560,767,4095,1433,1\n"
            "or [alive] ms=...,adc=...,delta=...,base=...,td=...,rd=...,armed=YES"
        )

    if stream_count == 0 and alive_count > 0:
        print(
            "Detected [alive] heartbeat format only (about 1 Hz). "
            "For waveform-like detail, enable 'stream on' before logging."
        )

    return Samples(sig_ms, adc, delta, base, td, rd, armed)


def clip_range(samples: Samples, t_start: Optional[float], t_end: Optional[float]) -> Samples:
    t0_ms = samples.sig_ms[0]
    x_sec = [(t - t0_ms) / 1000.0 for t in samples.sig_ms]

    keep: List[int] = []
    for i, t in enumerate(x_sec):
        if t_start is not None and t < t_start:
            continue
        if t_end is not None and t > t_end:
            continue
        keep.append(i)

    if not keep:
        raise ValueError("No samples left after --t-start/--t-end clipping.")

    def pick(v: List[int]) -> List[int]:
        return [v[i] for i in keep]

    return Samples(
        sig_ms=pick(samples.sig_ms),
        adc=pick(samples.adc),
        delta=pick(samples.delta),
        base=pick(samples.base),
        td=pick(samples.td),
        rd=pick(samples.rd),
        armed=pick(samples.armed),
    )


def plot_samples(samples: Samples, output: str, show: bool) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "matplotlib is required. Run:\n"
            "  uv run --with matplotlib python scripts/plot_signal.py stream_raw.log -o signal_plot.png"
        ) from e

    t0 = samples.sig_ms[0]
    x = [(ms - t0) / 1000.0 for ms in samples.sig_ms]

    fig, (ax0, ax1) = plt.subplots(
        nrows=2, ncols=1, figsize=(12, 7), sharex=True, constrained_layout=True
    )

    # Top: raw signal and baseline.
    ax0.plot(x, samples.adc, label="adc", linewidth=1.0)
    ax0.plot(x, samples.base, label="base", linewidth=1.0, alpha=0.9)
    ax0.set_ylabel("ADC")
    ax0.set_title("ESP32 Vibration Signal")
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="upper right")

    # Bottom: delta and thresholds.
    ax1.plot(x, samples.delta, label="delta", linewidth=1.0)
    ax1.plot(x, samples.td, label="trigger td", linewidth=1.0, linestyle="--")
    ax1.plot(x, samples.rd, label="rearm rd", linewidth=1.0, linestyle=":")
    ax1.set_ylabel("Delta")
    ax1.set_xlabel("Time (s, relative)")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper right")

    # Mark armed/hold state in background.
    ax_state = ax1.twinx()
    ax_state.step(x, samples.armed, where="post", linewidth=0.8, alpha=0.35, label="armed")
    ax_state.set_ylim(-0.2, 1.2)
    ax_state.set_yticks([0, 1])
    ax_state.set_yticklabels(["HOLD", "ARM"])
    ax_state.set_ylabel("State")

    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    fig.savefig(output, dpi=160)
    print(f"Saved plot: {output}")

    if show:
        plt.show()
    plt.close(fig)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Plot vibration stream for threshold tuning.",
        epilog=(
            "Examples:\n"
            "  uv run python scripts/plot_signal.py stream_raw.log -o signal_plot.png\n"
            "  uv run python scripts/plot_signal.py signal_stream.csv --t-start 2 --t-end 12\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "input",
        nargs="?",
        default="signal_stream.csv",
        help="Input file (signal_stream.csv or stream_raw.log).",
    )
    p.add_argument(
        "-o",
        "--output",
        default="signal_plot.png",
        help="Output PNG path.",
    )
    p.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Start time in seconds (relative to first sample).",
    )
    p.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="End time in seconds (relative to first sample).",
    )
    p.add_argument(
        "--show",
        action="store_true",
        help="Show plot window in addition to saving PNG.",
    )
    return p


def main() -> None:
    args = build_arg_parser().parse_args()
    samples = parse_stream_file(args.input)
    samples = clip_range(samples, args.t_start, args.t_end)
    plot_samples(samples, args.output, args.show)


if __name__ == "__main__":
    main()
