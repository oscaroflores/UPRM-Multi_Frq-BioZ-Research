#!/usr/bin/env python3
"""
Plot IMU (accelerometer) and BioZ data stored in CSV logs.

The script expects two files that live next to this script by default:
    - imu-log-20251205-105535.csv (timestamp, ax, ay, az)
    - bioz-log-20251205-105535.csv (timestamp, Q, I, F_BIOZ)

Use `python GraphingScript.py` to show the plots or provide custom paths with
`--imu` and `--bioz` if needed.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.widgets import RangeSlider


REPO_DIR = Path(__file__).resolve().parent
DEFAULT_IMU = REPO_DIR / "imu-log-20251205-105535.csv"
DEFAULT_BIOZ = REPO_DIR / "bioz-log-20251205-105535.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Graph BioZ I/Q data alongside IMU accelerometer samples."
    )
    parser.add_argument("--imu", type=Path, default=DEFAULT_IMU, help="Path to the IMU CSV log.")
    parser.add_argument("--bioz", type=Path, default=DEFAULT_BIOZ, help="Path to the BioZ CSV log.")
    parser.add_argument(
        "--max-freq-plots",
        type=int,
        default=4,
        help="Maximum number of BioZ frequency groups to plot (sorted ascending).",
    )
    return parser.parse_args()


def _ensure_file(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Could not find file: {resolved}")
    return resolved


def _get_first_line(path: Path) -> str:
    with path.open("r", encoding="utf-8") as handle:
        return handle.readline().strip()


def load_imu_data(path: Path) -> pd.DataFrame:
    """
    Load IMU samples while being tolerant to logs missing the timestamp header.
    """
    path = _ensure_file(path)
    first_line = _get_first_line(path).lower()
    has_timestamp_header = "timestamp" in first_line
    if has_timestamp_header:
        imu_df = pd.read_csv(path, usecols=[0, 1, 2, 3])
        imu_df.columns = ["timestamp", "ax", "ay", "az"]
    else:
        # Some logs only include "ax,ay,az" as the header. Skip it and add the timestamp label.
        imu_df = pd.read_csv(path, names=["timestamp", "ax", "ay", "az"], skiprows=1)

    imu_df = imu_df.dropna(subset=["timestamp"])
    imu_df = imu_df.apply(pd.to_numeric, errors="coerce").dropna()
    imu_df = imu_df.sort_values("timestamp").reset_index(drop=True)
    imu_df["time_s"] = (imu_df["timestamp"] - imu_df["timestamp"].iloc[0]) / 1000.0
    return imu_df


def load_bioz_data(path: Path) -> pd.DataFrame:
    """
    Load BioZ samples and expose a consistent F_BIOZ column.
    """
    path = _ensure_file(path)
    bioz_df = pd.read_csv(path)
    if "F_BIOZ" not in bioz_df.columns:
        bioz_df = bioz_df.rename(columns={"F": "F_BIOZ"})

    bioz_df = bioz_df.dropna(subset=["timestamp", "I", "Q", "F_BIOZ"])
    convert_cols = ["timestamp", "I", "Q", "F_BIOZ"]
    bioz_df[convert_cols] = bioz_df[convert_cols].apply(pd.to_numeric, errors="coerce")
    bioz_df = bioz_df.dropna().sort_values("timestamp").reset_index(drop=True)
    bioz_df["time_s"] = (bioz_df["timestamp"] - bioz_df["timestamp"].iloc[0]) / 1000.0
    return bioz_df


def _plot_channel_with_imu(
    imu_time: pd.Series,
    imu_df: pd.DataFrame,
    channel_time: pd.Series,
    channel_values: pd.Series,
    title: str,
) -> None:
    axes_labels = ["ax", "ay", "az"]
    fig, axes = plt.subplots(4, 1, figsize=(14, 9), sharex=True)
    fig.subplots_adjust(bottom=0.18, hspace=0.08)

    for axis, label in zip(axes[:3], axes_labels):
        axis.plot(imu_time, imu_df[label], linewidth=0.9)
        axis.set_ylabel(f"{label} (g)")
        axis.grid(True, which="both", alpha=0.3)

    axes[0].set_title(title)
    axes[3].plot(channel_time, channel_values, linewidth=0.9, color="tab:red")
    axes[3].set_ylabel("BioZ")
    axes[3].grid(True, which="both", alpha=0.3)
    axes[3].set_xlabel("Time (s from first IMU sample)")

    time_min = min(imu_time.min(), channel_time.min())
    time_max = max(imu_time.max(), channel_time.max())
    slider_ax = fig.add_axes([0.12, 0.06, 0.76, 0.04])
    slider = RangeSlider(
        ax=slider_ax,
        label="Time window (s)",
        valmin=time_min,
        valmax=time_max,
        facecolor="tab:blue",
    )

    def _set_axis_y_limits(axis: plt.Axes, data: pd.Series) -> None:
        if data.empty:
            return
        y_min = data.min()
        y_max = data.max()
        if y_min == y_max:
            padding = 0.05 * (abs(y_min) if y_min else 1.0)
        else:
            padding = 0.05 * (y_max - y_min)
        axis.set_ylim(y_min - padding, y_max + padding)

    def update_view(xmin: float, xmax: float) -> None:
        for axis in axes:
            axis.set_xlim(xmin, xmax)

        mask = (imu_time >= xmin) & (imu_time <= xmax)
        for axis, label in zip(axes[:3], axes_labels):
            _set_axis_y_limits(axis, imu_df.loc[mask, label])

        channel_mask = (channel_time >= xmin) & (channel_time <= xmax)
        _set_axis_y_limits(axes[3], channel_values.loc[channel_mask])

        fig.canvas.draw_idle()

    def on_slider_change(_: tuple[float, float]) -> None:
        xmin, xmax = slider.val
        update_view(xmin, xmax)

    slider.on_changed(on_slider_change)
    update_view(*slider.val)
    plt.show()


def plot_data(
    imu_df: pd.DataFrame,
    bioz_df: pd.DataFrame,
    max_freq_plots: int = 4,
) -> None:
    shared_time_start = min(imu_df["time_s"].min(), bioz_df["time_s"].min())
    imu_time = imu_df["time_s"] - shared_time_start
    bioz_df = bioz_df.assign(aligned_time=bioz_df["time_s"] - shared_time_start)

    unique_freqs = sorted(bioz_df["F_BIOZ"].unique())
    if not unique_freqs:
        raise ValueError("BioZ CSV does not contain any frequency values to plot.")

    freq_subset = unique_freqs[:max_freq_plots]
    for freq in freq_subset:
        freq_df = bioz_df[bioz_df["F_BIOZ"] == freq]
        freq_label = f"{freq/1000:.1f} kHz" if freq >= 1000 else f"{int(freq)} Hz"
        for channel in ("I", "Q"):
            channel_title = f"{channel} @ {freq_label} with accelerometer traces"
            _plot_channel_with_imu(
                imu_time=imu_time,
                imu_df=imu_df,
                channel_time=freq_df["aligned_time"],
                channel_values=freq_df[channel],
                title=channel_title,
            )


def main() -> None:
    args = parse_args()
    imu_df = load_imu_data(args.imu)
    bioz_df = load_bioz_data(args.bioz)
    plot_data(imu_df, bioz_df, max_freq_plots=args.max_freq_plots)


if __name__ == "__main__":
    main()
