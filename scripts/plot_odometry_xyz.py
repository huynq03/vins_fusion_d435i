#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)

    if not rows:
        raise ValueError(f"No odometry samples found in {path}")

    timestamps = np.array([float(row["%time"]) for row in rows]) * 1e-9
    positions = np.array(
        [
            [float(row["field.x"]), float(row["field.y"]), float(row["field.z"])]
            for row in rows
        ]
    )
    return timestamps - timestamps[0], positions


def main():
    parser = argparse.ArgumentParser(description="Plot VINS odometry XYZ from rostopic CSV output.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    time_s, positions = load_csv(args.csv_path)
    displacement = positions - positions[0]
    output = args.output or args.csv_path.with_name(f"{args.csv_path.stem}_plot.png")

    colors = ("#2463A3", "#D17A22", "#66752D")
    labels = ("x", "y", "z")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True, constrained_layout=True)

    for index, (label, color) in enumerate(zip(labels, colors)):
        axes[0].plot(time_s, positions[:, index], label=label, color=color, linewidth=1.2)
        axes[1].plot(time_s, displacement[:, index], label=f"delta {label}", color=color, linewidth=1.2)

    axes[0].set_title("VINS odometry position")
    axes[0].set_ylabel("Position (m)")
    axes[1].set_title("Displacement from first sample")
    axes[1].set_xlabel("Elapsed time (s)")
    axes[1].set_ylabel("Delta position (m)")

    for axis in axes:
        axis.axhline(0.0, color="#30343B", linewidth=0.8, alpha=0.7)
        axis.grid(True, color="#D8DCE2", linewidth=0.7, alpha=0.8)
        axis.legend(ncol=3, frameon=False, loc="best")

    fig.savefig(output, dpi=160, facecolor="white")

    xy_stem = args.csv_path.stem.removesuffix("_xyz")
    xy_output = output.with_name(f"{xy_stem}_xy_plot.png")
    xy_fig, xy_axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
    xy_axis.plot(positions[:, 0], positions[:, 1], color="#2463A3", linewidth=1.4)
    xy_axis.scatter(
        positions[0, 0], positions[0, 1],
        color="#66752D", edgecolor="#30343B", s=70, label="Start", zorder=3,
    )
    xy_axis.scatter(
        positions[-1, 0], positions[-1, 1],
        color="#D17A22", edgecolor="#30343B", s=70, label="End", zorder=3,
    )
    xy_axis.set_title("VINS odometry X-Y trajectory")
    xy_axis.set_xlabel("X position (m)")
    xy_axis.set_ylabel("Y position (m)")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.grid(True, color="#D8DCE2", linewidth=0.7, alpha=0.8)
    xy_axis.legend(frameon=False)
    xy_fig.savefig(xy_output, dpi=160, facecolor="white")
    plt.close(xy_fig)

    duration = time_s[-1]
    final_delta = displacement[-1]
    ranges = np.ptp(positions, axis=0)
    std = np.std(positions, axis=0)
    rate = (len(time_s) - 1) / duration if duration > 0 else float("nan")
    print(f"saved: {output}")
    print(f"saved: {xy_output}")
    print(f"samples: {len(time_s)}, duration: {duration:.3f} s, average rate: {rate:.2f} Hz")
    for index, label in enumerate(labels):
        print(
            f"{label}: final delta={final_delta[index]:+.6f} m, "
            f"range={ranges[index]:.6f} m, std={std[index]:.6f} m"
        )


if __name__ == "__main__":
    main()
