#!/usr/bin/env python3
"""Plot raw VINS odometry against the PX4 (EKF2) fused odometry from record_odom.sh CSVs.

Input is an output/odom_logs/<time>/ directory with vins_odom.csv and fc_odom.csv.
Panels: x, y, z and yaw over time plus the X-Y trajectory, VINS and FC overlaid.
Yaw of VINS is read from vins_flu_odom.csv (body FLU, like the FC); the raw VINS body
is RDF, so its yaw is not comparable. Run inside the container.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

VINS_COLOR, FC_COLOR = "#2a78d6", "#eb6834"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e3e2dc"
COLS = ("time_s", "x_m", "y_m", "z_m", "yaw_deg")


def read_csv(path):
    with path.open(newline="") as stream:
        rows = [[float(row[c]) for c in COLS] for row in csv.DictReader(stream)]
    if not rows:
        raise SystemExit("No samples in {}".format(path))
    data = np.asarray(rows, dtype=float)
    return data[np.argsort(data[:, 0])]


def unwrap_deg(yaw):
    return np.degrees(np.unwrap(np.radians(yaw)))


def rms_diff(t_a, a, t_b, b):
    """RMS of b - a over the common time span, a interpolated onto the stamps of b."""
    mask = (t_b >= t_a[0]) & (t_b <= t_a[-1])
    if not mask.any():
        return float("nan")
    return float(np.sqrt(np.mean((b[mask] - np.interp(t_b[mask], t_a, a)) ** 2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", type=Path, help="output/odom_logs/<time> directory")
    parser.add_argument("--output", type=Path, help="PNG path (default: DIR/odom_compare.png)")
    parser.add_argument("--relative", action="store_true",
                        help="plot positions relative to each source's first sample")
    args = parser.parse_args()

    src = args.directory.expanduser().resolve()
    for name in ("vins_odom.csv", "fc_odom.csv"):
        if not (src / name).is_file():
            parser.error("missing {}".format(src / name))
    vins, fc = read_csv(src / "vins_odom.csv"), read_csv(src / "fc_odom.csv")
    flu_path = src / "vins_flu_odom.csv"
    flu = read_csv(flu_path) if flu_path.is_file() else None

    t0 = min(vins[0, 0], fc[0, 0])
    if args.relative:
        vins[:, 1:4] -= vins[0, 1:4]
        fc[:, 1:4] -= fc[0, 1:4]
    tv, tf = vins[:, 0] - t0, fc[:, 0] - t0

    plt.rcParams.update({"text.color": INK, "axes.labelcolor": MUTED, "axes.edgecolor": GRID,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlesize": 11,
                         "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(15, 9), constrained_layout=True)
    # Row 0 holds the figure title and legend so they cannot overlap the panels.
    grid = fig.add_gridspec(5, 2, width_ratios=(3, 2), height_ratios=(0.18, 1, 1, 1, 1))
    ax_head = fig.add_subplot(grid[0, :])
    ax_head.axis("off")
    time_axes = [fig.add_subplot(grid[1, 0])]
    time_axes += [fig.add_subplot(grid[i, 0], sharex=time_axes[0]) for i in (2, 3, 4)]
    ax_xy = fig.add_subplot(grid[1:, 1])

    print("RMS difference FC - VINS over the common time span:")
    for i, (ax, axis) in enumerate(zip(time_axes, "XYZ"), start=1):
        rms = rms_diff(tv, vins[:, i], tf, fc[:, i])
        ax.plot(tv, vins[:, i], color=VINS_COLOR, linewidth=1.6, label="VINS raw")
        ax.plot(tf, fc[:, i], color=FC_COLOR, linewidth=1.6, label="FC fused (EKF2)")
        ax.set_title("{} position   RMS diff {:.3f} m".format(axis, rms), loc="left")
        ax.set_ylabel("m")
        print("  {}: {:.4f} m".format(axis.lower(), rms))

    ax_yaw = time_axes[3]
    fc_yaw = unwrap_deg(fc[:, 4])
    if flu is not None:
        flu_t, flu_yaw = flu[:, 0] - t0, unwrap_deg(flu[:, 4])
        # Both unwrapped series start within +-180 deg; keep them on the same turn.
        flu_yaw += 360.0 * np.round((np.interp(flu_t[0], tf, fc_yaw) - flu_yaw[0]) / 360.0)
        rms = rms_diff(flu_t, flu_yaw, tf, fc_yaw)
        ax_yaw.plot(flu_t, flu_yaw, color=VINS_COLOR, linewidth=1.6)
        ax_yaw.set_title("Yaw   RMS diff {:.2f} deg".format(rms), loc="left")
        print("  yaw: {:.3f} deg".format(rms))
    else:
        ax_yaw.set_title("Yaw   (FC only: vins_flu_odom.csv not found)", loc="left")
    ax_yaw.plot(tf, fc_yaw, color=FC_COLOR, linewidth=1.6)
    ax_yaw.set(xlabel="Elapsed time (s)", ylabel="deg")
    for ax in time_axes[:3]:
        ax.tick_params(labelbottom=False)

    for data, color in ((vins, VINS_COLOR), (fc, FC_COLOR)):
        ax_xy.plot(data[:, 1], data[:, 2], color=color, linewidth=1.6)
        ax_xy.scatter(data[0, 1], data[0, 2], marker="o", s=70, color=color, edgecolor="white", linewidth=1.5, zorder=3)
        ax_xy.scatter(data[-1, 1], data[-1, 2], marker="s", s=70, color=color, edgecolor="white", linewidth=1.5, zorder=3)
    ax_xy.set_title("X-Y trajectory   (circle = start, square = end)", loc="left")
    ax_xy.set(xlabel="X (m)", ylabel="Y (m)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.xaxis.set_major_locator(MaxNLocator(6))

    for ax in time_axes + [ax_xy]:
        ax.grid(True, color=GRID, linewidth=0.7)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    handles, labels = time_axes[0].get_legend_handles_labels()
    ax_head.legend(handles, labels, loc="center right", ncol=2, frameon=False, fontsize=11)
    ax_head.text(0.0, 0.5, "VINS raw vs FC fused odometry: {}{}".format(
        src.name, " (relative to start)" if args.relative else ""), fontsize=14, va="center")

    output = args.output or src / "odom_compare.png"
    fig.savefig(str(output), dpi=160, facecolor="white")
    print("saved: {}".format(output))
    print("samples: VINS {} ({:.1f} s), FC {} ({:.1f} s)".format(len(tv), tv[-1] - tv[0], len(tf), tf[-1] - tf[0]))


if __name__ == "__main__":
    main()
