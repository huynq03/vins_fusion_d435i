#!/usr/bin/env python3
"""Plot raw VINS odometry against the PX4 (EKF2) fused odometry from record_odom.sh CSVs.

Input is an output/odom_logs/<time>/ directory with vins_odom_<time>.csv and fc_odom_<time>.csv.
Panels: x, y, z and yaw over time plus the X-Y trajectory, VINS and FC overlaid.
Yaw of VINS is read from vins_flu_odom_<time>.csv (body FLU, like the FC); the raw VINS body
is RDF, so its yaw is not comparable. Run inside the container.

If gps_raw_<time>.csv has samples with a fix, the raw GPS / UWB position is overlaid as a
third source. Its frame (east/north from its own origin) is unrelated to the VINS world
(arbitrary initial yaw), so it is first fitted onto the VINS X-Y track with one rotation and
one translation (no scale); the fitted yaw is printed. --no-gps-align plots it unfitted.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

VINS_COLOR, FC_COLOR, GPS_COLOR = "#2a78d6", "#eb6834", "#1f9d6b"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e3e2dc"
COLS = ("time_s", "x_m", "y_m", "z_m", "yaw_deg")
GPS_COLS = COLS[:4]
# Below this X-Y spread the GPS track is too short to fit a rotation onto VINS.
MIN_ALIGN_SPREAD_M = 0.3


def read_csv(path, cols=COLS, required=True):
    with path.open(newline="") as stream:
        # gps_raw rows before the first fix have empty x/y/z.
        rows = [[float(row[c]) for c in cols] for row in csv.DictReader(stream) if all(row[c] for c in cols)]
    if not rows:
        if required:
            raise SystemExit("No samples in {}".format(path))
        return None
    data = np.asarray(rows, dtype=float)
    return data[np.argsort(data[:, 0])]


def align_xy(ref, data):
    """Rotate and translate the x/y of data onto ref (least squares over the common time span); shift z.

    Returns the applied yaw in degrees, or None when the two do not overlap in time.
    """
    t = data[:, 0]
    mask = (t >= ref[0, 0]) & (t <= ref[-1, 0])
    if mask.sum() < 2:
        return None
    src = data[mask, 1:3]
    dst = np.column_stack([np.interp(t[mask], ref[:, 0], ref[:, i]) for i in (1, 2)])
    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - src_mean, dst - dst_mean
    yaw = 0.0
    if np.sqrt((a ** 2).sum(axis=1).mean()) >= MIN_ALIGN_SPREAD_M:
        yaw = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    data[:, 1:3] = (data[:, 1:3] - src_mean) @ rot.T + dst_mean
    data[:, 3] += np.interp(t[mask], ref[:, 0], ref[:, 3]).mean() - data[mask, 3].mean()  # z: offset only
    return float(np.degrees(yaw))


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
    parser.add_argument("--output", type=Path, help="PNG path (default: DIR/odom_compare_<time>.png)")
    parser.add_argument("--relative", action="store_true",
                        help="plot positions relative to each source's first sample")
    parser.add_argument("--no-gps-align", action="store_true",
                        help="do not fit the raw GPS track onto the VINS track")
    args = parser.parse_args()

    src = args.directory.expanduser().resolve()
    # Files are NAME_<YYYYmmdd_HHMMSS>.csv (NAME.csv in older logs); take the newest recording.
    stamped = sorted(src.glob("vins_odom_" + "[0-9]" * 8 + "_" + "[0-9]" * 6 + ".csv"))
    suffix = stamped[-1].stem[len("vins_odom"):] if stamped else ""
    paths = [src / "{}{}.csv".format(name, suffix) for name in ("vins_odom", "fc_odom", "vins_flu_odom")]
    for path in paths[:2]:
        if not path.is_file():
            parser.error("missing {}".format(path))
    vins, fc = read_csv(paths[0]), read_csv(paths[1])
    flu = read_csv(paths[2], required=False) if paths[2].is_file() else None  # empty when the bridge was not running
    gps_path = src / "gps_raw{}.csv".format(suffix)
    gps = read_csv(gps_path, GPS_COLS, required=False) if gps_path.is_file() else None

    t0 = min(vins[0, 0], fc[0, 0])
    if args.relative:
        vins[:, 1:4] -= vins[0, 1:4]
        fc[:, 1:4] -= fc[0, 1:4]
    gps_label = "GPS raw"
    if gps is not None:
        gps_yaw = None if args.no_gps_align else align_xy(vins, gps)
        if gps_yaw is not None:
            gps_label = "GPS raw (fitted to VINS, yaw {:+.1f} deg)".format(gps_yaw)
        elif args.relative:
            gps[:, 1:4] -= gps[0, 1:4]
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
        title = "{} position   RMS diff FC-VINS {:.3f} m".format(axis, rms)
        print("  {}: {:.4f} m".format(axis.lower(), rms))
        if gps is not None:
            tg = gps[:, 0] - t0
            ax.plot(tg, gps[:, i], color=GPS_COLOR, linewidth=1.2, label=gps_label, zorder=1)  # under VINS / FC
            gps_rms = [rms_diff(t, ref, tg, gps[:, i]) for t, ref in ((tv, vins[:, i]), (tf, fc[:, i]))]
            title += ", GPS-VINS {:.3f} m, GPS-FC {:.3f} m".format(*gps_rms)
            print("     GPS - VINS {:.4f} m, GPS - FC {:.4f} m".format(*gps_rms))
        ax.set_title(title, loc="left")
        ax.set_ylabel("m")

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
        ax_yaw.set_title("Yaw   (FC only: no samples in {})".format(paths[2].name), loc="left")
    ax_yaw.plot(tf, fc_yaw, color=FC_COLOR, linewidth=1.6)
    ax_yaw.set(xlabel="Elapsed time (s)", ylabel="deg")
    for ax in time_axes[:3]:
        ax.tick_params(labelbottom=False)

    for data, color in ((vins, VINS_COLOR), (fc, FC_COLOR)) + (((gps, GPS_COLOR),) if gps is not None else ()):
        ax_xy.plot(data[:, 1], data[:, 2], color=color, linewidth=1.2 if data is gps else 1.6,
                   zorder=1 if data is gps else 2)
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
    ax_head.legend(handles, labels, loc="center right", ncol=len(handles), frameon=False, fontsize=11)
    ax_head.text(0.0, 0.5, "VINS raw vs FC fused{}: {}{}".format(
        " vs GPS raw" if gps is not None else " odometry",
        src.name, " (relative to start)" if args.relative else ""), fontsize=14, va="center")

    output = args.output or src / "odom_compare{}.png".format(suffix)
    fig.savefig(str(output), dpi=160, facecolor="white")
    print("saved: {}".format(output))
    print("samples: VINS {} ({:.1f} s), FC {} ({:.1f} s)".format(len(tv), tv[-1] - tv[0], len(tf), tf[-1] - tf[0]))
    if gps is not None:
        print("         GPS {} ({:.1f} s), {}".format(len(gps), gps[-1, 0] - gps[0, 0], gps_label))
    elif gps_path.is_file():
        print("         GPS: no samples with a fix in {}".format(gps_path.name))


if __name__ == "__main__":
    main()
