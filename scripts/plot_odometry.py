#!/usr/bin/env python3
"""Plot nav_msgs/Odometry (position, velocity, X-Y and 3D trajectory) from a bag or CSV.

Input is either a ROS 1 .bag or a CSV written by `rostopic echo -p` of the odometry
topic or of its .../pose/pose/position field, or by record_odom_csv.py. Run bag input
inside the container.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers the "3d" projection
import numpy as np

COLORS = ("#2463A3", "#D17A22", "#66752D")
AXES = ("x", "y", "z")


def read_bag(path, topic):
    import rosbag

    rows = []
    with rosbag.Bag(str(path)) as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp if not msg.header.stamp.is_zero() else bag_time
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            rows.append((stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z))
    if not rows:
        raise SystemExit("No odometry messages on {} in {}".format(topic, path))
    return np.asarray(rows, dtype=float)


def read_csv(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("No samples in {}".format(path))
    if "time_s" in rows[0]:  # written by record_odom_csv.py or by --csv below
        cols = ["time_s"] + [a + "_m" for a in AXES] + ["v" + a + "_mps" for a in AXES]
        return np.asarray([[float(row[c]) for c in cols] for row in rows], dtype=float)
    # `rostopic echo -p` of the whole Odometry vs. of its position field.
    prefix = "field.pose.pose.position." if "field.pose.pose.position.x" in rows[0] else "field."
    vel = "field.twist.twist.linear."
    has_vel = vel + "x" in rows[0]
    data = []
    for row in rows:
        v = [float(row[vel + a]) for a in AXES] if has_vel else [np.nan] * 3
        data.append([float(row["%time"]) * 1e-9] + [float(row[prefix + a]) for a in AXES] + v)
    return np.asarray(data, dtype=float)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help=".bag or .csv")
    parser.add_argument("--topic", default="/vins_estimator/odometry", help="odometry topic for bag input")
    parser.add_argument("--output", type=Path, help="PNG path (default: next to input)")
    parser.add_argument("--csv", type=Path, help="also export time,x,y,z,vx,vy,vz to this CSV")
    parser.add_argument("--absolute", action="store_true", help="plot absolute positions, not relative to start")
    args = parser.parse_args()

    src = args.input.expanduser().resolve()
    if not src.is_file():
        parser.error("input does not exist: {}".format(src))
    data = read_bag(src, args.topic) if src.suffix == ".bag" else read_csv(src)
    data = data[np.argsort(data[:, 0])]
    t = data[:, 0] - data[0, 0]
    pos = data[:, 1:4] if args.absolute else data[:, 1:4] - data[0, 1:4]
    vel = data[:, 4:7]

    output = args.output or src.with_name(src.stem + "_odometry.png")
    fig = plt.figure(figsize=(14, 9), constrained_layout=True)
    ax_pos, ax_vel = fig.add_subplot(2, 2, 1), fig.add_subplot(2, 2, 2)
    ax_xy, ax_3d = fig.add_subplot(2, 2, 3), fig.add_subplot(2, 2, 4, projection="3d")

    for i, (a, c) in enumerate(zip(AXES, COLORS)):
        ax_pos.plot(t, pos[:, i], color=c, linewidth=1.1, label=a)
        ax_vel.plot(t, vel[:, i], color=c, linewidth=1.0, label="v" + a)
    ax_pos.set(title="Position" + ("" if args.absolute else " relative to start"),
               xlabel="Elapsed time (s)", ylabel="m")
    ax_vel.set(title="Linear velocity", xlabel="Elapsed time (s)", ylabel="m/s")

    ax_xy.plot(pos[:, 0], pos[:, 1], color=COLORS[0], linewidth=1.2)
    ax_3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=COLORS[0], linewidth=1.1)
    for ax, xyz in ((ax_xy, (pos[:, 0], pos[:, 1])), (ax_3d, (pos[:, 0], pos[:, 1], pos[:, 2]))):
        ax.scatter(*[c[0] for c in xyz], color=COLORS[2], edgecolor="#30343B", s=50, label="Start", zorder=3)
        ax.scatter(*[c[-1] for c in xyz], color=COLORS[1], edgecolor="#30343B", s=50, label="End", zorder=3)
    ax_xy.set(title="X-Y trajectory", xlabel="X (m)", ylabel="Y (m)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_3d.set(title="3D trajectory", xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")

    for ax in (ax_pos, ax_vel, ax_xy):
        ax.grid(True, color="#D8DCE2", linewidth=0.7)
    for ax in (ax_pos, ax_vel, ax_xy, ax_3d):
        ax.legend(frameon=False)
    fig.suptitle("Odometry: {}".format(src.name), fontsize=14)
    fig.savefig(str(output), dpi=160, facecolor="white")

    if args.csv:
        with args.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("time_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps"))
            writer.writerows(data)
        print("csv: {}".format(args.csv))

    duration = t[-1]
    rate = (len(t) - 1) / duration if duration > 0 else float("nan")
    delta = data[-1, 1:4] - data[0, 1:4]
    print("saved: {}".format(output))
    print("samples: {}, duration: {:.3f} s, rate: {:.2f} Hz".format(len(t), duration, rate))
    for i, a in enumerate(AXES):
        print("{}: final delta {:+.4f} m, range {:.4f} m".format(a, delta[i], np.ptp(data[:, 1 + i])))


if __name__ == "__main__":
    main()
