#!/usr/bin/env python3
"""Plot an Oxy-plane odometry trajectory from a ROS 1 bag."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def read_odometry(bag_path: Path, topic: str):
    timestamps = []
    positions = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[topic]):
            position = message.pose.pose.position
            stamp = message.header.stamp
            timestamp = stamp.to_sec() if not stamp.is_zero() else bag_time.to_sec()
            timestamps.append(timestamp)
            positions.append((position.x, position.y))

    if not timestamps:
        raise ValueError(f"No odometry messages found on topic {topic!r}")

    time_s = np.asarray(timestamps, dtype=float)
    xy = np.asarray(positions, dtype=float)
    return time_s - time_s[0], xy


def main():
    parser = argparse.ArgumentParser(
        description="Plot an odometry trajectory in the Oxy plane from a ROS 1 bag."
    )
    parser.add_argument("bag", type=Path, help="Input ROS 1 .bag file")
    parser.add_argument(
        "--topic",
        default="/vins_estimator/odometry",
        help="nav_msgs/Odometry topic (default: %(default)s)",
    )
    parser.add_argument("--output", type=Path, help="Output PNG path")
    parser.add_argument(
        "--absolute",
        action="store_true",
        help="Plot absolute positions instead of displacement from the first sample",
    )
    args = parser.parse_args()

    bag_path = args.bag.expanduser().resolve()
    if not bag_path.is_file():
        parser.error(f"bag file does not exist: {bag_path}")

    output = (
        args.output.expanduser().resolve()
        if args.output
        else bag_path.with_name(f"{bag_path.stem}_odometry_oxy.png")
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    time_s, xy = read_odometry(bag_path, args.topic)
    plotted_xy = xy if args.absolute else xy - xy[0]

    fig, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
    axis.plot(
        plotted_xy[:, 0], plotted_xy[:, 1],
        color="#2463A3", linewidth=1.4, label="Trajectory",
    )
    axis.scatter(
        plotted_xy[0, 0], plotted_xy[0, 1],
        color="#2E7D32", edgecolor="white", s=70, label="Start", zorder=3,
    )
    axis.scatter(
        plotted_xy[-1, 0], plotted_xy[-1, 1],
        color="#D17A22", edgecolor="white", s=70, label="End", zorder=3,
    )
    axis.axhline(0.0, color="#30343B", linewidth=0.8, alpha=0.6)
    axis.axvline(0.0, color="#30343B", linewidth=0.8, alpha=0.6)
    axis.set_title(f"VINS odometry trajectory in Oxy — {bag_path.name}")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, color="#D8DCE2", linewidth=0.7, alpha=0.8)
    axis.legend(frameon=False)
    fig.savefig(output, dpi=160, facecolor="white")
    plt.close(fig)

    duration = time_s[-1]
    rate = (len(time_s) - 1) / duration if duration > 0 else float("nan")
    delta = xy[-1] - xy[0]
    print(f"saved: {output}")
    print(f"topic: {args.topic}")
    print(f"samples: {len(time_s)}, duration: {duration:.3f} s, average rate: {rate:.2f} Hz")
    print(f"final displacement: x={delta[0]:+.6f} m, y={delta[1]:+.6f} m")


if __name__ == "__main__":
    main()
