#!/usr/bin/env python3
"""Extract ROS1 nav_msgs/Odometry from a bag and render its trajectory."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Registers the "3d" projection.
import numpy as np
import rosbag


ODOMETRY_TYPE = "nav_msgs/Odometry"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render position, velocity, and trajectory from a ROS1 odometry bag."
    )
    parser.add_argument("bag", help="Input ROS1 .bag file")
    parser.add_argument("output", help="Output .png file")
    parser.add_argument(
        "--topic",
        help="Odometry topic (auto-selects /vins_estimator/odometry when present)",
    )
    parser.add_argument(
        "--csv",
        help="Optional CSV output containing timestamp, position, and velocity",
    )
    return parser.parse_args()


def choose_topic(bag, requested):
    topics = bag.get_type_and_topic_info().topics
    odometry_topics = [name for name, info in topics.items() if info.msg_type == ODOMETRY_TYPE]
    if requested:
        if requested not in topics:
            raise SystemExit("Topic not found in bag: {}".format(requested))
        if topics[requested].msg_type != ODOMETRY_TYPE:
            raise SystemExit("Topic is not nav_msgs/Odometry: {}".format(requested))
        return requested
    if not odometry_topics:
        raise SystemExit("The bag contains no nav_msgs/Odometry topic")
    return "/vins_estimator/odometry" if "/vins_estimator/odometry" in odometry_topics else sorted(odometry_topics)[0]


def read_odometry(bag_path, topic):
    samples = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[topic]):
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = bag_time.to_sec()
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            samples.append((stamp, position.x, position.y, position.z, velocity.x, velocity.y, velocity.z))
    if not samples:
        raise SystemExit("No odometry messages found on {}".format(topic))
    return np.asarray(samples, dtype=float)


def write_csv(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps"))
        writer.writerows(samples)


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not bag_path.is_file():
        raise SystemExit("Bag does not exist: {}".format(bag_path))
    if output_path.suffix.lower() != ".png":
        raise SystemExit("Output filename must end in .png")

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic = choose_topic(bag, args.topic)
    samples = read_odometry(bag_path, topic)
    samples[:, 0] -= samples[0, 0]
    positions = samples[:, 1:4]
    velocities = samples[:, 4:7]
    relative = positions - positions[0]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(14, 9), constrained_layout=True)
    position_axis = fig.add_subplot(2, 2, 1)
    velocity_axis = fig.add_subplot(2, 2, 2)
    xy_axis = fig.add_subplot(2, 2, 3)
    trajectory_axis = fig.add_subplot(2, 2, 4, projection="3d")
    colors = ("#2463A3", "#D17A22", "#66752D")

    for index, label in enumerate(("x", "y", "z")):
        position_axis.plot(samples[:, 0], relative[:, index], label=label, color=colors[index], linewidth=1.1)
        velocity_axis.plot(samples[:, 0], velocities[:, index], label="v{}".format(label), color=colors[index], linewidth=1.0)
    position_axis.set(title="Position relative to first sample", xlabel="Elapsed time (s)", ylabel="Position (m)")
    velocity_axis.set(title="Linear velocity", xlabel="Elapsed time (s)", ylabel="Velocity (m/s)")

    xy_axis.plot(relative[:, 0], relative[:, 1], color=colors[0], linewidth=1.2)
    xy_axis.scatter(relative[0, 0], relative[0, 1], color=colors[2], edgecolor="#30343B", s=55, label="Start", zorder=3)
    xy_axis.scatter(relative[-1, 0], relative[-1, 1], color=colors[1], edgecolor="#30343B", s=55, label="End", zorder=3)
    xy_axis.set(title="X-Y trajectory", xlabel="X (m)", ylabel="Y (m)")
    xy_axis.set_aspect("equal", adjustable="box")

    trajectory_axis.plot(relative[:, 0], relative[:, 1], relative[:, 2], color=colors[0], linewidth=1.1)
    trajectory_axis.scatter(relative[0, 0], relative[0, 1], relative[0, 2], color=colors[2], s=35, label="Start")
    trajectory_axis.scatter(relative[-1, 0], relative[-1, 1], relative[-1, 2], color=colors[1], s=35, label="End")
    trajectory_axis.set(title="3D trajectory", xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")

    for axis in (position_axis, velocity_axis, xy_axis):
        axis.grid(True, color="#D8DCE2", linewidth=0.7)
        axis.legend(frameon=False)
    trajectory_axis.legend(frameon=False)
    fig.suptitle("Odometry: {}".format(topic), fontsize=14)
    fig.savefig(str(output_path), dpi=160, facecolor="white")
    plt.close(fig)

    if args.csv:
        csv_path = Path(args.csv).expanduser().resolve()
        write_csv(csv_path, samples)
        print("CSV: {}".format(csv_path))

    duration = samples[-1, 0]
    rate = (len(samples) - 1) / duration if duration > 0 else float("nan")
    print("Topic: {}".format(topic))
    print("Samples: {}, duration: {:.3f} s, average rate: {:.2f} Hz".format(len(samples), duration, rate))
    print("Output: {}".format(output_path))


if __name__ == "__main__":
    main()
