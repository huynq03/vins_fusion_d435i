#!/usr/bin/env python3
"""Render VINS tracking images with timestamp-aligned odometry and export CSV."""

import argparse
import bisect
import csv
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

from rosbag_to_tracking_video import estimate_fps


def stamp(message, bag_time):
    return (message.header.stamp if not message.header.stamp.is_zero() else bag_time).to_sec()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--image-topic", default="/vins_estimator/image_track")
    parser.add_argument("--odom-topic", default="/vins_estimator/odometry")
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    if args.output.suffix.lower() != ".mp4" or args.output.exists():
        parser.error("Output must be a new .mp4 file")
    if args.csv and args.csv.exists():
        parser.error("CSV output already exists")

    samples, image_stamps = [], []
    with rosbag.Bag(str(args.bag)) as bag:
        topics = bag.get_type_and_topic_info().topics
        for topic, kind in ((args.image_topic, "sensor_msgs/Image"), (args.odom_topic, "nav_msgs/Odometry")):
            if topic not in topics or topics[topic].msg_type != kind:
                parser.error(f"Required {kind} topic missing: {topic}")
        for topic, msg, bag_time in bag.read_messages(topics=[args.image_topic, args.odom_topic]):
            t = stamp(msg, bag_time)
            if topic == args.image_topic:
                image_stamps.append(t)
            else:
                p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
                samples.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z))
    if not samples or not image_stamps:
        parser.error("Both tracking images and odometry samples are required")
    samples.sort(key=lambda row: row[0])
    if not np.isfinite(np.asarray(samples)).all():
        raise RuntimeError("Odometry contains non-finite values")
    odom_stamps = [row[0] for row in samples]
    fps = estimate_fps(image_stamps)
    writer, size = None, None
    count = 0
    bridge = CvBridge()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with rosbag.Bag(str(args.bag)) as bag:
            for _, msg, bag_time in bag.read_messages(topics=[args.image_topic]):
                t = stamp(msg, bag_time)
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                h, w = frame.shape[:2]
                canvas = np.zeros((h + 100, w, 3), dtype=np.uint8)
                canvas[:h] = frame
                index = bisect.bisect_right(odom_stamps, t) - 1
                lines = [f"VINS-Fusion stereo + IMU | t = {t - image_stamps[0]:.2f} s"]
                if index < 0:
                    lines.append("Odometry: waiting for initialization")
                else:
                    row = samples[index]
                    age = t - row[0]
                    lines.append(f"Position [m]: x {row[1]:+.3f}   y {row[2]:+.3f}   z {row[3]:+.3f}")
                    lines.append(f"Speed: {np.linalg.norm(row[8:11]):.3f} m/s | odometry age: {age:.3f} s"
                                 + (" (STALE)" if age > 0.5 else ""))
                for i, line in enumerate(lines):
                    cv2.putText(canvas, line, (12, h + 25 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (240, 240, 240), 1, cv2.LINE_AA)
                current_size = (w, h + 100)
                if writer is None:
                    size = current_size
                    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
                    if not writer.isOpened():
                        raise RuntimeError("Cannot open output video")
                if size != current_size:
                    raise RuntimeError("Tracking image resolution changed")
                writer.write(canvas)
                count += 1
    finally:
        if writer is not None:
            writer.release()
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as stream:
            output = csv.writer(stream)
            output.writerow(("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "vx_mps", "vy_mps", "vz_mps"))
            output.writerows(samples)
    print(f"Video: {args.output}; {count} frames, {fps:.3f} FPS, {size}")
    print(f"Odometry: {len(samples)} samples, {odom_stamps[-1] - odom_stamps[0]:.3f} seconds")


if __name__ == "__main__":
    main()
