#!/usr/bin/env python3
"""Convert a ROS 1 bag image topic directly into a feature-tracking MP4."""

import argparse
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


SUPPORTED_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read video frames from a ROS 1 bag and create a tracked MP4."
    )
    parser.add_argument("bag", help="Input ROS 1 .bag file")
    parser.add_argument("output", help="Output tracking .mp4 file")
    parser.add_argument(
        "--topic",
        help="Image topic (auto-selects color, then infra1, then the first image topic)",
    )
    parser.add_argument(
        "--right-topic",
        help="Optional second camera topic to synchronize and place beside --topic",
    )
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.01,
        help="Maximum stereo timestamp difference in seconds (default: 0.01)",
    )
    parser.add_argument("--fps", type=float, help="Override FPS calculated from timestamps")
    parser.add_argument("--max-features", type=int, default=300)
    parser.add_argument("--quality", type=float, default=0.01)
    parser.add_argument("--min-distance", type=float, default=12.0)
    parser.add_argument("--trail-length", type=int, default=20)
    parser.add_argument("--redetect-every", type=int, default=10)
    return parser.parse_args()


def choose_topic(bag, requested):
    topics = bag.get_type_and_topic_info().topics
    image_topics = [name for name, info in topics.items() if info.msg_type in SUPPORTED_TYPES]
    if requested:
        if requested not in topics:
            raise SystemExit(f"Topic not found in bag: {requested}")
        if topics[requested].msg_type not in SUPPORTED_TYPES:
            raise SystemExit(f"Topic is not an image topic: {requested}")
        return requested
    if not image_topics:
        raise SystemExit("The bag contains no sensor_msgs image topics")
    for preferred in ("/camera/color/image_raw", "/camera/infra1/image_rect_raw"):
        if preferred in image_topics:
            return preferred
    return sorted(image_topics)[0]


def message_to_frame(message, message_type, bridge):
    if message_type == "sensor_msgs/CompressedImage":
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    return bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")


def estimate_fps(stamps):
    differences = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not differences:
        return 30.0
    return 1.0 / statistics.median(differences)


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not bag_path.is_file():
        raise SystemExit(f"Bag does not exist: {bag_path}")
    if output_path.suffix.lower() != ".mp4":
        raise SystemExit("Output filename must end in .mp4")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    bridge = CvBridge()
    with rosbag.Bag(str(bag_path), "r") as bag:
        topic = choose_topic(bag, args.topic)
        topic_info = bag.get_type_and_topic_info().topics
        message_type = topic_info[topic].msg_type
        if args.right_topic:
            if args.right_topic not in topic_info:
                raise SystemExit(f"Topic not found in bag: {args.right_topic}")
            if topic_info[args.right_topic].msg_type not in SUPPORTED_TYPES:
                raise SystemExit(f"Topic is not an image topic: {args.right_topic}")
            right_message_type = topic_info[args.right_topic].msg_type
        else:
            right_message_type = None
        stamps = []
        for _, message, bag_time in bag.read_messages(topics=[topic]):
            stamp = (
                message.header.stamp.to_sec()
                if not message.header.stamp.is_zero()
                else bag_time.to_sec()
            )
            stamps.append(stamp)

    if not stamps:
        raise SystemExit(f"No images found on {topic}")
    fps = args.fps if args.fps else estimate_fps(stamps)

    with tempfile.TemporaryDirectory(prefix="rosbag_tracking_") as temp_directory:
        raw_video = Path(temp_directory) / "source.mp4"
        writer = None
        frame_count = 0
        width = height = 0

        def write_frame(frame):
            nonlocal writer, frame_count, width, height
            if writer is None:
                height, width = frame.shape[:2]
                writer = cv2.VideoWriter(
                    str(raw_video),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    fps,
                    (width, height),
                )
                if not writer.isOpened():
                    raise SystemExit("Could not create the temporary source video")
            if frame.shape[:2] != (height, width):
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
            frame_count += 1

        with rosbag.Bag(str(bag_path), "r") as bag:
            try:
                if not args.right_topic:
                    for _, message, _ in bag.read_messages(topics=[topic]):
                        frame = message_to_frame(message, message_type, bridge)
                        if frame is not None:
                            write_frame(frame)
                else:
                    pending = {topic: None, args.right_topic: None}
                    types = {topic: message_type, args.right_topic: right_message_type}
                    for name, message, bag_time in bag.read_messages(
                        topics=[topic, args.right_topic]
                    ):
                        frame = message_to_frame(message, types[name], bridge)
                        if frame is None:
                            continue
                        stamp = (
                            message.header.stamp.to_sec()
                            if not message.header.stamp.is_zero()
                            else bag_time.to_sec()
                        )
                        pending[name] = (stamp, frame)
                        left = pending[topic]
                        right = pending[args.right_topic]
                        if left is None or right is None:
                            continue
                        difference = left[0] - right[0]
                        if abs(difference) <= args.sync_tolerance:
                            left_frame, right_frame = left[1], right[1]
                            if right_frame.shape[0] != left_frame.shape[0]:
                                right_width = round(
                                    right_frame.shape[1]
                                    * left_frame.shape[0]
                                    / right_frame.shape[0]
                                )
                                right_frame = cv2.resize(
                                    right_frame, (right_width, left_frame.shape[0])
                                )
                            write_frame(cv2.hconcat([left_frame, right_frame]))
                            pending[topic] = None
                            pending[args.right_topic] = None
                        elif difference < 0:
                            pending[topic] = None
                        else:
                            pending[args.right_topic] = None
            finally:
                if writer is not None:
                    writer.release()
        if frame_count == 0:
            raise SystemExit(f"No readable images found on {topic}")

        tracker = Path(__file__).with_name("track_features_video.py")
        command = [
            sys.executable,
            str(tracker),
            str(raw_video),
            str(output_path),
            "--max-features",
            str(args.max_features),
            "--quality",
            str(args.quality),
            "--min-distance",
            str(args.min_distance),
            "--trail-length",
            str(args.trail_length),
            "--redetect-every",
            str(args.redetect_every),
        ]
        subprocess.run(command, check=True)

    if args.right_topic:
        print(f"Topics: {topic} + {args.right_topic}")
    else:
        print(f"Topic: {topic}")
    print(f"Frames: {frame_count}, FPS: {fps:.2f}, resolution: {width}x{height}")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
