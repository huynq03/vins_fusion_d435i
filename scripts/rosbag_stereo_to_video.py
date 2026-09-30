#!/usr/bin/env python3
"""Create a side-by-side MP4 from two ROS 1 bag image topics."""

import argparse
import statistics
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge


def message_stamp(message, bag_time):
    stamp = message.header.stamp
    return stamp.to_sec() if not stamp.is_zero() else bag_time.to_sec()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Input ROS 1 bag")
    parser.add_argument("output", type=Path, help="Output MP4")
    parser.add_argument(
        "--left-topic",
        default="/camera/infra1/image_rect_raw",
        help="Left image topic (default: %(default)s)",
    )
    parser.add_argument(
        "--right-topic",
        default="/camera/infra2/image_rect_raw",
        help="Right image topic (default: %(default)s)",
    )
    parser.add_argument("--fps", type=float, help="Override output frame rate")
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.01,
        help="Maximum left/right timestamp difference in seconds (default: %(default)s)",
    )
    args = parser.parse_args()

    bag_path = args.bag.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not bag_path.is_file():
        parser.error(f"bag does not exist: {bag_path}")
    if output_path.suffix.lower() != ".mp4":
        parser.error("output must have an .mp4 extension")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    topics = (args.left_topic, args.right_topic)
    stamps = []
    with rosbag.Bag(str(bag_path)) as bag:
        bag_topics = bag.get_type_and_topic_info().topics
        for topic in topics:
            if topic not in bag_topics:
                parser.error(f"topic not found: {topic}")
            if bag_topics[topic].msg_type != "sensor_msgs/Image":
                parser.error(f"topic is not sensor_msgs/Image: {topic}")
        for _, message, bag_time in bag.read_messages(topics=[args.left_topic]):
            stamps.append(message_stamp(message, bag_time))

    differences = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    fps = args.fps or (1.0 / statistics.median(differences) if differences else 30.0)

    bridge = CvBridge()
    pending = {args.left_topic: None, args.right_topic: None}
    writer = None
    frame_count = 0
    output_size = None

    try:
        with rosbag.Bag(str(bag_path)) as bag:
            for topic, message, bag_time in bag.read_messages(topics=list(topics)):
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                pending[topic] = (message_stamp(message, bag_time), frame)
                left = pending[args.left_topic]
                right = pending[args.right_topic]
                if left is None or right is None:
                    continue

                delta = left[0] - right[0]
                if abs(delta) <= args.sync_tolerance:
                    left_frame, right_frame = left[1], right[1]
                    if right_frame.shape[:2] != left_frame.shape[:2]:
                        right_frame = cv2.resize(
                            right_frame, (left_frame.shape[1], left_frame.shape[0])
                        )
                    combined = cv2.hconcat([left_frame, right_frame])
                    if writer is None:
                        output_size = (combined.shape[1], combined.shape[0])
                        writer = cv2.VideoWriter(
                            str(output_path),
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            fps,
                            output_size,
                        )
                        if not writer.isOpened():
                            raise RuntimeError(f"cannot create video: {output_path}")
                    writer.write(combined)
                    frame_count += 1
                    pending[args.left_topic] = None
                    pending[args.right_topic] = None
                elif delta < 0:
                    pending[args.left_topic] = None
                else:
                    pending[args.right_topic] = None
    finally:
        if writer is not None:
            writer.release()

    if frame_count == 0:
        raise RuntimeError("no synchronized stereo frames were found")
    print(f"topics: {args.left_topic} + {args.right_topic}")
    print(f"frames: {frame_count}, fps: {fps:.2f}, resolution: {output_size[0]}x{output_size[1]}")
    print(f"saved: {output_path}")


if __name__ == "__main__":
    main()
