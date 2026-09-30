#!/usr/bin/env python3
"""Export a ROS 1 bag image topic to MP4 without replaying it into ROS."""

import argparse
import math
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge

from rosbag_to_tracking_video import choose_topic, estimate_fps, message_to_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--fps", type=float, help="Override timestamp-derived FPS")
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f"Bag not found: {args.bag}")
    if args.output.suffix.lower() != ".mp4":
        parser.error("Output must end in .mp4")
    if args.output.exists():
        parser.error(f"Output already exists: {args.output}")
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0):
        parser.error("FPS must be finite and positive")

    bridge = CvBridge()
    with rosbag.Bag(str(args.bag)) as bag:
        topic = choose_topic(bag, args.topic)
        info = bag.get_type_and_topic_info().topics[topic]
        stamps = []
        if args.fps is None:
            for _, message, bag_time in bag.read_messages(topics=[topic]):
                stamp = message.header.stamp
                stamps.append(stamp.to_sec() if not stamp.is_zero() else bag_time.to_sec())
        fps = args.fps if args.fps is not None else estimate_fps(stamps)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = None
        count = 0
        size = None
        try:
            for _, message, _ in bag.read_messages(topics=[topic]):
                frame = message_to_frame(message, info.msg_type, bridge)
                if frame is None:
                    raise RuntimeError("Could not decode image")
                frame_size = (frame.shape[1], frame.shape[0])
                if writer is None:
                    size = frame_size
                    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
                    if not writer.isOpened():
                        raise RuntimeError(f"Cannot create video: {args.output}")
                if frame_size != size:
                    raise RuntimeError("Image resolution changed within bag")
                writer.write(frame)
                count += 1
        finally:
            if writer is not None:
                writer.release()
        if count == 0:
            raise RuntimeError("No image frames found")
        print(f"topic: {topic}")
        print(f"frames: {count}, fps: {fps:.3f}, resolution: {size[0]}x{size[1]}")
        print(f"duration: {count / fps:.2f}s")
        print(f"saved: {args.output.resolve()}")


if __name__ == "__main__":
    main()
