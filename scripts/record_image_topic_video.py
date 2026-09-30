#!/usr/bin/env python3
import argparse
import signal

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description="Record a ROS image topic to MP4")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("image_topic_video_recorder", anonymous=True)
    bridge = CvBridge()
    writer = None
    frame_count = 0

    def close_writer():
        nonlocal writer
        if writer is not None:
            writer.release()
            writer = None
        rospy.loginfo("Saved %d frames to %s", frame_count, args.output)

    def callback(message):
        nonlocal writer, frame_count
        frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if writer is None:
            height, width = frame.shape[:2]
            writer = cv2.VideoWriter(
                args.output,
                cv2.VideoWriter_fourcc(*"mp4v"),
                args.fps,
                (width, height),
            )
            if not writer.isOpened():
                raise RuntimeError("Could not open video writer: " + args.output)
        writer.write(frame)
        frame_count += 1

    rospy.on_shutdown(close_writer)
    signal.signal(signal.SIGTERM, lambda *_: rospy.signal_shutdown("SIGTERM"))
    rospy.Subscriber(args.topic, Image, callback, queue_size=100)
    rospy.spin()


if __name__ == "__main__":
    main()
