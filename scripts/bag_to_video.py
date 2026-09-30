#!/usr/bin/env python3
"""Export camera data to MP4. Run inside the container (needs rosbag / rospy).

  raw   BAG OUT [--topic T] [--right-topic T2]   image topic (or synced stereo pair) to MP4
  track INPUT OUT [same as raw]                   draw Lucas-Kanade feature tracks; INPUT = bag or video file
  vins  BAG OUT [--csv CSV]                       /vins_estimator/image_track with odometry overlay
  live  --topic T OUT                             record a live ROS image topic until Ctrl+C
"""

import argparse
import bisect
import csv
from pathlib import Path

import cv2
import numpy as np

IMAGE_TYPES = ("sensor_msgs/Image", "sensor_msgs/CompressedImage")


def stamp(msg, bag_time):
    return (msg.header.stamp if not msg.header.stamp.is_zero() else bag_time).to_sec()


def to_frame(msg, bridge):
    if msg._type == "sensor_msgs/CompressedImage":
        return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


class VideoOut:
    """MP4 writer opened on the first frame; later frames are resized to match."""

    def __init__(self, path, fps):
        self.path, self.fps, self.writer, self.size, self.count = Path(path), fps, None, None, 0
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, frame):
        size = (frame.shape[1], frame.shape[0])
        if self.writer is None:
            self.size = size
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, size)
            if not self.writer.isOpened():
                raise SystemExit("cannot create video: {}".format(self.path))
        if size != self.size:
            frame = cv2.resize(frame, self.size)
        self.writer.write(frame)
        self.count += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
        if self.count == 0:
            raise SystemExit("no frames written")
        print("saved: {}  ({} frames, {:.2f} fps, {}x{})".format(self.path, self.count, self.fps, *self.size))


def bag_topic_fps(bag, topic, override):
    info = bag.get_type_and_topic_info().topics
    if topic not in info or info[topic].msg_type not in IMAGE_TYPES:
        raise SystemExit("image topic not found in bag: {}".format(topic))
    return override or info[topic].frequency or 30.0


def bag_frames(bag, topic, right_topic=None, tolerance=0.01):
    """Yield frames of `topic`, or left|right side by side when `right_topic` is given."""
    from cv_bridge import CvBridge

    bridge = CvBridge()
    if not right_topic:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            yield to_frame(msg, bridge)
        return
    pending = {topic: None, right_topic: None}
    for name, msg, bag_time in bag.read_messages(topics=[topic, right_topic]):
        pending[name] = (stamp(msg, bag_time), to_frame(msg, bridge))
        left, right = pending[topic], pending[right_topic]
        if left is None or right is None:
            continue
        delta = left[0] - right[0]
        if abs(delta) <= tolerance:
            r = right[1]
            if r.shape[0] != left[1].shape[0]:
                r = cv2.resize(r, (round(r.shape[1] * left[1].shape[0] / r.shape[0]), left[1].shape[0]))
            yield cv2.hconcat([left[1], r])
            pending = {topic: None, right_topic: None}
        else:  # drop the older frame
            pending[topic if delta < 0 else right_topic] = None


class FeatureTracker:
    """Shi-Tomasi corners tracked with forward-backward checked pyramidal LK."""

    LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    def __init__(self, args):
        self.args, self.tracks, self.prev, self.n = args, [], None, 0

    def detect(self, gray):
        remaining = self.args.max_features - len(self.tracks)
        if remaining <= 0:
            return
        mask = np.full(gray.shape, 255, dtype=np.uint8)
        for track in self.tracks:
            cv2.circle(mask, tuple(int(round(v)) for v in track[-1]), int(self.args.min_distance), 0, -1)
        found = cv2.goodFeaturesToTrack(gray, remaining, self.args.quality, self.args.min_distance,
                                        mask=mask, blockSize=7)
        if found is not None:
            self.tracks += [p.reshape(1, 2) for p in found]

    def step(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        if self.tracks and self.prev is not None:
            p0 = np.float32([t[-1] for t in self.tracks])
            p1, st1, _ = cv2.calcOpticalFlowPyrLK(self.prev, gray, p0, None, **self.LK)
            back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev, p1, None, **self.LK)
            ok = st1.ravel().astype(bool) & st2.ravel().astype(bool) & (np.abs(p0 - back).max(axis=1) < 1.0)
            self.tracks = [np.vstack((t, p))[-self.args.trail_length:]
                           for t, p, k in zip(self.tracks, p1, ok) if k and 0 <= p[0] < w and 0 <= p[1] < h]
        if self.n % self.args.redetect_every == 0 or len(self.tracks) < self.args.max_features // 2:
            self.detect(gray)
        self.prev, self.n = gray, self.n + 1

        out = frame.copy()
        for t in self.tracks:
            path = np.round(t).astype(np.int32)
            if self.args.draw_trails and len(path) > 1:
                cv2.polylines(out, [path], False, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.circle(out, tuple(path[-1]), 3, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(out, "Tracked features: {}".format(len(self.tracks)), (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        return out


def cmd_raw_or_track(args):
    tracker = FeatureTracker(args) if args.cmd == "track" else None
    process = tracker.step if tracker else (lambda f: f)
    if args.input.suffix != ".bag":  # plain video file (track only)
        cap = cv2.VideoCapture(str(args.input))
        if not cap.isOpened():
            raise SystemExit("cannot open video: {}".format(args.input))
        out = VideoOut(args.output, args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0)
        ok, frame = cap.read()
        while ok:
            out.write(process(frame))
            ok, frame = cap.read()
        cap.release()
        return out.close()

    import rosbag

    with rosbag.Bag(str(args.input)) as bag:
        out = VideoOut(args.output, bag_topic_fps(bag, args.topic, args.fps))
        if args.right_topic:
            bag_topic_fps(bag, args.right_topic, None)
        for frame in bag_frames(bag, args.topic, args.right_topic, args.sync_tolerance):
            out.write(process(frame))
    out.close()


def cmd_vins(args):
    import rosbag
    from cv_bridge import CvBridge

    odom = []
    with rosbag.Bag(str(args.input)) as bag:
        fps = bag_topic_fps(bag, args.topic, args.fps)
        for _, msg, bag_time in bag.read_messages(topics=[args.odom_topic]):
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            odom.append((stamp(msg, bag_time), p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z))
        odom.sort()
        odom_t = [row[0] for row in odom]

        bridge, out, t0 = CvBridge(), VideoOut(args.output, fps), None
        for _, msg, bag_time in bag.read_messages(topics=[args.topic]):
            t = stamp(msg, bag_time)
            t0 = t if t0 is None else t0
            frame = to_frame(msg, bridge)
            h, w = frame.shape[:2]
            canvas = np.zeros((h + 100, w, 3), dtype=np.uint8)
            canvas[:h] = frame
            lines = ["VINS-Fusion stereo + IMU | t = {:.2f} s".format(t - t0)]
            i = bisect.bisect_right(odom_t, t) - 1
            if i < 0:
                lines.append("Odometry: waiting for initialization")
            else:
                r, age = odom[i], t - odom[i][0]
                lines.append("Position [m]: x {:+.3f}   y {:+.3f}   z {:+.3f}".format(*r[1:4]))
                lines.append("Speed: {:.3f} m/s | odometry age: {:.3f} s{}".format(
                    np.linalg.norm(r[8:11]), age, " (STALE)" if age > 0.5 else ""))
            for k, line in enumerate(lines):
                cv2.putText(canvas, line, (12, h + 25 + k * 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (240, 240, 240), 1, cv2.LINE_AA)
            out.write(canvas)
    out.close()

    if args.csv:
        with args.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "vx_mps", "vy_mps", "vz_mps"))
            writer.writerows(odom)
        print("csv: {} ({} samples)".format(args.csv, len(odom)))


def cmd_live(args):
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    rospy.init_node("image_topic_video_recorder", anonymous=True)
    bridge, out = CvBridge(), VideoOut(args.output, args.fps or 30.0)
    rospy.Subscriber(args.topic, Image, lambda msg: out.write(to_frame(msg, bridge)), queue_size=100)
    rospy.on_shutdown(out.close)
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--fps", type=float, help="override frame rate")

    for name in ("raw", "track"):
        p = sub.add_parser(name, parents=[common])
        p.add_argument("input", type=Path)
        p.add_argument("output", type=Path)
        p.add_argument("--topic", default="/camera/color/image_raw")
        p.add_argument("--right-topic", help="second topic, synced and placed on the right")
        p.add_argument("--sync-tolerance", type=float, default=0.01, help="max stereo stamp difference (s)")
        p.set_defaults(func=cmd_raw_or_track)
        if name == "track":
            p.add_argument("--max-features", type=int, default=300)
            p.add_argument("--quality", type=float, default=0.01)
            p.add_argument("--min-distance", type=float, default=12.0)
            p.add_argument("--trail-length", type=int, default=20)
            p.add_argument("--redetect-every", type=int, default=10)
            p.add_argument("--draw-trails", action="store_true")

    p = sub.add_parser("vins", parents=[common])
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--topic", default="/vins_estimator/image_track")
    p.add_argument("--odom-topic", default="/vins_estimator/odometry")
    p.add_argument("--csv", type=Path, help="also export odometry to CSV")
    p.set_defaults(func=cmd_vins)

    p = sub.add_parser("live", parents=[common])
    p.add_argument("output", type=Path)
    p.add_argument("--topic", required=True)
    p.set_defaults(func=cmd_live)

    args = parser.parse_args()
    if getattr(args, "input", None) is not None and not args.input.is_file():
        parser.error("input does not exist: {}".format(args.input))
    args.func(args)


if __name__ == "__main__":
    main()
