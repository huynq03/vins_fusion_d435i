#!/usr/bin/env python3
"""Record VINS and PX4 odometry (nav_msgs/Odometry) to CSV, one file per topic.

Default topics -> files in output/odom_logs/<YYYYmmdd_HHMMSS>/:
  vins_odom.csv      /vins_estimator/odometry      raw VINS: body RDF, velocity in world frame
  vins_flu_odom.csv  /mavros/odometry/out          VINS as sent to PX4: body FLU, velocity in body frame
  fc_odom.csv        /mavros/local_position/odom   PX4 EKF2 output: ENU world, body FLU, velocity in body frame

Compare fc_odom.csv with vins_flu_odom.csv (same conventions); vins_odom.csv is the
untouched estimator output. time_s is the message stamp, recv_time_s the arrival time.
Run inside the container (or use scripts/record_odom.sh from the host). Ctrl+C stops.
"""

import argparse
import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry

WS = Path(__file__).resolve().parent.parent
TOPICS = (
    ("vins_odom", "/vins_estimator/odometry"),
    ("vins_flu_odom", "/mavros/odometry/out"),
    ("fc_odom", "/mavros/local_position/odom"),
)
HEADER = ("time_s", "recv_time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
          "roll_deg", "pitch_deg", "yaw_deg",
          "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps")


def euler_deg(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


class TopicLog:
    def __init__(self, name, topic, directory):
        self.name, self.topic, self.path = name, topic, directory / (name + ".csv")
        self.count = 0
        self._lock = threading.Lock()
        self._stream = self.path.open("w", newline="")
        self._writer = csv.writer(self._stream)
        self._writer.writerow(HEADER)
        self._sub = rospy.Subscriber(topic, Odometry, self._on_msg, queue_size=200, tcp_nodelay=True)

    def _on_msg(self, msg):
        recv = time.time()
        stamp = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() else recv
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        row = ["{:.6f}".format(stamp), "{:.6f}".format(recv)]
        row += ["{:.6f}".format(a) for a in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)]
        row += ["{:.3f}".format(a) for a in euler_deg(q)]
        row += ["{:.6f}".format(a) for a in (v.x, v.y, v.z, w.x, w.y, w.z)]
        with self._lock:
            if not self._stream.closed:
                self._writer.writerow(row)
                self.count += 1

    def flush(self):
        with self._lock:
            if not self._stream.closed:
                self._stream.flush()

    def close(self):
        self._sub.unregister()
        with self._lock:
            self._stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, help="output directory (default: output/odom_logs/<time>)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (default: until Ctrl+C)")
    parser.add_argument("--topic", action="append", default=[], metavar="NAME=TOPIC",
                        help="extra Odometry topic -> NAME.csv, e.g. fc_mavlink_odom=/mavros/odometry/in")
    args = parser.parse_args(rospy.myargv()[1:])

    topics = list(TOPICS)
    for item in args.topic:
        name, sep, topic = item.partition("=")
        if not sep or not name or not topic.startswith("/"):
            parser.error("--topic expects NAME=/topic, got: {}".format(item))
        topics.append((name, topic))

    directory = args.output or WS / "output" / "odom_logs" / datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = directory.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)

    rospy.init_node("record_odom_csv", anonymous=True)
    logs = [TopicLog(name, topic, directory) for name, topic in topics]
    print("Recording to {} (Ctrl+C to stop)".format(directory))

    start, last, ticks = time.time(), [0] * len(logs), 0
    try:
        while not rospy.is_shutdown() and (args.duration <= 0 or time.time() - start < args.duration):
            time.sleep(1.0)
            ticks += 1
            for log in logs:
                log.flush()
            if ticks % 5 == 0:
                counts = [log.count for log in logs]
                print(" | ".join("{}: {} ({:.1f} Hz)".format(log.name, c, (c - p) / 5.0)
                                 for log, c, p in zip(logs, counts, last)))
                last = counts
    except KeyboardInterrupt:
        pass
    finally:
        for log in logs:
            log.close()

    print("Saved:")
    for log in logs:
        note = "" if log.count else "   <- no messages on {}".format(log.topic)
        print("  {}  {} rows{}".format(log.path, log.count, note))


if __name__ == "__main__":
    main()
