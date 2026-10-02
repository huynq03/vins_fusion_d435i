#!/usr/bin/env python3
"""Record VINS and PX4 odometry (nav_msgs/Odometry) and the raw GPS of the FC to CSV, one file per topic.

Default topics -> files in output/odom_logs/<YYYYmmdd_HHMMSS>/, each named NAME_<YYYYmmdd_HHMMSS>.csv:
  vins_odom      /vins_estimator/odometry      raw VINS: body RDF, velocity in world frame
  vins_flu_odom  /mavros/odometry/out          VINS as sent to PX4: body FLU, velocity in body frame
  fc_odom        /mavros/local_position/odom   PX4 EKF2 output: ENU world, body FLU, velocity in body frame
  gps_raw        /mavros/gpsstatus/gps1/raw    MAVLink GPS_RAW_INT: the GPS / UWB receiver output before EKF2

gps_raw has its own columns: lat/lon/alt as reported plus x_m (east), y_m (north), z_m (up)
relative to the first sample with a fix, using the spherical earth of PX4 (R = 6371000 m).
lat/lon come as 1e-7 deg, so x/y are quantized to about 1 cm.

Unless --no-fc-log is given, the FC is told to log to its SD card for the same period
(NSH `logger on` / `logger off` through MAVROS; PX4 otherwise logs only while armed). The ULog
path on the FC is printed and written to fc_ulog_<time>.txt; it holds the EKF2 internals
(fusion flags, innovations) that the odometry topics do not show.

Compare fc_odom with vins_flu_odom (same conventions); vins_odom is the untouched
estimator output. The date/time is the local start time (record_odom.sh passes the host
time zone; a container created before compose.yaml mounted /etc/localtime is UTC). time_s is the message stamp, recv_time_s the arrival time.
Run inside the container (or use scripts/record_odom.sh from the host). Ctrl+C stops.
"""

import argparse
import csv
import math
import re
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import rospy
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry

from px4_shell import Px4Shell

WS = Path(__file__).resolve().parent.parent
TOPICS = (
    ("vins_odom", "/vins_estimator/odometry"),
    ("vins_flu_odom", "/mavros/odometry/out"),
    ("fc_odom", "/mavros/local_position/odom"),
)
HEADER = ("time_s", "recv_time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
          "roll_deg", "pitch_deg", "yaw_deg",
          "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps")
GPS_NAME, GPS_TOPIC = "gps_raw", "/mavros/gpsstatus/gps1/raw"
GPS_HEADER = ("time_s", "recv_time_s", "x_m", "y_m", "z_m", "lat_deg", "lon_deg", "alt_m",
              "fix_type", "satellites", "eph", "epv", "vel_mps", "cog_deg")
EARTH_RADIUS_M = 6371000.0  # CONSTANTS_RADIUS_OF_EARTH in PX4


def euler_deg(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


class TopicLog:
    msg_type, header = Odometry, HEADER

    def __init__(self, name, topic, directory, stamp):
        self.name, self.topic, self.path = name, topic, directory / "{}_{}.csv".format(name, stamp)
        self.count = 0
        self._lock = threading.Lock()
        self._stream = self.path.open("w", newline="")
        self._writer = csv.writer(self._stream)
        self._writer.writerow(self.header)
        self._sub = rospy.Subscriber(topic, self.msg_type, self._on_msg, queue_size=200, tcp_nodelay=True)

    def _values(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        row = ["{:.6f}".format(a) for a in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)]
        row += ["{:.3f}".format(a) for a in euler_deg(q)]
        return row + ["{:.6f}".format(a) for a in (v.x, v.y, v.z, w.x, w.y, w.z)]

    def _on_msg(self, msg):
        recv = time.time()
        stamp = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() else recv
        row = ["{:.6f}".format(stamp), "{:.6f}".format(recv)] + self._values(msg)
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


class GpsLog(TopicLog):
    msg_type, header = GPSRAW, GPS_HEADER
    _origin = None

    def _values(self, msg):
        lat, lon, alt = msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3
        local = ["", "", ""]  # no local position before the first fix
        if msg.fix_type >= GPSRAW.GPS_FIX_TYPE_2D_FIX:
            if self._origin is None:
                self._origin = (lat, lon, alt)
            lat0, lon0, alt0 = self._origin
            east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
            north = math.radians(lat - lat0) * EARTH_RADIUS_M
            local = ["{:.4f}".format(a) for a in (east, north, alt - alt0)]
        # 65535 marks an unknown eph / epv / vel / cog in GPS_RAW_INT.
        scaled = ["" if raw == 65535 else "{:.2f}".format(raw * 0.01) for raw in (msg.eph, msg.epv, msg.vel, msg.cog)]
        return local + ["{:.7f}".format(lat), "{:.7f}".format(lon), "{:.3f}".format(alt),
                        msg.fix_type, msg.satellites_visible] + scaled


def fc_log_start(directory, stamp):
    """Start SD-card logging on the FC; returns the shell, or None when the FC did not answer."""
    shell = Px4Shell()
    shell.run("logger on")
    status = shell.run("logger status")
    found = re.search(r"(/fs/microsd/\S+\.ulg)", status)
    if not found:
        print("FC log: not started (no answer from the FC shell; is MAVROS connected?)")
        return None
    print("FC log: {}".format(found.group(1)))
    (directory / "fc_ulog_{}.txt".format(stamp)).write_text(found.group(1) + "\n")
    return shell


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, help="output directory (default: output/odom_logs/<time>)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (default: until Ctrl+C)")
    parser.add_argument("--topic", action="append", default=[], metavar="NAME=TOPIC",
                        help="extra Odometry topic -> NAME_<time>.csv, e.g. fc_mavlink_odom=/mavros/odometry/in")
    parser.add_argument("--no-fc-log", action="store_true", help="do not start SD-card logging on the FC")
    args = parser.parse_args(rospy.myargv()[1:])

    topics = list(TOPICS)
    for item in args.topic:
        name, sep, topic = item.partition("=")
        if not sep or not name or not topic.startswith("/"):
            parser.error("--topic expects NAME=/topic, got: {}".format(item))
        topics.append((name, topic))

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = args.output or WS / "output" / "odom_logs" / stamp
    directory = directory.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)

    # rospy's own SIGINT handler would shut the node down before `logger off` can be published:
    # keep the signals, stop through KeyboardInterrupt and shut down at the very end.
    rospy.init_node("record_odom_csv", anonymous=True, disable_signals=True)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    fc_shell = None if args.no_fc_log else fc_log_start(directory, stamp)
    logs = [TopicLog(name, topic, directory, stamp) for name, topic in topics]
    logs.append(GpsLog(GPS_NAME, GPS_TOPIC, directory, stamp))
    print("Recording to {} (Ctrl+C to stop)".format(directory))

    start, last, ticks = time.time(), [0] * len(logs), 0
    try:
        while not rospy.is_shutdown() and (args.duration <= 0 or time.time() - start < args.duration):
            time.sleep(1.0)
            ticks += 1
            for log in logs:
                log.flush()
            if ticks == 5:
                for log in logs:
                    if not log.count:
                        print("WARNING: no messages on {} after 5 s ({} stays empty)".format(log.topic, log.path.name))
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
        if fc_shell is not None:
            fc_shell.run("logger off")
            stopped = "Not logging" in fc_shell.run("logger status")
            print("FC log: stopped" if stopped else "FC log: could NOT be stopped (FC shell did not answer)")
        rospy.signal_shutdown("recording finished")

    print("Saved:")
    for log in logs:
        note = "" if log.count else "   <- no messages on {}".format(log.topic)
        print("  {}  {} rows{}".format(log.path, log.count, note))


if __name__ == "__main__":
    main()
