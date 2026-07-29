#!/usr/bin/env python3
"""Measure ROS timestamp age and VINS -> MAVROS odometry arrival skew.

This node is intentionally read-only: it never modifies or republishes a
message.  For each topic it measures age using rospy.Time.now(), tracks stamp
continuity, and pairs equal input/output stamps to measure signed observer
arrival skew.  The paired skew includes two independent TCPROS paths and
callback scheduling; it is not an intrinsic bridge processing-time probe.
"""

import argparse
import collections
import math
import statistics
import threading
import time

import rospy
from nav_msgs.msg import Odometry


NSEC_PER_SEC = 1_000_000_000


def format_ros_time(stamp):
    return "{}.{:09d}".format(stamp.secs, stamp.nsecs)


def finite_stats(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return float("nan"), float("nan"), float("nan")
    return min(finite), statistics.fmean(finite), max(finite)


def format_value(value, digits=3):
    if not math.isfinite(value):
        return "n/a"
    return ("{:." + str(digits) + "f}").format(value)


class TopicStats:
    def __init__(self, label, window_size, jump_threshold_ms):
        self.label = label
        self.window_size = window_size
        self.jump_threshold_ms = jump_threshold_ms
        self.ages_ms = collections.deque(maxlen=window_size)
        self.stamp_deltas_ms = collections.deque(maxlen=window_size)
        self.arrival_deltas_ms = collections.deque(maxlen=window_size)
        self.stamps_ns = collections.deque(maxlen=window_size)
        self.received = 0
        self.backward = 0
        self.duplicate = 0
        self.forward_jumps = 0
        self.zero_stamps = 0
        self.negative_ages = 0
        self.last_stamp_ns = None
        self.last_arrival_monotonic = None
        self.latest = None

    def add(self, message, now_ros, now_monotonic):
        stamp = message.header.stamp
        stamp_ns = stamp.to_nsec()
        age_ms = (now_ros - stamp).to_sec() * 1000.0
        stamp_delta_ms = float("nan")
        arrival_delta_ms = float("nan")

        self.received += 1
        self.ages_ms.append(age_ms)
        self.stamps_ns.append(stamp_ns)

        if stamp_ns == 0:
            self.zero_stamps += 1
        if age_ms < 0.0:
            self.negative_ages += 1

        if self.last_stamp_ns is not None:
            stamp_delta_ms = (stamp_ns - self.last_stamp_ns) / 1.0e6
            self.stamp_deltas_ms.append(stamp_delta_ms)
            if stamp_ns < self.last_stamp_ns:
                self.backward += 1
            elif stamp_ns == self.last_stamp_ns:
                self.duplicate += 1
            elif stamp_delta_ms > self.jump_threshold_ms:
                self.forward_jumps += 1

        if self.last_arrival_monotonic is not None:
            arrival_delta_ms = (
                now_monotonic - self.last_arrival_monotonic
            ) * 1000.0
            self.arrival_deltas_ms.append(arrival_delta_ms)

        self.last_stamp_ns = stamp_ns
        self.last_arrival_monotonic = now_monotonic
        self.latest = {
            "stamp": stamp,
            "now": now_ros,
            "age_ms": age_ms,
            "stamp_delta_ms": stamp_delta_ms,
            "arrival_delta_ms": arrival_delta_ms,
        }

        return {
            "stamp_ns": stamp_ns,
            "ros_time_ns": now_ros.to_nsec(),
            "monotonic": now_monotonic,
            "age_ms": age_ms,
        }

    def frequency_hz(self, deltas_ms):
        positive = [
            value for value in deltas_ms
            if math.isfinite(value) and value > 0.0
        ]
        if not positive:
            return float("nan")
        return 1000.0 / statistics.fmean(positive)

    def report(self):
        if self.latest is None:
            return "[{}] waiting for messages".format(self.label)

        age_min, age_mean, age_max = finite_stats(self.ages_ms)
        stamp_hz = self.frequency_hz(self.stamp_deltas_ms)
        arrival_hz = self.frequency_hz(self.arrival_deltas_ms)
        latest = self.latest
        return (
            "[{label}] n={received} stamp={stamp} now={now} "
            "age={age} ms dt_stamp={dt_stamp} ms dt_arrival={dt_arrival} ms "
            "freq_stamp={stamp_hz} Hz freq_arrival={arrival_hz} Hz "
            "age_window[{window}] min/mean/max={age_min}/{age_mean}/{age_max} ms "
            "anomalies(back/dup/jump>{jump}ms/zero/negative_age)="
            "{back}/{dup}/{forward}/{zero}/{negative}"
        ).format(
            label=self.label,
            received=self.received,
            stamp=format_ros_time(latest["stamp"]),
            now=format_ros_time(latest["now"]),
            age=format_value(latest["age_ms"]),
            dt_stamp=format_value(latest["stamp_delta_ms"]),
            dt_arrival=format_value(latest["arrival_delta_ms"]),
            stamp_hz=format_value(stamp_hz, 2),
            arrival_hz=format_value(arrival_hz, 2),
            window=len(self.ages_ms),
            age_min=format_value(age_min),
            age_mean=format_value(age_mean),
            age_max=format_value(age_max),
            jump=format_value(self.jump_threshold_ms, 1),
            back=self.backward,
            dup=self.duplicate,
            forward=self.forward_jumps,
            zero=self.zero_stamps,
            negative=self.negative_ages,
        )


class StampMatcher:
    """Pair input/output observations that have exactly equal header stamps."""

    def __init__(self, window_size, pending_limit):
        self.latency_ms = collections.deque(maxlen=window_size)
        self.ros_age_delta_ms = collections.deque(maxlen=window_size)
        self.pending_input = collections.OrderedDict()
        self.pending_output = collections.OrderedDict()
        self.pending_limit = pending_limit
        self.matched = 0
        self.output_before_observer_input = 0
        self.latest_stamp_ns = None
        self.latest_latency_ms = float("nan")

    @staticmethod
    def _append_pending(pending, sample):
        stamp_ns = sample["stamp_ns"]
        if stamp_ns not in pending:
            pending[stamp_ns] = collections.deque()
        pending[stamp_ns].append(sample)
        pending.move_to_end(stamp_ns)

    def _trim(self, pending):
        while sum(len(samples) for samples in pending.values()) > self.pending_limit:
            oldest_stamp = next(iter(pending))
            samples = pending[oldest_stamp]
            samples.popleft()
            if not samples:
                del pending[oldest_stamp]

    def _match_stamp(self, stamp_ns):
        inputs = self.pending_input.get(stamp_ns)
        outputs = self.pending_output.get(stamp_ns)
        while inputs and outputs:
            input_sample = inputs.popleft()
            output_sample = outputs.popleft()
            latency_ms = (
                output_sample["monotonic"] - input_sample["monotonic"]
            ) * 1000.0
            age_delta_ms = (
                output_sample["age_ms"] - input_sample["age_ms"]
            )
            self.latency_ms.append(latency_ms)
            self.ros_age_delta_ms.append(age_delta_ms)
            self.matched += 1
            self.latest_stamp_ns = stamp_ns
            self.latest_latency_ms = latency_ms
            if latency_ms < 0.0:
                self.output_before_observer_input += 1

        if inputs is not None and not inputs:
            del self.pending_input[stamp_ns]
        if outputs is not None and not outputs:
            del self.pending_output[stamp_ns]

    def add_input(self, sample):
        self._append_pending(self.pending_input, sample)
        self._match_stamp(sample["stamp_ns"])
        self._trim(self.pending_input)

    def add_output(self, sample):
        self._append_pending(self.pending_output, sample)
        self._match_stamp(sample["stamp_ns"])
        self._trim(self.pending_output)

    @staticmethod
    def _pending_count(pending):
        return sum(len(samples) for samples in pending.values())

    def report(self):
        latency_min, latency_mean, latency_max = finite_stats(self.latency_ms)
        age_min, age_mean, age_max = finite_stats(self.ros_age_delta_ms)
        stamp = "n/a"
        if self.latest_stamp_ns is not None:
            secs, nsecs = divmod(self.latest_stamp_ns, NSEC_PER_SEC)
            stamp = "{}.{:09d}".format(secs, nsecs)
        return (
            "[PAIR exact-stamp observer] matched={matched} latest_stamp={stamp} "
            "latest_arrival_skew={latest} ms "
            "signed_arrival_skew_window[{window}] min/mean/max="
            "{lat_min}/{lat_mean}/{lat_max} ms "
            "output_age-input_age min/mean/max="
            "{age_min}/{age_mean}/{age_max} ms "
            "pending(input/output)={pending_input}/{pending_output} "
            "observer_reordered={reordered}"
        ).format(
            matched=self.matched,
            stamp=stamp,
            latest=format_value(self.latest_latency_ms),
            window=len(self.latency_ms),
            lat_min=format_value(latency_min),
            lat_mean=format_value(latency_mean),
            lat_max=format_value(latency_max),
            age_min=format_value(age_min),
            age_mean=format_value(age_mean),
            age_max=format_value(age_max),
            pending_input=self._pending_count(self.pending_input),
            pending_output=self._pending_count(self.pending_output),
            reordered=self.output_before_observer_input,
        )


class TimestampDiagnostic:
    def __init__(self, args):
        self.lock = threading.Lock()
        self.input_stats = TopicStats(
            "VINS", args.window, args.jump_threshold_ms
        )
        self.output_stats = TopicStats(
            "BRIDGE", args.window, args.jump_threshold_ms
        )
        self.matcher = StampMatcher(args.window, max(args.window * 5, 1000))
        self.input_subscriber = rospy.Subscriber(
            args.input_topic,
            Odometry,
            self.input_callback,
            queue_size=args.queue_size,
            buff_size=args.buffer_bytes,
            tcp_nodelay=True,
        )
        self.output_subscriber = rospy.Subscriber(
            args.output_topic,
            Odometry,
            self.output_callback,
            queue_size=args.queue_size,
            buff_size=args.buffer_bytes,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(args.report_period),
            self.timer_callback,
        )
        rospy.loginfo(
            "Read-only timestamp diagnostic: %s and %s; window=%d, "
            "subscriber_queue=%d. Exact-stamp pair delta is signed observer "
            "arrival skew, not intrinsic bridge processing time.",
            args.input_topic,
            args.output_topic,
            args.window,
            args.queue_size,
        )

    def input_callback(self, message):
        now_ros = rospy.Time.now()
        now_monotonic = time.monotonic()
        with self.lock:
            sample = self.input_stats.add(message, now_ros, now_monotonic)
            self.matcher.add_input(sample)

    def output_callback(self, message):
        now_ros = rospy.Time.now()
        now_monotonic = time.monotonic()
        with self.lock:
            sample = self.output_stats.add(message, now_ros, now_monotonic)
            self.matcher.add_output(sample)

    def timer_callback(self, _event):
        with self.lock:
            lines = [
                self.input_stats.report(),
                self.output_stats.report(),
                self.matcher.report(),
            ]
        rospy.loginfo("\n%s", "\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure VINS and MAVROS odometry timestamp age without modifying messages."
    )
    parser.add_argument(
        "--input-topic",
        default="/vins_estimator/odometry",
        help="VINS odometry topic",
    )
    parser.add_argument(
        "--output-topic",
        default="/mavros/odometry/out",
        help="Bridge output topic",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=200,
        help="Sliding statistics window (default: 200 samples)",
    )
    parser.add_argument(
        "--report-period",
        type=float,
        default=2.0,
        help="Seconds between reports",
    )
    parser.add_argument(
        "--jump-threshold-ms",
        type=float,
        default=100.0,
        help="Count a positive stamp interval above this value as a jump",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=1,
        help=(
            "Diagnostic subscriber queue (default: 1 to avoid diagnostic-side "
            "backlog; the statistics window remains 200)"
        ),
    )
    parser.add_argument(
        "--buffer-bytes",
        type=int,
        default=1048576,
        help="TCPROS receive buffer bytes (default: 1048576)",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if args.window < 2:
        parser.error("--window must be at least 2")
    if args.report_period <= 0.0:
        parser.error("--report-period must be positive")
    if args.jump_threshold_ms <= 0.0:
        parser.error("--jump-threshold-ms must be positive")
    if args.queue_size < 1:
        parser.error("--queue-size must be at least 1")
    if args.buffer_bytes < 65536:
        parser.error("--buffer-bytes must be at least 65536")
    return args


def main():
    args = parse_args()
    rospy.init_node("ev_timestamp_diagnostic", anonymous=False)
    TimestampDiagnostic(args)
    rospy.spin()


if __name__ == "__main__":
    main()
