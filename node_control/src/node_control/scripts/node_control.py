#!/usr/bin/env python3

import csv
from datetime import datetime
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from nav_msgs.msg import Odometry


def workspace_root():
    script_path = Path(__file__).resolve()
    for parent in script_path.parents:
        if (parent / 'src' / 'node_control' / 'package.xml').is_file():
            return parent
    return Path.cwd()


class NodeControl:
    def __init__(self):
        self.initial_target = None
        self.next_target = None
        self.lowered_target = None
        self._active_target = None
        self._initial_height = None
        self._inside_target_since_ns = None
        self._target_started_ns = None
        self._timeout_descent_active = False
        self._flight_control_active = False

        self._latest_odom = None
        self._last_log_time_ns = None
        self._target_yaw = None
        self._message_interval_configured = False
        self._logging_active = True

        self._run_dir = self._create_run_dir()
        self._log_path = self._run_dir / 'odometry.csv'
        self._log_file = self._log_path.open(
            'x',
            newline='',
            encoding='utf-8',
        )
        self._csv_writer = csv.writer(self._log_file)
        self._csv_writer.writerow(
            ('timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
             'target_x', 'target_y', 'target_z', 'current_yaw', 'target_yaw')
        )
        rospy.loginfo('logging this run to %s', self._log_path)

        self._odom_subscription = rospy.Subscriber(
            '/mavros/local_position/odom',
            Odometry,
            self._on_odometry,
            queue_size=10,
        )
        self._state_subscription = rospy.Subscriber(
            '/mavros/state',
            State,
            self._on_state,
            queue_size=10,
        )
        self._setpoint_publisher = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10,
        )
        self._setpoint_timer = rospy.Timer(
            rospy.Duration(1.0 / 50.0),
            self._publish_setpoint,
        )
        self._set_message_interval = rospy.ServiceProxy(
            '/mavros/set_message_interval',
            MessageInterval,
        )
        self._message_interval_timer = rospy.Timer(
            rospy.Duration(1.0),
            self._request_local_position_stream,
        )

    @staticmethod
    def _create_run_dir():
        output_root = Path(rospy.get_param(
            '~output_root',
            str(workspace_root() / 'output'),
        )).expanduser()
        requested_name = str(rospy.get_param(
            '~run_name',
            datetime.now().strftime('%Y%m%d_%H%M%S'),
        ))
        if not requested_name or Path(requested_name).name != requested_name:
            raise ValueError('~run_name must be a single directory name')

        output_root.mkdir(parents=True, exist_ok=True)
        run_dir = output_root / requested_name
        suffix = 1
        while run_dir.exists():
            run_dir = output_root / f'{requested_name}_{suffix:02d}'
            suffix += 1
        run_dir.mkdir()
        return run_dir

    def _request_local_position_stream(self, _event):
        if self._message_interval_configured:
            return

        try:
            response = self._set_message_interval(
                message_id=32,
                message_rate=30.0,
            )
        except rospy.ServiceException:
            rospy.logwarn_throttle(
                5.0,
                'waiting for /mavros/set_message_interval',
            )
            return

        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                'PX4 rejected LOCAL_POSITION_NED at 30 Hz',
            )
            return

        self._message_interval_configured = True
        self._message_interval_timer.shutdown()
        rospy.loginfo('requested PX4 LOCAL_POSITION_NED (ID 32) at 30 Hz')

    def _on_state(self, msg):
        flight_control_active = msg.armed and msg.mode == 'OFFBOARD'
        if flight_control_active == self._flight_control_active:
            return

        self._flight_control_active = flight_control_active
        self._inside_target_since_ns = None
        self._target_started_ns = None
        if flight_control_active:
            rospy.loginfo('OFFBOARD and armed: starting target timeout timers')
        else:
            rospy.loginfo('OFFBOARD/armed inactive: pausing target timeout timers')

    def _on_odometry(self, msg):
        self._latest_odom = msg

        if self.initial_target is None:
            position = msg.pose.pose.position
            self._initial_height = position.z
            self.initial_target = (
                position.x,
                position.y,
                position.z + 0.3,
            )
            self.next_target = (
                position.x,
                position.y + 0.5,
                position.z + 0.3,
            )
            self.lowered_target = (
                position.x,
                position.y + 0.5,
                position.z,
            )
            self._active_target = self.initial_target

        if self._target_yaw is None:
            q = msg.pose.pose.orientation
            self._target_yaw = self._yaw_from_orientation(q)

        now_ns = rospy.Time.now().to_nsec()
        self._update_target(msg, now_ns)

        if self._logging_active and (
            self._last_log_time_ns is None
            or now_ns - self._last_log_time_ns >= 1_000_000_000
        ):
            self._write_odometry_log(msg, now_ns)
            target_x, target_y, target_z = self._active_target
            current_yaw = self._yaw_from_orientation(msg.pose.pose.orientation)
            rospy.loginfo(
                'current_target=(%.1f, %.1f, %.1f) '
                'current_yaw=%.3f rad target_yaw=%.3f rad',
                target_x,
                target_y,
                target_z,
                current_yaw,
                self._target_yaw,
            )
            self._last_log_time_ns = now_ns

    @staticmethod
    def _yaw_from_orientation(orientation):
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _update_target(self, odom, now_ns):
        if (
            self._active_target is None
            or self.next_target is None
            or self.lowered_target is None
            or self._initial_height is None
        ):
            return

        if self._timeout_descent_active or self._active_target == self.lowered_target:
            return

        if not self._flight_control_active:
            self._inside_target_since_ns = None
            self._target_started_ns = None
            return

        position = odom.pose.pose.position
        distance = math.dist(
            (position.x, position.y, position.z),
            self._active_target,
        )
        if self._target_started_ns is None:
            self._target_started_ns = now_ns

        if distance >= 0.15:
            self._inside_target_since_ns = None
            if now_ns - self._target_started_ns >= 25_000_000_000:
                self._active_target = (
                    position.x,
                    position.y,
                    self._initial_height,
                )
                self._timeout_descent_active = True
                self._stop_logging('target timeout; switching to descent')
                rospy.logwarn(
                    'target timeout after 25 seconds; descending to initial height'
                )
            return

        if self._inside_target_since_ns is None:
            self._inside_target_since_ns = now_ns
        elif now_ns - self._inside_target_since_ns >= 5_000_000_000:
            if self._active_target == self.initial_target:
                self._active_target = self.next_target
            elif self._active_target == self.next_target:
                self._active_target = self.lowered_target
                self._stop_logging('switching to final descent/disarm phase')
            self._inside_target_since_ns = None
            self._target_started_ns = None

    def _publish_setpoint(self, _event):
        odom = self._latest_odom
        yaw = self._target_yaw
        target = self._active_target
        if odom is None or yaw is None or target is None:
            return

        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = odom.header.frame_id or 'map'
        (
            setpoint.pose.position.x,
            setpoint.pose.position.y,
            setpoint.pose.position.z,
        ) = target
        setpoint.pose.orientation.z = math.sin(yaw / 2.0)
        setpoint.pose.orientation.w = math.cos(yaw / 2.0)

        self._setpoint_publisher.publish(setpoint)

    def _write_odometry_log(self, odom, timestamp_ns):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        target_x, target_y, target_z = self._active_target
        current_yaw = self._yaw_from_orientation(orientation)

        self._csv_writer.writerow(
            (
                f'{timestamp_ns / 1e9:.9f}',
                f'{position.x:.6f}',
                f'{position.y:.6f}',
                f'{position.z:.6f}',
                f'{orientation.x:.7f}',
                f'{orientation.y:.7f}',
                f'{orientation.z:.7f}',
                f'{orientation.w:.7f}',
                f'{target_x:.6f}',
                f'{target_y:.6f}',
                f'{target_z:.6f}',
                f'{current_yaw:.7f}',
                f'{self._target_yaw:.7f}',
            )
        )
        self._log_file.flush()

    def _stop_logging(self, reason):
        if not self._logging_active:
            return

        self._logging_active = False
        self._log_file.close()
        rospy.loginfo('stopped odometry logging: %s', reason)

    def shutdown(self):
        if self._logging_active:
            self._log_file.close()


def main():
    rospy.init_node('node_control')
    node = NodeControl()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
