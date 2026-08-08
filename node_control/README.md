# node_control (ROS 1 Noetic)

This catkin workspace controls PX4 through the MAVROS instance running in the
same Docker container. It subscribes to `/mavros/local_position/odom` and
publishes position setpoints to `/mavros/setpoint_position/local` at 50 Hz.
On startup it requests PX4 to send MAVLink `LOCAL_POSITION_NED` (message ID 32)
at 30 Hz through `/mavros/set_message_interval`.

## Build in Docker

```bash
docker exec -it vins_d435i_local bash
source /opt/ros/noetic/setup.bash
cd /work/node_control
catkin build node_control
source devel/setup.bash
```

## Run

Keep the existing MAVROS ROS 1 launch running in another Docker terminal, then
run:

```bash
source /opt/ros/noetic/setup.bash
source /work/node_control/devel/setup.bash
rosrun node_control node_control.py
```

Each run creates `/work/node_control/output/<timestamp>/odometry.csv`. Give a
run a meaningful name with:

```bash
rosrun node_control node_control.py _run_name:=flight_01
```

Logging stops and the CSV is closed as soon as the node switches to its final
descent/disarm phase (including a target-timeout descent). The node continues
to publish the descent setpoint; it does not arm or disarm PX4 itself.

Generate a Z-time plot and an XY-plane plot for every old, ungrouped log:

```bash
rosrun node_control plot_orphaned_logs.py \
  --input /work/node_control/odom_listener_command.csv \
  --output-root /work/node_control/output
```

The plotting script copies each recovered run into its own output directory and
creates `trajectory.png` beside its `odometry.csv` file.

The node does not arm PX4 or switch it to OFFBOARD mode; it only sends position
setpoints after receiving odometry.



# build
source /opt/ros/noetic/setup.bash
cd /work/node_control
catkin build node_control
source devel/setup.bash

rosrun node_control node_control.py
