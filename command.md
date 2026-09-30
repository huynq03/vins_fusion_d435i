# open cam
```bash
source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

roslaunch /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch
```
# run VINS_Fusion
```bash
source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

mkdir -p /home/hann/vins_fusion_d435i_local/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
```

# save bags
```bash
mkdir -p bags/recordings

rosbag record --lz4 --buffsize=512 \
  -O "bags/recordings/d435i_$(date +%Y%m%d_%H%M%S).bag" \
  /camera/infra1/image_rect_raw \
  /camera/infra2/image_rect_raw \
  /camera/color/image_raw \
  /camera/imu
```

# attach shell
docker exec -it 8177aa5e34bf bash


# tmux
tmux a : vao cai cu
ctrl +b -> d : exit
ctrl + b -> shift 5 : chia doi
ctrl + b -> shift 4 : remame
ctrl + b -> s : overview all tmux
ctr + b -> mui ten : doi terminal




### Terminal 1: Camera D435i

```bash
source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

roslaunch /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch
```


### Terminal 2: VINS-Fusion

Chờ camera đã publish image và IMU rồi chạy:

```bash
source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

mkdir -p /home/hann/vins_fusion_d435i_local/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
```

### Terminal 3: MAVROS kết nối PX4

```bash
source /opt/ros/noetic/setup.bash

roslaunch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600
```

Nếu flight controller dùng cổng khác, thay `fcu_url` bằng
`/dev/ttyACM0:921600` hoặc thiết bị thực tế.

### Terminal 4: Bridge MAVLink ODOMETRY

```bash
source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

roslaunch vins vins_px4_bridge.launch \
  input_topic:=/vins_estimator/odometry \
  output_topic:=/mavros/odometry/out \
  parent_frame_id:=odom \
  child_frame_id:=base_link
```

### Terminal 5: Kiểm tra pipeline

```bash
source /opt/ros/noetic/setup.bash

rostopic hz /camera/imu
rostopic hz /camera/infra1/image_rect_raw
rostopic hz /camera/color/image_raw
```
```bash
rostopic echo -n1 /mavros/state
rostopic hz /vins_estimator/odometry
rostopic info /mavros/odometry/out
rostopic hz /mavros/odometry/out
rostopic echo -n1 /mavros/odometry/out
```
# cam record
```bash
./record_camera.sh
```
