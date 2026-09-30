# Docker (chạy trên host)
```bash
cd ~/huy/vins_fusion_d435i

docker compose up -d                        # bật container (nếu chưa chạy)
docker compose ps                           # xem container
docker exec -it vins_d435i_compose bash     # mở shell trong container
docker compose up -d --force-recreate       # tạo lại container sau khi sửa compose.yaml
docker compose build                        # build lại image sau khi sửa Dockerfile
```
Nếu `docker` báo `permission denied`: logout/login lại, hoặc tạm dùng `newgrp docker`.

# Kiểm tra phần cứng (chạy trên host)
```bash
lsusb -t | grep -A1 uvcvideo | head -2      # cam phải là 5000M (USB 3), 480M là USB 2
ls -l /dev/ttyTHS1                          # FC nối qua GPIO header (chân 8/10)
```
Trong container:
```bash
rs-enumerate-devices -s                     # phải thấy Intel RealSense D435I
rs-enumerate-devices | grep "Usb Type"      # phải là 3.2
```

# open cam
```bash
source /opt/ros/noetic/setup.bash
source /home/air/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/home/air/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

roslaunch /home/air/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch
```
# run VINS_Fusion
```bash
source /opt/ros/noetic/setup.bash
source /home/air/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

mkdir -p /home/air/vins_fusion_d435i_local/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /home/air/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
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
```bash
docker exec -it vins_d435i_compose bash
```


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
source /home/air/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/home/air/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

roslaunch /home/air/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch
```


### Terminal 2: VINS-Fusion

Chờ camera đã publish image và IMU rồi chạy:

```bash
source /opt/ros/noetic/setup.bash
source /home/air/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

mkdir -p /home/air/vins_fusion_d435i_local/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /home/air/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
```

### Terminal 3: MAVROS kết nối PX4

```bash
source /opt/ros/noetic/setup.bash

# FC nối qua GPIO UART (chân 8/10 header 40-pin)
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

Nếu flight controller dùng cổng khác, thay `fcu_url` bằng thiết bị thực tế:
- USB native: `/dev/ttyACM0:921600`
- CP2102: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600`

### Terminal 4: Bridge MAVLink ODOMETRY

```bash
source /opt/ros/noetic/setup.bash
source /home/air/vins_fusion_d435i_local/catkin_ws/devel/setup.bash

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
rostopic hz /mavros/local_position/pose
```
Kiểm tra param EKF2 của PX4 (cần fuse vision):
```bash
rosservice call /mavros/param/get EKF2_EV_CTRL
rosservice call /mavros/param/get EKF2_HGT_REF    # 3 = vision height
rosservice call /mavros/param/get EKF2_EV_DELAY
```

# Scripts (chạy trên host, trong ~/huy/vins_fusion_d435i)
```bash
./scripts/run.sh vins                 # tmux: camera + VINS
./scripts/run.sh px4                  # tmux: camera + VINS + MAVROS (/dev/ttyTHS1:921600) + bridge
./scripts/run.sh px4 --fcu /dev/ttyACM0:921600
./scripts/run.sh record               # tmux: camera RGB+IR+IMU + VINS + rosbag -> bags/recordings/
./scripts/run.sh stop                 # LUÔN dừng bằng lệnh này (Ctrl+C từng node, lưu bag, đóng tmux)
./scripts/run.sh px4 --dry-run        # chỉ in lệnh
# run.sh cũng chạy được bên trong container (tmux có sẵn trong image):
#   docker exec -it vins_d435i_compose bash  ->  ./scripts/run.sh vins

./scripts/replay_bag_vins.sh bags/recordings/<bag>.bag [--rate 2] [--output output/x.bag]
./scripts/record_webcam.sh [cam1|cam2|all]
```
Trong container (`cd ~/vins_fusion_d435i_local`):
```bash
python3 scripts/plot_odometry.py <bag|csv> [--csv out.csv] [--absolute]
python3 scripts/bag_to_video.py raw   <bag> out.mp4 [--topic T] [--right-topic T2]   # RGB / stereo
python3 scripts/bag_to_video.py track <bag|video> out.mp4 [--draw-trails]          # feature tracking
python3 scripts/bag_to_video.py vins  <bag> out.mp4 [--csv out.csv]                # cần /vins_estimator/image_track
python3 scripts/bag_to_video.py live  out.mp4 --topic /camera/color/image_raw       # ghi topic live, Ctrl+C dừng
```
Bag `record` khá nặng: ~25 MB/s (RGB + 2 IR + IMU).

# Lỗi thường gặp
- `/camera/imu` không có data (VINS in mãi `wait for imu ...`), image vẫn 30 Hz:
  `./scripts/run.sh stop` rồi chạy lại (hoặc Ctrl+C camera rồi chạy lại Terminal 1), kiểm tra lại `rostopic hz /camera/imu` (~400 Hz).
- VINS odom trôi xa khi cam đứng yên: lắc/xoay cam vài giây lúc khởi tạo,
  hướng vào cảnh nhiều chi tiết, kiểm tra odom ổn định trước khi cho PX4 dùng.
- `vins_node` in `Aborted (core dumped)` khi Ctrl+C: bình thường.

# Build lại workspace (trong container, khi sửa source)
```bash
source /opt/ros/noetic/setup.bash
cd ~/vins_fusion_d435i_local/rs_ros_ws && catkin build -j3    # driver RealSense
cd ~/vins_fusion_d435i_local/catkin_ws && catkin build -j3    # VINS-Fusion + bridge
```
