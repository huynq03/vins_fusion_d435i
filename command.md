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
ls -l /dev/serial/by-id/                    # FC nối qua USB-TTL CP2102 (-> ttyUSB0)
```
Trong container:
```bash
rs-enumerate-devices -s                     # phải thấy Intel RealSense D435I
rs-enumerate-devices | grep "Usb Type"      # phải là 3.2
```

# Chạy nhanh: chỉ bật cam + VINS
Trên host:
```bash
cd ~/huy/vins_fusion_d435i
./scripts/run.sh vins
```
Hoặc trong container:
```bash
docker exec -it vins_d435i_compose bash
./scripts/run.sh vins
```
tmux `vins` có 2 pane: CAMERA (`rs_camera.launch`) và VINS (tự chờ có IR + IMU rồi mới chạy `vins_node`;
quá 30 s không có data thì báo lỗi và giữ log).
```bash
./scripts/run.sh vins --reset     # reset D435i lúc khởi động (khi cam lỗi USB)
./scripts/run.sh vins --dry-run   # chỉ in lệnh, không chạy
./scripts/run.sh stop             # dừng cam + VINS, đóng tmux (LUÔN dừng bằng lệnh này)
tmux a -t vins                    # vào lại sau khi Ctrl+b d
```
Kiểm tra odometry (terminal khác, trong container):
```bash
source /opt/ros/noetic/setup.bash
rostopic hz /vins_estimator/odometry    # ~30 Hz
```
Lúc khởi động lắc/xoay cam vài giây để VINS khởi tạo tốt.

# Chạy tất cả: cam + VINS + MAVROS + gửi odom sang FC
```bash
cd ~/huy/vins_fusion_d435i
./scripts/run.sh px4                              # FC qua USB-TTL CP2102, 921600 (mặc định)
./scripts/run.sh px4 --fcu /dev/ttyACM0:921600    # FC qua USB-C native của FC
./scripts/run.sh px4 --reset                      # reset D435i lúc khởi động
./scripts/run.sh stop                             # dừng tất cả
```
Chạy pipeline + ghi odom (GPS raw, VINS raw, FC fuse) chỉ cần 2 lệnh:
```bash
./scripts/run.sh px4          # 1. pipeline, tmux `vins`
./scripts/record_odom.sh      # 2. ghi CSV -> output/odom_logs/<ngày_giờ>/, tmux riêng `odom_rec` (gõ ở terminal khác hoặc sau Ctrl+b d)
tmux a -t odom_rec            # vào lại tmux ghi; Ctrl+C trong đó để dừng ghi và lưu file
./scripts/run.sh stop         # dừng cả pipeline lẫn ghi (file vẫn được lưu)
```
Vẽ hình so sánh GPS raw / VINS raw / FC fuse từ thư mục vừa ghi (chạy trên host, kết quả là `odom_compare_<ngày_giờ>.png` trong chính thư mục đó):
```bash
ls output/odom_logs/          # xem tên thư mục <ngày_giờ> vừa ghi
docker exec -w /home/air/vins_fusion_d435i_local vins_d435i_compose \
  python3 scripts/plot_odom_compare.py output/odom_logs/<ngày_giờ>
# thêm --relative: trừ vị trí đầu của từng nguồn;  --no-gps-align: không xoay/tịnh tiến GPS khớp vào VINS
```
Chi tiết file và cách vẽ: xem mục "Lưu odom VINS + odom FC + GPS raw ra CSV" bên dưới.

tmux `vins` có 4 pane:
1. CAMERA: `rs_camera.launch`
2. VINS: chờ IR + IMU rồi chạy `vins_node`
3. MAVROS: `mavros px4.launch fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-port0:921600`
4. BRIDGE: chờ `/vins_estimator/odometry` rồi chạy `vins_px4_bridge` -> `/mavros/odometry/out`

Kiểm tra (terminal khác, trong container):
```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n1 /mavros/state            # connected: True
rostopic hz /vins_estimator/odometry       # ~30 Hz
rostopic hz /mavros/odometry/out           # ~30 Hz, odom đang gửi sang FC
rostopic hz /mavros/local_position/pose    # EKF2 của PX4 có output
```
Nếu `connected: False`: kiểm tra FC có nguồn / dây UART / USB-TTL đã cắm (`ls /dev/ttyUSB0`), trong container
(khi MAVROS đang tắt) chạy `stty -F /dev/ttyUSB0 921600 raw && timeout 2 cat /dev/ttyUSB0 | wc -c` phải > 0.
Không dùng GPIO UART `/dev/ttyTHS1` ở 921600: đo được FC chỉ nhận ~50% gói Jetson gửi lên (mất nửa odom,
FC báo `Connection to mission computer lost`), trong khi `rostopic hz /mavros/odometry/out` vẫn hiện 30 Hz.
Trước khi bay: odom VINS phải ổn định, và PX4 phải bật fuse vision (`EKF2_EV_CTRL`, xem mục Terminal 5).

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

# FC (TELEM1) nối qua USB-TTL CP2102
roslaunch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600
```

Nếu flight controller dùng cổng khác, thay `fcu_url` bằng thiết bị thực tế:
- USB-C native của FC: `/dev/ttyACM0:921600`
- GPIO UART (chân 8/10 header 40-pin): `/dev/ttyTHS1:921600` — mất ~50% gói chiều Jetson -> FC, không nên dùng

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
./scripts/run.sh px4                  # tmux: camera + VINS + MAVROS (USB-TTL CP2102, 921600) + bridge
./scripts/run.sh px4 --fcu /dev/ttyACM0:921600
./scripts/run.sh record               # tmux: camera RGB+IR+IMU + VINS + rosbag -> bags/recordings/
./scripts/run.sh stop                 # LUÔN dừng bằng lệnh này (Ctrl+C từng node, lưu bag, đóng tmux)
./scripts/run.sh px4 --dry-run        # chỉ in lệnh
# run.sh cũng chạy được bên trong container (tmux có sẵn trong image):
#   docker exec -it vins_d435i_compose bash  ->  ./scripts/run.sh vins

./scripts/replay_bag_vins.sh bags/recordings/<bag>.bag [--rate 2] [--output output/x.bag]
./scripts/record_odom.sh [--duration 60] [--output output/odom_logs/x]   # odom VINS + FC + GPS raw -> CSV, tmux riêng odom_rec
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

# Topic odom từ FC về
Có khi pipeline đang chạy (`./scripts/run.sh px4`); đo qua USB-TTL 921600 đều ~30 Hz.

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | EKF2 của PX4 đã fuse: vị trí + hướng (ENU), vận tốc theo body FLU. Dùng topic này |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | chỉ vị trí + hướng |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | vận tốc theo trục world ENU |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | bản tin MAVLink `ODOMETRY` của FC, có covariance |

`/mavros/odometry/out` là chiều ngược lại (odom VINS gửi lên FC), không phải odom của FC.
```bash
source /opt/ros/noetic/setup.bash
rostopic hz /mavros/local_position/odom         # ~30 Hz
rostopic echo -n1 /mavros/local_position/odom
```

# Lưu odom VINS + odom FC + GPS raw ra CSV
Chỉ cần 2 lệnh, trên host hoặc trong container:
```bash
./scripts/run.sh px4                        # 1. toàn bộ pipeline (tmux vins)
./scripts/record_odom.sh                    # 2. ghi -> output/odom_logs/<ngày_giờ>/, trong tmux riêng `odom_rec`
./scripts/record_odom.sh --duration 60      # tự dừng sau 60 s
./scripts/record_odom.sh --topic fc_mavlink_odom=/mavros/odometry/in   # ghi thêm topic Odometry khác
./scripts/record_odom.sh --no-tmux          # ghi ngay trong terminal đang gõ, không mở tmux
tmux a -t odom_rec                          # vào lại tmux ghi sau khi Ctrl+b d
```
Lệnh 2 gõ ở terminal khác (hoặc sau khi `Ctrl+b d`); bộ ghi chạy trong tmux `odom_rec`, tách khỏi tmux `vins` của pipeline.
Dừng ghi: Ctrl+C trong tmux `odom_rec`, hoặc `./scripts/run.sh stop` (dừng cả pipeline); cả hai cách đều lưu file.
Sau khi dừng, tmux `odom_rec` tự đóng, terminal đã gõ lệnh trở lại bình thường và in danh sách file đã ghi. Chạy khi pipeline chưa bật thì bộ ghi chờ ROS master.

Khi bắt đầu ghi, script cũng bật ghi log trên thẻ SD của FC (`logger on`, PX4 bình thường chỉ ghi khi arm) và tắt khi dừng (`logger off`).
Đường dẫn file ULog trên FC được in ra và lưu vào `fc_ulog_<ngày_giờ>.txt` trong thư mục ghi; log này chứa trạng thái EKF2 (cờ fuse, innovation).
Log FC khá nặng (~220 kB/s, tức ~20 MB cho 90 s). Không muốn ghi log FC: `./scripts/record_odom.sh --no-fc-log`.
Gõ lệnh NSH của PX4 qua MAVROS (trong container, khi MAVROS đang chạy):
```bash
python3 scripts/px4_shell.py "logger status" "gps status"
```
Tên file có ngày giờ lúc bắt đầu ghi (giờ máy host), ví dụ `fc_odom_20261001_205754.csv`:
- `vins_odom_<ngày_giờ>.csv`: `/vins_estimator/odometry` (VINS raw, body RDF, vận tốc theo world)
- `vins_flu_odom_<ngày_giờ>.csv`: `/mavros/odometry/out` (VINS sau bridge, body FLU, vận tốc theo body)
- `fc_odom_<ngày_giờ>.csv`: `/mavros/local_position/odom` (EKF2 của PX4 đã fuse, ENU/FLU, vận tốc theo body)
- `gps_raw_<ngày_giờ>.csv`: `/mavros/gpsstatus/gps1/raw` (GPS / UWB Nooploop thô, FC chuyển tiếp nguyên trước khi vào EKF2):
  `lat_deg, lon_deg, alt_m, fix_type, ...` và `x_m` (đông), `y_m` (bắc), `z_m` tính từ mẫu đầu tiên có fix; lat/lon bước 1e-7 độ nên x/y có bước ~1 cm.
  File chỉ có header nếu FC không có dữ liệu GPS (kiểm tra `rostopic hz /mavros/gpsstatus/gps1/raw`).

So sánh FC với VINS thì dùng file `fc_odom` và `vins_flu_odom` (cùng quy ước trục).
Vẽ (trong container, `cd ~/vins_fusion_d435i_local`):
```bash
python3 scripts/plot_odom_compare.py output/odom_logs/<ngày_giờ>              # VINS raw + FC fuse chung 1 hình -> odom_compare_<ngày_giờ>.png
python3 scripts/plot_odom_compare.py output/odom_logs/<ngày_giờ> --relative   # trừ vị trí đầu của từng nguồn
python3 scripts/plot_odom_compare.py output/odom_logs/<ngày_giờ> --no-gps-align   # vẽ GPS raw không xoay/tịnh tiến
python3 scripts/plot_odometry.py output/odom_logs/<ngày_giờ>/fc_odom_<ngày_giờ>.csv      # vẽ riêng 1 file
```
Nếu có `gps_raw` thì hình có thêm đường GPS raw và RMS GPS−VINS, GPS−FC. Hệ trục GPS/UWB (đông/bắc) không trùng hệ VINS
(yaw ban đầu tùy ý), nên script tự xoay + tịnh tiến GPS khớp vào quỹ đạo X-Y của VINS (không co giãn) và in góc yaw đã xoay.
Cần di chuyển ít nhất ~0,3 m thì mới tính được góc xoay; RMS GPS là sai lệch còn lại sau khi khớp.

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
