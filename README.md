# VINS-Fusion + RealSense D435i trên Jetson Orin Nano

Project chạy VINS-Fusion stereo-IMU với Intel RealSense D435i trong Docker ROS Noetic.

## 1. Luồng dữ liệu

```text
D435i
  |
  v
librealsense                    Đọc camera qua USB
  |
  v
realsense2_camera               Chuyển dữ liệu thành ROS topic
  |
  +-- /camera/infra1/image_rect_raw   Camera IR trái, 640x480, 30 Hz
  +-- /camera/infra2/image_rect_raw   Camera IR phải, 640x480, 30 Hz
  +-- /camera/imu                     Gyroscope + accelerometer
  |
  v
VINS-Fusion                     Ghép stereo và IMU
  |
  +-- /vins_estimator/odometry       Pose, velocity, angular velocity
  +-- /vins_estimator/path           Quỹ đạo
  +-- /work/output/.../vio.csv       Kết quả lưu tự động
  |
  v
vins_px4_bridge -> MAVROS -> PX4 EKF2
```

Ba tầng được tách riêng để khóa đúng phiên bản và build độc lập:

1. `librealsense`: SDK giao tiếp với phần cứng.
2. `realsense-ros`: driver publish ROS topic.
3. `VINS-Fusion`: thuật toán tính odometry.

## 2. Cấu trúc project

```text
vins_fusion_d435i_local/
├── bags/          Calibration, rosbag và config theo từng lần đo
├── catkin_ws/     ROS workspace của VINS-Fusion
├── docker/        Môi trường Docker
├── output/        CSV, pose graph và biểu đồ kết quả
├── rs_ros_ws/     ROS workspace của RealSense driver
├── scripts/       Script kiểm tra và vẽ dữ liệu
└── third_party/   Thư viện ngoài ROS, hiện có librealsense
```

### `bags`

`bags/realsense_d435i_kalibr_183222/` là bộ calibration đang dùng:

| File | Chức năng |
|---|---|
| `rs_camera.launch` | Bật stereo IR và IMU của D435i |
| `realsense_stereo_imu_config.yaml` | Config chính cho VINS |
| `left.yaml`, `right.yaml` | Intrinsic và distortion hai camera |
| `*-camchain-imucam.yaml` | Kết quả extrinsic camera-IMU từ Kalibr |
| `*-imu.yaml` | Thông số noise IMU |
| `*-report-*.pdf` | Báo cáo chất lượng calibration |

Các đường dẫn `left.yaml` và `right.yaml` là tương đối với file config VINS. Vì ba file nằm cùng thư mục nên VINS đọc đúng calibration mới.

### `catkin_ws`

Workspace chứa source VINS-Fusion:

```text
catkin_ws/src/VINS-Fusion/    Source và config gốc
catkin_ws/build/              File build trung gian
catkin_ws/devel/              Executable, library, setup.bash
catkin_ws/logs/               Log build
```

Chỉ sửa source trong `src`. Không sửa trực tiếp `build` hoặc `devel`.

### `rs_ros_ws`

Workspace chứa `realsense-ros` branch ROS1 legacy:

```text
rs_ros_ws/src/realsense-ros/  Source driver
rs_ros_ws/devel/              Nodelet và setup.bash sau khi build
rs_ros_ws/build/              File build trung gian
rs_ros_ws/logs/               Log build
```

Driver này link với `librealsense2.so` được cài tại `/opt/librealsense` trong container.

### `third_party`

`third_party/librealsense/` là Intel RealSense SDK v2.56.5. SDK đọc frame và IMU trực tiếp từ USB. Nó không phải package ROS.

### `docker`

`docker/Dockerfile` tạo image ROS Noetic và cài công cụ build:

- OpenCV, Eigen, Ceres và SuiteSparse cho VINS.
- libusb và libudev cho RealSense.
- catkin tools và các package ROS cần thiết.
- RViz và công cụ kiểm tra topic.

Dockerfile không copy source vào image và không tự build ba workspace. Toàn bộ project được mount từ host vào `/work`, vì vậy source và output vẫn nằm trên Jetson sau khi container dừng.

Không cài `ros-noetic-librealsense2` hoặc `ros-noetic-realsense2-camera` từ apt. Project dùng bản build source để tránh xung đột phiên bản và lỗi IMU.

Ý nghĩa các phần trong Dockerfile:

| Phần | Chức năng |
|---|---|
| `FROM ros:noetic-perception` | Image nền Ubuntu 20.04 và ROS Noetic |
| `ENV DEBIAN_FRONTEND=noninteractive` | Cài package không hỏi tương tác |
| `SHELL ["/bin/bash", "-c"]` | Dùng Bash cho các lệnh build |
| `apt-get install` | Cài compiler, ROS, Ceres, OpenCV và USB dependency |
| `rosdep init` | Khởi tạo công cụ cài dependency ROS |
| `mkdir /opt/...` | Chuẩn bị nơi cài SDK và workspace |
| Ghi vào `.bashrc` | Nạp ROS và biến môi trường khi mở shell |
| `WORKDIR /work` | Đặt project mount làm thư mục làm việc |
| `CMD ["/bin/bash"]` | Mở Bash khi container khởi động |

### `output`

Chứa dữ liệu sinh ra khi VINS chạy:

| File | Nội dung |
|---|---|
| `vio.csv` | Timestamp, position, quaternion và velocity |
| `odometry_xyz.csv` | Timestamp và position `x,y,z` |
| `odometry_xy_plot.png` | Quỹ đạo trên mặt phẳng X-Y |
| `extrinsic_parameter.csv` | Extrinsic do VINS ghi khi cho phép estimate |
| `pose_graph/` | Dữ liệu loop closure |

### `scripts`

`scripts/plot_odometry_xyz.py` vẽ position theo thời gian và quỹ đạo X-Y.

## 3. Phiên bản đang dùng

```text
Host:                  Jetson Orin Nano, Ubuntu 22.04
Container:             ROS Noetic
librealsense:          v2.56.5
RealSense backend:     FORCE_RSUSB_BACKEND=ON
realsense-ros:         ros1-legacy
realsense2_camera:     2.3.2
Camera:                Intel RealSense D435i
```

Kiểm tra phiên bản:

```bash
rs-enumerate-devices --version
rosversion realsense2_camera
```

Hai số version khác nhau là bình thường: một số thuộc SDK, một số thuộc ROS wrapper.

## 4. Repository gốc và tài liệu tham khảo

Project được xây dựng từ ba nguồn chính:

| Repository | Vai trò trong project |
|---|---|
| [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) | Calib intrinsic, stereo extrinsic, camera-IMU extrinsic và time offset |
| [engcang/VINS-application - Intel-D435i](https://github.com/engcang/VINS-application/tree/Intel-D435i) | Tham khảo cách cấu hình và chạy VINS với RealSense D435i |
| [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | Source thuật toán VINS-Fusion gốc |

Kalibr được dùng để tạo bộ calibration local trong `bags/realsense_d435i_kalibr_183222/`. Các file `left.yaml`, `right.yaml`, extrinsic và time offset hiện tại lấy từ lần calibration này.

Repository Engcang chỉ được dùng để đối chiếu cách triển khai D435i. Config Engcang không còn được dùng trong project.

Source VINS-Fusion trong `catkin_ws/src/VINS-Fusion/` bắt đầu từ repository HKUST và đã được chỉnh sửa để phù hợp pipeline D435i hiện tại.

## 5. Build Docker image

Chạy trên host Jetson:

```bash
cd ~/vins_fusion_d435i_local

docker build \
  -t vins-fusion-d435i-local:noetic \
  -f docker/Dockerfile .
```

Image là môi trường chạy. Container là một phiên chạy của image.

## 6. Tạo và mở container

Tạo container lần đầu:

```bash
cd ~/vins_fusion_d435i_local

docker run -it \
  --name vins_d435i_local \
  --net=host \
  --ipc=host \
  --privileged \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  -v "$PWD":/work \
  -w /work \
  vins-fusion-d435i-local:noetic \
  bash
```

Ý nghĩa các option chính:

| Option | Ý nghĩa |
|---|---|
| `--net=host` | Container dùng chung mạng với ROS master |
| `--ipc=host` | Dùng chung shared memory |
| `--privileged` | Cho phép truy cập thiết bị USB |
| `-v /dev:/dev` | Đưa thiết bị camera vào container |
| `-v "$PWD":/work` | Mount project host vào `/work` |

Mở lại container:

```bash
docker start -ai vins_d435i_local
```

Mở thêm terminal trong container:

```bash
docker exec -it vins_d435i_local bash
```

Kiểm tra container đang chạy:

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

Nếu báo trùng tên container, không chạy `docker run` lần nữa. Dùng `docker start -ai vins_d435i_local`.

## 7. Build source trong container

Các bước này chỉ cần chạy khi cài lần đầu hoặc source thay đổi.

Repository chỉ lưu source, config và công cụ build. Các thư mục sau không được push vì có thể tạo lại:

```text
catkin_ws/build/       catkin_ws/devel/       catkin_ws/logs/
rs_ros_ws/build/       rs_ros_ws/devel/       rs_ros_ws/logs/
third_party/librealsense/build/
output/
```

Sau khi clone repository trên máy mới, thực hiện theo thứ tự:

```text
1. Build Docker image
2. Tạo container và mount repository vào /work
3. Build librealsense
4. Build RealSense ROS driver
5. Build VINS-Fusion
6. Chạy camera rồi chạy VINS
```

`build`, `devel` và `logs` sẽ tự xuất hiện sau khi build. `output` sẽ xuất hiện khi chạy VINS hoặc có thể tạo bằng `mkdir -p`.

### 6.1 Build librealsense

```bash
cd /work/third_party/librealsense
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=ON \
  -DBUILD_WITH_OPENGL=OFF \
  -DBUILD_WITH_CUDA=OFF

make -j2
make install
ldconfig
```

### 6.2 Build RealSense ROS driver

```bash
source /opt/ros/noetic/setup.bash
export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/opt/librealsense/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/librealsense:/opt/ros/noetic:$CMAKE_PREFIX_PATH

cd /work/rs_ros_ws
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build -j2
```

### 6.3 Build VINS-Fusion

```bash
source /opt/ros/noetic/setup.bash

cd /work/catkin_ws
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build -j2
```

## 8. Chạy camera và VINS

Chỉ cần ba terminal. `roslaunch` ở Terminal 1 sẽ tự bật ROS master nếu chưa có.

### Terminal 1: Camera D435i

```bash
docker exec -it vins_d435i_local bash
```

Trong container:

```bash
source /opt/ros/noetic/setup.bash
source /work/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/work/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

rospack find realsense2_camera

roslaunch /work/bags/realsense_d435i_kalibr_183222/rs_camera.launch
```

Launch này bật:

```text
Stereo IR:       640x480, 30 Hz
Gyroscope:       400 Hz
Accelerometer:   100 Hz
IMU merge:       linear_interpolation
Depth/color:     tắt
Emitter:         tắt bằng d435i_emitter_off.json
```

### Terminal 2: VINS-Fusion

Chờ camera publish đủ ba topic rồi chạy:

```bash
docker exec -it vins_d435i_local bash
```

Trong container:

```bash
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash

mkdir -p /work/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /work/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
```

Config này dùng calibration Kalibr local:

- Hai camera IR và một IMU.
- Extrinsic camera-IMU cố định: `estimate_extrinsic: 0`.
- Time offset cố định: `estimate_td: 0`.
- Kết quả lưu vào `/work/output/kalibr_183222/`.

### Terminal 3: Kiểm tra

```bash
docker exec -it vins_d435i_local bash
```

Trong container:

```bash
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash

rostopic hz /camera/infra1/image_rect_raw
rostopic hz /camera/infra2/image_rect_raw
rostopic hz /camera/imu
rostopic hz /vins_estimator/odometry
```

Xem đồng thời position và orientation:

```bash
rostopic echo /vins_estimator/odometry/pose/pose
```

Xem riêng position `x, y, z`:

```bash
rostopic echo /vins_estimator/odometry/pose/pose/position
```

Xem riêng orientation quaternion `x, y, z, w`:

```bash
rostopic echo /vins_estimator/odometry/pose/pose/orientation
```

## 9. Gửi odometry sang PX4

Topic VINS hiện dùng:

```text
World: x phải, y tiến, z lên          (ENU: East, North, Up)
Body:  x phải, y xuống, z tiến         (RDF: Right, Down, Forward)
```

PX4 không nhận trực tiếp quy ước body RFU. Node `vins_px4_bridge` thực hiện:

```text
Body RDF -> ROS FLU (Forward, Left, Up)
Quaternion, velocity và covariance -> đổi cùng hệ trục
World ENU -> giữ nguyên
Output -> /mavros/odometry/out
```

MAVROS tự đổi ENU/FLU sang NED/FRD của PX4. Không tự đổi `x,y,z` sang NED lần nữa.

### 9.1 Kết nối MAVROS với PX4

Sau khi sửa Dockerfile, build lại image và tạo lại container. Trên host, kiểm tra
tên cổng serial trước khi chạy MAVROS:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
```

- Cổng USB native của flight controller thường là `/dev/ttyACM0`.
- Adapter USB-UART thường là `/dev/ttyUSB0`.
- Ưu tiên `/dev/serial/by-id/...` vì tên này không đổi khi reboot hoặc cắm lại.

Kết nối bằng cổng USB native của PX4:

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

Nếu dùng UART, thay bằng thiết bị thực tế, ví dụ:

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

CP2102 đang dùng trong project có đường dẫn ổn định:

```bash
roslaunch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600
```

Nếu không có cả `/dev/ttyACM*` và `/dev/ttyUSB*`, kiểm tra `lsusb -t`. Thiết bị
phải có driver như `cdc_acm`, `cp210x`, `ftdi_sio` hoặc `ch341`; trạng thái
`Driver=usbfs` nghĩa là chưa có driver serial nên MAVROS chưa thể mở cổng.

Container được tạo với `--privileged` và bind `/dev`, vì vậy cổng xuất hiện trên
host cũng phải xuất hiện bên trong container. Nếu dùng UDP, thay `fcu_url` theo
kết nối thực tế. Kiểm tra MAVROS:

```bash
rostopic echo -n1 /mavros/state
```

Trường `connected` phải là `true`.

### 9.2 Chạy bridge VINS-PX4

Mở terminal khác trong cùng container:

```bash
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash

roslaunch vins vins_px4_bridge.launch
```

Kiểm tra dữ liệu gửi vào MAVROS:

```bash
rostopic hz /mavros/odometry/out
rostopic echo -n1 /mavros/odometry/out
```

Trong MAVLink Console của QGroundControl:

```text
listener vehicle_visual_odometry
```

### 9.3 Cấu hình EKF2 trên PX4

Profile của project dùng vision cho position X/Y và yaw, rangefinder cho Z:

| Tham số | Giá trị | Ý nghĩa |
|---|---:|---|
| `EKF2_EV_CTRL` | `9` | Vision horizontal position (bit 0) và yaw (bit 3) |
| `EKF2_HGT_REF` | `2` | Rangefinder là height reference |
| `EKF2_RNG_CTRL` | `2` | Luôn fuse rangefinder height |
| `EKF2_EV_NOISE_MD` | `0` | Dùng covariance từ message odometry |
| `EKF2_EV_DELAY` | `0` rồi tune | Độ trễ vision, đơn vị ms |
| `EKF2_EV_POS_X/Y/Z` | Theo vị trí lắp | Offset camera so với IMU PX4, hệ body FRD |

Thiết lập qua MAVROS:

```bash
rosrun mavros mavparam set EKF2_EV_CTRL 9
rosrun mavros mavparam set EKF2_HGT_REF 2
rosrun mavros mavparam set EKF2_RNG_CTRL 2
rosrun mavros mavparam set EKF2_EV_NOISE_MD 0
```

Không dùng `EKF2_EV_CTRL=7` cho profile này vì nó bật vertical vision position
và 3D vision velocity. PX4 1.14 không có bit riêng cho horizontal vision
velocity; `EKF2_EV_CTRL=13` sẽ thêm toàn bộ velocity X/Y/Z và vì vậy VINS vẫn
ảnh hưởng đến chuyển động theo Z.

Reboot PX4 sau khi đổi tham số. Với firmware cũ, tên tham số có thể khác.

Trước khi arm, đặt thiết bị đứng yên và kiểm tra:

1. Di chuyển sang phải: East tăng.
2. Di chuyển về trước: North tăng.
3. Di chuyển lên: altitude tăng, Down của PX4 giảm.
4. Quay yaw phải/trái: heading đổi đúng chiều.
5. `vehicle_visual_odometry` không nhảy pose hoặc timestamp.

Với profile này, sau khi reboot cần thấy `cs_ev_pos`, `cs_ev_yaw` và
`cs_rng_hgt` bật; `cs_ev_hgt` và `cs_ev_vel` phải tắt trong
`estimator_status_flags`.

## 10. Lưu và vẽ odometry

Lưu timestamp và position `x,y,z`:

```bash
rostopic echo -p \
  /vins_estimator/odometry/pose/pose/position \
  > /work/output/kalibr_183222/odometry_xyz.csv
```

Nhấn `Ctrl+C` để dừng ghi.

Thoát ra host Jetson rồi vẽ position theo thời gian và quỹ đạo X-Y:

```bash
cd ~/vins_fusion_d435i_local
MPLCONFIGDIR=/tmp/matplotlib-config \
python3 scripts/plot_odometry_xyz.py \
  output/kalibr_183222/odometry_xyz.csv
```

Kết quả:

```text
output/kalibr_183222/odometry_xyz_plot.png
output/kalibr_183222/odometry_xy_plot.png
```

`vio.csv` được VINS tạo tự động khi node khởi động. File này chứa:

```text
timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz
```

## 11. Ghi rosbag

Ghi dữ liệu camera và IMU:

```bash
rosbag record -O /work/bags/d435i_stereo_imu.bag \
  /camera/infra1/image_rect_raw \
  /camera/infra2/image_rect_raw \
  /camera/imu
```

Ghi thêm kết quả VINS:

```bash
rosbag record -O /work/bags/vins_result.bag \
  /camera/infra1/image_rect_raw \
  /camera/infra2/image_rect_raw \
  /camera/imu \
  /vins_estimator/odometry \
  /vins_estimator/path
```

## 12. Source workspace đúng cách

Hai workspace được build độc lập. Mỗi terminal chỉ source workspace nó cần:

Camera:

```bash
source /opt/ros/noetic/setup.bash
source /work/rs_ros_ws/devel/setup.bash
```

VINS:

```bash
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash
```

Không cần source cả hai trong cùng terminal để chạy pipeline. Các node trao đổi qua ROS master.

Kiểm tra package:

```bash
rospack find realsense2_camera
rospack find vins
```

Nếu package không được tìm thấy, source lại đúng workspace rồi chạy `rospack profile`.

## 13. Lỗi thường gặp

### `Resource not found: realsense2_camera`

```bash
source /opt/ros/noetic/setup.bash
source /work/rs_ros_ws/devel/setup.bash
rospack profile
rospack find realsense2_camera
```

### Không tìm thấy `librealsense2_camera.so`

```bash
export LD_LIBRARY_PATH=/work/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH
ls -l /work/rs_ros_ws/devel/lib/librealsense2_camera.so
ldconfig -p | grep librealsense
```

### Camera không xuất hiện trong container

```bash
lsusb
rs-enumerate-devices
```

Container phải có `--privileged`, `/dev` và `/run/udev`.

### Odometry trôi mạnh khi bật IMU

Kiểm tra theo thứ tự:

1. Đúng topic và tần số camera/IMU.
2. Camera không rơi frame và không bị USB reset.
3. Dùng đúng config `kalibr_183222`.
4. Extrinsic và time offset đang cố định.
5. Camera đứng yên vài giây trước khi bắt đầu chuyển động.
6. Cảnh có đủ texture và không bị motion blur.

Tham số `freq` trong YAML hiện không giới hạn tần số xử lý vì source này không đọc giá trị đó.

## 14. Dừng hệ thống

Nhấn `Ctrl+C` theo thứ tự:

1. Dừng VINS.
2. Dừng camera.
3. Thoát các terminal container.

Container vẫn được giữ để chạy lại bằng:

```bash
docker start -ai vins_d435i_local
```




# lệnh chạy 
cd ~/vins_fusion_d435i_local
docker start vins_d435i_local


docker exec -it vins_d435i_local bash

# bật cam 

source /opt/ros/noetic/setup.bash
source /work/rs_ros_ws/devel/setup.bash

export PATH=/opt/librealsense/bin:$PATH
export LD_LIBRARY_PATH=/work/rs_ros_ws/devel/lib:/opt/librealsense/lib:$LD_LIBRARY_PATH

roslaunch /work/bags/realsense_d435i_kalibr_183222/rs_camera.launch

# bật vins 

source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash

mkdir -p /work/output/kalibr_183222/pose_graph

rosrun vins vins_node \
  /work/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml

# Kết nối MAVROS với PX4

source /opt/ros/noetic/setup.bash

roslaunch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600


# Gửi VINS Odometry sang PX4  
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash

roslaunch vins vins_px4_bridge.launch \
  input_topic:=/vins_estimator/odometry \
  output_topic:=/mavros/odometry/out
