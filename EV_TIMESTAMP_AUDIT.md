# Audit timestamp và external-vision delay

Thời điểm đo: 2026-07-28 (Asia/Ho_Chi_Minh)  
Pipeline: ROS1 Noetic, VINS-Fusion, MAVROS 1.20.1, PX4 báo version 1.14.3  
Phạm vi: chỉ đọc/đo; không đổi parameter PX4 và không sửa code vận hành

## Kết luận nhanh

Nguyên nhân nổi bật trong lần chạy được đo không phải một delay cố định cần
nhập vào `EKF2_EV_DELAY`, mà là backlog tăng dần bên trong VINS:

- Camera phát measurement stamp sạch ở 29.996 Hz, age khoảng 9–10 ms.
- Stamp odometry VINS vẫn tiến ở 29.990 Hz, nhưng message chỉ được publish ở
  26.778 Hz.
- Odometry VINS đã già trung bình 15.408 s và age tăng khoảng 107 ms mỗi giây.
- Bridge giữ nguyên stamp; signed arrival skew quan sát bởi diagnostic trung
  bình khoảng +0.425 ms (gồm TCPROS/callback scheduling, không phải timing nội
  tại đã instrument).
- MAVROS–PX4 timesync có RTT trung bình khoảng 1.484 ms và residual offset
  trung bình khoảng 0.063 ms.
- `EKF2_EV_DELAY` đang là `0 ms` trong RAM.

Vì delay VINS đang lớn hơn giới hạn parameter 300 ms khoảng 50 lần và còn tăng,
không có giá trị `EKF2_EV_DELAY` nào sửa được lần chạy này. Giá trị an toàn,
đúng dấu và có cơ sở hiện tại là **giữ `0 ms`**, chưa set gì. Cần đưa VINS về
realtime trước rồi mới ghi rosbag + ULog và tune residual delay.

Điều này giải thích đúng triệu chứng: khi camera đứng yên, một pose cũ 15 s vẫn
gần pose hiện tại nên innovation có thể qua gate; ngay khi nhấc/di chuyển, pose
cũ lệch mạnh khỏi state hiện tại nên horizontal position dễ bị reject.

## A. Timestamp VINS đúng hay sai?

### Chuỗi source

1. RealSense lấy frame timestamp và chuyển thành ROS time:
   [`base_realsense_node.cpp:1636`](rs_ros_ws/src/realsense-ros/realsense2_camera/src/base_realsense_node.cpp#L1636)
   và
   [`base_realsense_node.cpp:1647`](rs_ros_ws/src/realsense-ros/realsense2_camera/src/base_realsense_node.cpp#L1647).
2. Timestamp đó được truyền vào `publishFrame()` tại dòng 1721–1729 và gán
   `img->header.stamp = t` tại
   [`base_realsense_node.cpp:2433`](rs_ros_ws/src/realsense-ros/realsense2_camera/src/base_realsense_node.cpp#L2433).
3. Stereo sync của VINS so ảnh trái/phải với tolerance 3 ms, rồi chọn stamp ảnh
   trái tại
   [`rosNodeTest.cpp:83`](catkin_ws/src/VINS-Fusion/vins_estimator/src/rosNodeTest.cpp#L83)
   và dòng 98–109.
4. `Estimator::inputImage()` giữ cùng `t` trong `featureBuf` tại
   [`estimator.cpp:160`](catkin_ws/src/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp#L160)
   và dòng 187–189.
5. Processing thread lấy `feature.first`, chạy estimator, rồi tạo:

   ```cpp
   header.stamp = ros::Time(feature.first);
   ```

   tại
   [`estimator.cpp:274`](catkin_ws/src/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp#L274)
   và dòng 283–331.
6. `pubOdometry()` copy header và publish tại
   [`visualization.cpp:153`](catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.cpp#L153)
   và dòng 158–180.

Do node dùng private `NodeHandle("~")`, publisher relative `"odometry"` tại
[`visualization.cpp:66`](catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.cpp#L66)
resolve thành `/vins_estimator/odometry`.

**Kết luận A:** `header.stamp` là measurement timestamp của ảnh infrared trái,
không phải `ros::Time::now()` lúc publish và không phải stamp IMU cuối.

### Clock domain RealSense thực tế

Một sample `/camera/infra1/metadata` trong lúc audit:

```text
clock_domain:    global_time
header stamp:    1785208576.413173437
frame_timestamp: 1785208576413.173340 ms
time_of_arrival: 1785208576421 ms
```

Header khớp frame timestamp; time-of-arrival muộn hơn khoảng 7.83 ms. Vì domain
đang là `global_time`, driver đi vào nhánh dùng trực tiếp
`frame.get_timestamp()/1000` tại dòng 1831–1834. Nhánh `HARDWARE_CLOCK` mới neo
clock vào `ros::Time::now()` tại dòng 1805–1829; nhánh đó không hoạt động trong
snapshot này.

### Offset camera–IMU trong VINS

Config đang chạy có:

```yaml
estimate_td: 0
td: 0.0023707443799577937
```

Bằng chứng:
[`realsense_stereo_imu_config.yaml:67`](bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml#L67).

VINS tích phân IMU tới `feature.first + td` tại `estimator.cpp:284`, nhưng
message vẫn mang raw image stamp. Do đó state được tính trên timeline muộn hơn
header khoảng 2.371 ms. Sai khác nhỏ này không giải thích được reject khi di
chuyển và một `EKF2_EV_DELAY` dương còn dịch sample theo hướng sớm hơn.

### Có node nào ghi đè stamp không?

- Bridge hiện tại copy đúng stamp tại
  [`vinsPx4Bridge.cpp:70`](catkin_ws/src/VINS-Fusion/vins_estimator/src/vinsPx4Bridge.cpp#L70).
- `globalOptNode` và `pose_graph_node` có subscribe VINS nhưng publish topic
  khác, không remap ngược vào input/bridge.
- Runtime lúc đo xác nhận sole publisher thực của input là `/vins_estimator`,
  sole publisher output là `/vins_px4_bridge`.

Không thấy bước ghi đè timestamp trên đường đang chạy.

## B. Latency VINS processing

### Đo 200 sample đồng thời

| Đại lượng | VINS input bridge | Bridge output |
|---|---:|---:|
| Age min | 15028.588 ms | 15028.920 ms |
| Age mean | 15408.479 ms | 15408.904 ms |
| Age median | 15384.732 ms | 15384.939 ms |
| Age p95 | 15794.614 ms | 15795.195 ms |
| Age max | 15824.418 ms | 15824.124 ms |
| Stamp frequency | 29.990 Hz | 29.990 Hz |
| Arrival/publish frequency | 26.778 Hz | 26.781 Hz |
| Stamp backward/duplicate/abnormal | 0/0/0 | 0/0/0 |

Tuổi VINS tăng từ 15028.588 lên 15824.418 ms trong cửa sổ và tốc độ tăng được
fit là khoảng **107.091 ms/s**.

Điều này tự khớp về số học:

```text
26.778 processed frame/s ÷ 29.990 measurement frame/s = 0.893 s timeline/s
1.000 - 0.893 = 0.107 s backlog growth/s
```

### Baseline sensor

| Stream | Samples | Age min/mean/max | Arrival rate | Stamp rate |
|---|---:|---:|---:|---:|
| infra1 camera_info | 120 | 8.783/9.665/11.621 ms | 29.990 Hz | 29.996 Hz |
| infra2 camera_info | 120 | 8.812/9.456/14.969 ms | 29.991 Hz | 29.996 Hz |
| `/camera/imu` | 400 | 0.508/5.735/13.288 ms | 401.916 Hz | 400.003 Hz |

Hai camera có 120/120 stamp khớp chính xác. Vì sensor input còn realtime nhưng
VINS output già khoảng 15 s, latency nằm sau camera input, trong
tracking/optimizer/queue của VINS.

### Tài nguyên

Snapshot process:

```text
VINS process CPU:       ~218%
RealSense manager CPU:   17.8%
Bridge CPU:               1.0%
MAVROS CPU:               5.2%
Container CPU total:    240.79%
Container RAM:          1012 MiB / 7.441 GiB
Swap:                   0
```

Đây là dấu hiệu throughput CPU/processing, không phải thiếu RAM.

### Queue có thể tích backlog

| Điểm | Queue | Nhận xét |
|---|---:|---|
| RealSense image publisher | 1 | Ưu tiên bỏ frame cũ |
| VINS image subscriber | 100/camera | Khoảng 3.3 s ở 30 Hz |
| VINS IMU subscriber | 2000 | Khoảng 5 s ở 400 Hz |
| `img0_buf`/`img1_buf` | Không giới hạn | Có thể tăng liên tục |
| `featureBuf`, `accBuf`, `gyrBuf` | Không giới hạn | Có thể tăng liên tục |
| VINS odometry publisher | 1000 | Có thể giữ message cũ |
| Bridge subscriber/publisher | 10/10 | Tối đa khoảng 0.37 s ở rate đo |
| MAVROS odometry outbound subscriber | 1 | Giữ newest ở đầu MAVROS |

Source VINS subscriber:
[`rosNodeTest.cpp:252`](catkin_ws/src/VINS-Fusion/vins_estimator/src/rosNodeTest.cpp#L252).
Bridge queue:
[`vinsPx4Bridge.cpp:30`](catkin_ws/src/VINS-Fusion/vins_estimator/src/vinsPx4Bridge.cpp#L30).

## C. Latency bridge

Trong 220 exact-stamp pairs:

```text
bridge receipt - VINS receipt:
min    -1.381 ms
mean   +0.425 ms
median +0.368 ms
p95    +0.946 ms
max    +3.143 ms
```

Đây là **signed observer-arrival skew** giữa hai TCP connection độc lập, không
phải intrinsic callback processing time của bridge. Diagnostic có thể nhận
output trước bản input tương ứng nên có số âm; cả số âm và dương được giữ để
không che bias scheduling. Muốn tách đúng intrinsic bridge time phải
instrument `ros::SteadyTime` trong callback, việc này không làm ở audit
read-only.

Tất cả stamp được giữ nguyên. Rate VINS và bridge gần như bằng nhau; source
callback chỉ đổi đại lượng không gian và không thấy sustained drop tại bridge.
Các bằng chứng đó đủ loại bridge khỏi nguồn latency nhiều giây, nhưng không
nên diễn giải `+0.425 ms` là benchmark nội tại tuyệt đối.

**Kết luận C:** bridge không tạo ra latency nhiều giây; local handoff skew quan
sát được ở mức dưới 1 ms trung bình.

## D. Latency MAVROS/PX4 transport

### Hướng topic

Runtime và source MAVROS 1.20.1 cùng xác nhận:

- `/mavros/odometry/out`: ROS companion → MAVROS → FCU.
- `/mavros/odometry/in`: FCU → MAVROS → ROS.

Source chính thức:
[`mavros_extras/src/plugins/odom.cpp` 1.20.1](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp#L35-L68).
Plugin advertise `in`, subscribe `out`; outbound subscriber queue là 1.

### ROS stamp sang MAVLink/PX4

MAVROS gán trực tiếp:

```cpp
msg.time_usec = odom->header.stamp.toNSec() / 1e3;
```

tại
[`odom.cpp:263`](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp#L263).
Không có `ros::Time::now()` hay delay cộng thêm.

`frame_id` và `child_frame_id` chỉ chọn TF/spatial conversion và MAVLink
`frame_id`/`child_frame_id`; chúng không thay numeric timestamp.

PX4 v1.14.3 nhận MAVLink `ODOMETRY` và làm:

```cpp
odom.timestamp_sample = _mavlink_timesync.sync_stamp(odom_in.time_usec);
```

tại
[`mavlink_receiver.cpp:1251`](https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/src/modules/mavlink/mavlink_receiver.cpp#L1251).
Sau đó `odom.timestamp` là FCU receipt/processing time.

Vì vậy trên PX4:

```text
(vehicle_visual_odometry.timestamp
 - vehicle_visual_odometry.timestamp_sample) / 1000
```

là total sample age tại FCU, đơn vị ms, sau khi timesync đã hội tụ.

### Timesync đo được

60 samples `/mavros/timesync_status`:

```text
rate:                           10.000 Hz
header age min/mean/max:        0.383/0.534/0.800 ms
round-trip min/mean/max:        1.077/1.484/4.325 ms
observed-estimated offset
  min/mean/max:                -1.419/+0.063/+0.675 ms
```

Timesync source yêu cầu khoảng 500 accepted exchanges để hội tụ; trước hội tụ
`sync_stamp()` có thể dùng FCU arrival time. Do đó không tin delta startup ngay
sau khi reset, nhưng snapshot hiện tại đã ổn định.

**Khoảng trống D:** chưa có PX4 NSH output/ULog nên chưa đo trực tiếp
`vehicle_visual_odometry.timestamp - timestamp_sample`. Số liệu ROS cho thấy
phần lớn total age sẽ là backlog VINS, không phải serial/timesync.

## E. `EKF2_EV_DELAY` hiện tại và metadata

FCU trả qua hai đường read-only:

```text
rosrun mavros mavparam get EKF2_EV_DELAY
0
```

và `/mavros/param/get` trả `success: true`, real value `0.0`. Đây là giá trị RAM
đang chạy; chưa chứng minh giá trị persisted sau reboot.

FCU báo:

```text
flight_sw_version: 0x010E0300 -> 1.14.3
flight_custom_version: 4a0e65f233000000
```

Metadata upstream v1.14.3:

- unit: `ms`
- min: `0`
- max: `300`
- default: `0`
- reboot required: `true`

Bằng chứng:
[`ekf2_params.c:141-151`](https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/src/modules/ekf2/ekf2_params.c#L141-L151).

Logic dấu:

```cpp
time_us = evdata.time_us
        - ev_delay_ms * 1000
        - half_EKF_update_period;
```

Bằng chứng:
[`estimator_interface.cpp:387-396`](https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/src/modules/ekf2/EKF/estimator_interface.cpp#L387-L396).

Do đó:

- Parameter dương dịch EV sample **sớm hơn**.
- Nếu upstream stamp là publish/arrival time muộn hơn capture `L ms`, dùng
  parameter dương khoảng `L ms`.
- Nếu upstream stamp đã là capture/measurement time và timesync đúng, expected
  value là gần `0 ms`.
- Không có giá trị âm trên firmware này.

Custom hash `4a0e65f233` không resolve được trong upstream/local source. Kết luận
metadata dựa trên semver chính FCU báo và tag v1.14.3, chưa phải byte-for-byte
audit custom build. Cần `ver all` và matching vendor source để đóng khoảng
trống đó.

## F. Delay đề xuất

### Cho trạng thái hiện tại

**Giữ `EKF2_EV_DELAY=0 ms`; không set parameter.**

Lý do:

1. Header là measurement stamp đúng.
2. MAVROS chuyển nguyên stamp tới MAVLink.
3. Timesync ổn định.
4. Delay quan sát được là backlog tăng dần khoảng 15 s, không phải offset cố
   định.
5. Parameter chỉ cho 0–300 ms và giá trị dương sẽ làm sample còn sớm/cũ hơn.

Không đề xuất quét quanh 15 s hoặc ép về 300 ms.

### Sau khi VINS đã realtime

Chỉ tune khi:

- `message age` ổn định, không tăng theo thời gian;
- VINS publish rate xấp xỉ camera rate;
- đã có rosbag test và ULog cùng thời đoạn;
- delay tối ưu X/Y có cùng dấu, gần nhau và correlation đủ rõ;
- PX4 listener xác nhận sample age/timesync.

Nếu sau đó chứng minh stamp EV muộn thật sự `d ms`, quét:

```text
max(0, d - 30 ms) ... min(300, d + 30 ms), bước 10 ms
```

Nếu X/Y không thống nhất hoặc phase lag chỉ xuất hiện giữa VINS và fused PX4
velocity, giữ 0: phase lag đó có thể là EKF dynamics, competing aids hoặc
filtering, không phải timestamp bias.

## G. Lệnh PX4 shell cần chạy

Firmware 1.14.3 có đúng các topic/fields sau. Chạy trong QGroundControl MAVLink
Console hoặc NSH và gửi lại toàn bộ output:

```text
ver all
param show EKF2_EV_DELAY
param show EKF2_EV_CTRL
param show EKF2_EV_NOISE_MD
param show EKF2_EVP_NOISE
param show EKF2_EVP_GATE
param show EKF2_EV_QMIN
param show EKF2_EV_POS_X
param show EKF2_EV_POS_Y
param show EKF2_EV_POS_Z
param show EKF2_OF_CTRL

uorb top -1 vehicle_visual_odometry estimator_aid_src_ev_pos
uorb top -1 vehicle_local_position estimator_status_flags

listener vehicle_visual_odometry -n 10 -r 10
listener estimator_aid_src_ev_pos
listener estimator_aid_src_ev_pos -i 0 -n 10 -r 10
listener vehicle_local_position -n 10 -r 10
listener estimator_status_flags
listener estimator_status_flags -i 0 -n 10 -r 5
```

Nếu multi-EKF:

```text
listener estimator_selector_status
```

Sau đó thay `-i 0` bằng primary instance được báo. Nếu filter `uorb top` không
hiện topic, chạy:

```text
uorb top -a -1 estimator_aid_src estimator_status_flags
uorb top -a -1 vehicle_visual_odometry vehicle_local_position
```

Các field cần giữ:

- `vehicle_visual_odometry`: `timestamp`, `timestamp_sample`, `pose_frame`,
  `position`, `reset_counter`, `quality`.
- `estimator_aid_src_ev_pos`: `timestamp`, `timestamp_sample`,
  `time_last_fuse`, `innovation[0:2]`, `test_ratio[0:2]`,
  `fusion_enabled`, `innovation_rejected`, `fused`.
- `estimator_status_flags`: `cs_ev_pos`, `reject_hor_pos`, timestamp/sample.
- `vehicle_local_position`: timestamp/sample, `xy_valid`, `v_xy_valid`,
  `x`, `y`, `vx`, `vy`.

`test_ratio > 1` nghĩa là vượt innovation gate. `fused=true` chỉ khi sample đó
fuse thành công.

Để bật ULog cho bench test:

```text
logger status
logger on
```

Nếu module logger chưa chạy:

```text
logger start -e -t
```

Sau test chạy lại `logger status` và giữ file `.ulg` tương ứng.

## H. Các lỗi còn nghi ngờ

Ưu tiên theo bằng chứng hiện có:

1. **VINS processing backlog/throughput** — đã đo trực tiếp, là nguyên nhân
   chính cần xử lý trước.
2. **Queue VINS không giới hạn và publisher queue 1000** — khiến stale state
   tiếp tục được xuất thay vì drop-to-latest.
3. **Covariance cố định có thể quá tự tin khi chuyển động** —
   [`visualization.cpp:34-59`](catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.cpp#L34)
   hard-code position variance `0.01 m²` (sigma 0.1 m). Nếu
   `EKF2_EV_NOISE_MD=0`, PX4 dùng covariance message/lower bound; covariance
   quá nhỏ sẽ làm test ratio tăng và reject.
4. **Competing horizontal aids** — optical flow/GNSS/fake position hoặc origin
   alignment có thể xung đột khi di chuyển. Cần ULog flags và aid-source
   innovations, không suy từ đứng yên.
5. **Quality gate** — MAVROS ROS1 odometry plugin gửi quality mặc định 0; nếu
   `EKF2_EV_QMIN > 0`, fusion không start/continue đúng. Cần `param show`.
6. **Spatial frame/TF/extrinsic** — frame không thay timestamp nhưng có thể làm
   innovation sai dấu/scale khi chuyển động. Kiểm tra `pose_frame=2`,
   transform `odom_ned↔odom`, camera lever arm và movement X/Y.
7. **Timesync startup/reset** — chỉ tin timestamp_sample sau khi timesync hội
   tụ; snapshot hiện tại tốt nhưng startup cần loại khỏi test.
8. **VINS `td` vs output header** — mismatch 2.371 ms có thật nhưng nhỏ và dấu
   không phù hợp để giải thích reject lớn.

## Clock và topology

Tất cả node ROS lúc đo ở cùng container `vins_d435i_local`:

```text
rosmaster
camera RealSense nodelets
vins_node
vins_px4_bridge
mavros_node
```

Chúng cùng:

```text
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROS_DISTRO=noetic
ROS_IP unset
ROS_HOSTNAME unset
```

Các lệnh read-only và output chính:

```text
host$ date --iso-8601=ns
2026-07-28T10:10:31,438925690+07:00

host$ timedatectl status
Time zone: Asia/Ho_Chi_Minh (+07, +0700)
System clock synchronized: yes
NTP service: active
RTC in local TZ: no

host$ timedatectl timesync-status
Server: time.cloudflare.com
Stratum: 3
Root distance: 25.703 ms
Offset: +27.883 ms
Delay: 39.401 ms
Jitter: 9.515 ms

host$ chronyc tracking
chronyc: command not found

container$ date --iso-8601=ns
2026-07-28T03:11:48,040602329+00:00
```

Container chạy `--net=host`, `--ipc=host`, bind project host vào `/work`.
Host và container có cùng Linux time namespace:

```text
/proc/self/ns/time -> time:[4026531834]
timens_offsets monotonic/boottime -> 0
```

Host hiển thị Asia/Ho_Chi_Minh, container hiển thị UTC; đây chỉ là timezone.
`timedatectl` báo system clock synchronized và NTP active. `chronyc` không được
cài. Vì measurement producer, VINS, bridge và diagnostic dùng cùng kernel
clock, `now - header.stamp` đáng tin trong snapshot này.

ROS master/pipeline đã ngừng sau khi toàn bộ số liệu trên được thu; audit không
tự restart để tránh thay đổi trạng thái của người dùng.

## Công cụ đã thêm

### Diagnostic node

[`scripts/ev_timestamp_diagnostic.py`](scripts/ev_timestamp_diagnostic.py)
subscribe đồng thời input/output, in định kỳ:

- header stamp và `rospy.Time.now()`;
- age ms;
- stamp/arrival interval;
- effective frequency;
- min/mean/max age cửa sổ 200;
- backward/duplicate/forward-jump/zero/negative-age counts;
- exact-stamp paired signed observer-arrival skew.

Subscriber queue mặc định là 1 và TCP receive buffer là 1 MiB để diagnostic
không tự tích thêm nhiều giây backlog; cửa sổ thống kê vẫn là 200 sample.

Chạy trong container:

```bash
source /opt/ros/noetic/setup.bash
source /work/catkin_ws/devel/setup.bash
/work/scripts/ev_timestamp_diagnostic.py --window 200 --report-period 2
```

Node không sửa/republish timestamp.

### Script ghi test

[`scripts/record_ev_delay_test.sh`](scripts/record_ev_delay_test.sh) kiểm tra
bốn topic bắt buộc về tên, type và có sample live trong 5 s; tự thêm MAVROS
estimator/status/time topic đang có; theo dõi PID recorder trong mọi phase; ghi
rosbag và hướng dẫn chuỗi:

```text
đứng yên 5 s
+X, dừng 3 s
-X, dừng 3 s
+Y, dừng 3 s
-Y, dừng
```

Không xoay yaw. Chạy:

```bash
source /opt/ros/noetic/setup.bash
/work/scripts/record_ev_delay_test.sh
```

Script tạo thêm phase CSV và topic list cạnh bag.

### Phân tích offline

[`scripts/analyze_ev_delay.py`](scripts/analyze_ev_delay.py):

- đọc VINS position X/Y và PX4 local velocity;
- resample uniform 50 Hz;
- centered moving-average nhẹ, đạo hàm position;
- không nội suy qua source gap lớn hơn 200 ms;
- dùng đúng một fixed overlap cho mọi candidate shift;
- scan `-300...+300 ms`, bước `5 ms`;
- xuất RMSE/correlation X, Y, combined;
- xuất ba plot riêng X before/after, Y before/after, RMSE/shift;
- không dùng seaborn.

Quy ước:

```text
score(d) so VINS velocity(t) với PX4 velocity(t+d)
d > 0 => curve PX4 xuất hiện muộn hơn
```

Không map trực tiếp `d` sang parameter. Đặt
`E = EV_stamp - actual_capture` (dương khi stamp bị muộn) và `R` là response
lag của fused PX4 output, thì xấp xỉ:

```text
d = R - E
```

Chỉ nếu độc lập chứng minh `R ≈ 0` mới có `E ≈ -d` và candidate
`EKF2_EV_DELAY ≈ max(0, -d)`. Trong thực tế cần ULog innovations/timestamps vì
fused velocity không tách riêng được `R` và `E`.

Chạy:

```bash
source /opt/ros/noetic/setup.bash
python3 /work/scripts/analyze_ev_delay.py \
  /work/output/ev_delay/ev_delay_YYYYMMDD_HHMMSS.bag
```

Container hiện có `rosbag`, `rospy`, NumPy 1.17.4 nhưng thiếu matplotlib. Có
thể chạy numeric trước với `--no-plots`; để tạo PNG cần cung cấp
`python3-matplotlib` trong image/container.

### Kiểm chứng công cụ

- `python3 -m py_compile`: pass.
- `bash -n`: pass.
- `shellcheck`: pass.
- Synthetic rosbag có PX4 lag đúng +120 ms: script tìm lại
  `X=120`, `Y=120`, `combined=120 ms`.
- Regression endpoint transient với true delay 0: fixed-overlap scan vẫn trả
  `X=Y=combined=0 ms`.
- Trục Y không excitation: script không abort, báo correlation unavailable và
  RMSE curve ambiguous.
- Synthetic paired receipt delay 0.4 ms: diagnostic báo đúng 0.400 ms.
- Matplotlib plot smoke test tạo đủ ba PNG.

Không có rosbag/ULog thật trong workspace, nên chưa có delay tối ưu X/Y của
bài test chuyển động thật.
