# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Visual-inertial odometry for a drone: Intel RealSense D435i (stereo IR + IMU) → VINS-Fusion → MAVROS → PX4 on a Jetson Orin Nano (Ubuntu 22.04 host, ROS 1 Noetic inside Docker). `tutorial.md` (Vietnamese) is the full operator guide; `command.md` is the day-to-day command cheat sheet — keep both in sync when commands change.

## Host / container layout

- Everything ROS runs in the compose container `vins_d435i_compose` (`compose.yaml`, `Dockerfile`), user `air`, host network, privileged, `/dev` mounted.
- The repo is bind-mounted at `/home/air/vins_fusion_d435i_local` inside the container; `/work` is a symlink to the same path. Paths in scripts, launch files and the VINS YAML are container paths, not host paths.
- librealsense is installed to `/opt/librealsense` inside the container, which is bind-mounted from `third_party/librealsense/install/` so it survives container re-creation.
- `compose.yaml` mounts the host `/etc/localtime` and `/etc/timezone`, so date-stamped names (bags, odometry CSVs) use local time; a container created before that mount runs on UTC until re-created.
- The container user is in `dialout` (MAVROS serial). The host needs the RealSense udev rules (`third_party/librealsense/config/99-realsense-libusb.rules` in `/etc/udev/rules.d/`), otherwise the non-root container user gets `RS2_USB_STATUS_ACCESS`.
- The flight controller (KakuteH7, TELEM1 at 921600) is connected through a CP2102 USB-TTL adapter: MAVROS `fcu_url:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600` (the default in `scripts/run.sh`, normally `/dev/ttyUSB0`). Do not go back to the Jetson GPIO UART `/dev/ttyTHS1` at 921600: the FC then receives only about half of the Jetson → FC frames (half the vision odometry is lost, PX4 logs `Connection to mission computer lost`) although `rostopic hz /mavros/odometry/out` still shows 30 Hz.

## Three independently built layers

Order matters; each is built inside the container. Build/devel/logs dirs are git-ignored and contain absolute paths, so if the mount path or user changes, delete `build devel logs .catkin_tools` and rebuild.

1. `third_party/librealsense` (SDK v2.56.5, plain CMake, `FORCE_RSUSB_BACKEND=ON`):
   ```bash
   cd third_party/librealsense && mkdir -p build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/librealsense -DFORCE_RSUSB_BACKEND=ON \
     -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=ON -DBUILD_WITH_OPENGL=OFF -DBUILD_WITH_CUDA=OFF
   make -j4 && make install
   ```
2. `rs_ros_ws` — realsense-ros `ros1-legacy` (realsense2_camera 2.3.2), links against `/opt/librealsense`. Do not install `ros-noetic-librealsense2` / `ros-noetic-realsense2-camera` from apt (version conflict / IMU issues).
3. `catkin_ws` — modified HKUST VINS-Fusion (`vins`, `loop_fusion`, `global_fusion`, `camera_models`).

```bash
source /opt/ros/noetic/setup.bash
cd <ws> && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build -j3            # or: catkin build vins
```

`rs_ros_ws` and `catkin_ws` both extend `/opt/ros/noetic` directly (not each other): each terminal sources only the workspace it needs (camera → `rs_ros_ws/devel/setup.bash`, VINS/bridge → `catkin_ws/devel/setup.bash`). The container `.bashrc` sources `catkin_ws`, so `rospack find realsense2_camera` fails there until `rs_ros_ws` is sourced.

`node_control/` is a separate catkin workspace (PX4 position-setpoint node via MAVROS, see `node_control/README.md`).

There is no test suite or linter; verification is done by running the pipeline and checking topic rates.

## Running

`scripts/run.sh` is the single entry point; it opens a tmux session `vins` (works on the host via `docker exec` per pane, or directly inside the container):

```bash
./scripts/run.sh vins      # camera + VINS
./scripts/run.sh px4       # camera + VINS + MAVROS + odometry bridge  (--fcu URL, --reset)
./scripts/run.sh record    # camera + VINS + rosbag -> bags/recordings/
./scripts/run.sh stop      # always stop with this
./scripts/run.sh px4 --dry-run
```

`stop` must be used instead of just killing tmux: from the host, killing tmux only ends the `docker exec` clients and the pane shells keep running in the container (a leftover pane once started a second rosbag writing the same file). `stop` SIGINTs `rosbag/record`, `vins_node`, `roslaunch` (so bags get indexed), then kills the pane shells and the master.

Only the camera pane runs `roslaunch` without waiting (it creates the ROS master); every other pane waits for the master, the VINS pane additionally waits for real IR + IMU data, the bridge/record panes wait for `/vins_estimator/odometry`.

Other scripts: `replay_bag_vins.sh BAG` (sim-time replay into VINS, records odometry; re-execs itself in the container from the host), `record_odom.sh` (VINS raw, bridge output, PX4 `/mavros/local_position/odom` and the FC's raw GPS `/mavros/gpsstatus/gps1/raw` — a Nooploop UWB module feeding NMEA into the GPS port — to `NAME_<time>.csv` in `output/odom_logs/<time>/` via `record_odom_csv.py`; re-execs in the container passing the host `TZ`; it runs in its own tmux session `odom_rec`, separate from the pipeline session `vins` (`--no-tmux` keeps it in the calling terminal); at start it also tells the FC to log to its SD card with NSH `logger on` and stops it with `logger off`, writing the ULog path to `fc_ulog_<time>.txt` (`--no-fc-log` disables it); also stopped by `run.sh stop`, which SIGINTs the recorder before MAVROS so `logger off` still gets through), `px4_shell.py "CMD"...` (PX4 NSH commands through MAVROS `/mavlink/to` with MAVLink SERIAL_CONTROL, no pymavlink), `plot_odom_compare.py DIR` (overlays VINS raw, PX4 fused odometry and, when `gps_raw` has fixes, the raw GPS track fitted onto VINS by one rotation + translation), `plot_odometry.py BAG|CSV`, `bag_to_video.py raw|track|vins|live`, `record_webcam.sh` (external USB webcams, host-side ffmpeg). Python scripts run inside the container (need `rosbag`/`cv_bridge`; Python 3.8, OpenCV 4.2 — no 3.9+ syntax).

Health check: `/camera/imu` ~400 Hz, `/camera/infra1/image_rect_raw` ~30 Hz, `/vins_estimator/odometry` ~30 Hz, `/mavros/odometry/out` ~30 Hz, `/mavros/state` `connected: True`.

## Data flow and frames

- Camera config: `bags/realsense_d435i_kalibr_183222/rs_camera.launch` (IR 640x480@30, RGB 640x480@30, gyro 400 / accel 100 united by `linear_interpolation` into `/camera/imu`; loads the emitter-off preset `catkin_ws/src/VINS-Fusion/config/realsense_d435i/d435i_emitter_off.json` so the IR dot pattern does not pollute feature tracking).
- VINS config: `bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml` — stereo + IMU, extrinsics/time offset from the Kalibr run in the same directory (`left.yaml`, `right.yaml`, `*-camchain-imucam.yaml`, reports). `estimate_extrinsic: 0`; `output_path` is `/work/output/kalibr_183222/` (VINS writes `vio.csv`, pose_graph there).
- `vins_estimator/src/vinsPx4Bridge.cpp` (`vins_px4_bridge` node, `vins_px4_bridge.launch`) is project-specific: converts VINS body RDF (right-down-forward, D435i optical/IMU axes) to ROS FLU, keeps the VINS world frame (arbitrary initial yaw, local ENU-like), rotates velocity into body FLU per REP-147, publishes `nav_msgs/Odometry` on `/mavros/odometry/out`. MAVROS turns that into MAVLink `ODOMETRY` with `LOCAL_FRD`/`BODY_FRD` so EKF2 can align heading. Do not additionally convert to NED. PX4 must have vision fusion enabled (`EKF2_EV_CTRL`, see tutorial §9.3).

## Known quirks

- After replugging the D435i or re-creating the container, one stream (IMU or IR) sometimes stays silent while the others publish; restarting the camera launch fixes it. The camera must be on USB 3 (`rs-enumerate-devices | grep "Usb Type"` → 3.2).
- `vins_node` prints `Aborted (core dumped)` on Ctrl+C; that is normal.
- VINS drifts badly if initialized while static; move the camera during startup.
- `record` bags are ~25 MB/s (RGB + two IR + IMU).
- Port 22 to GitHub is blocked on this network; `~/.ssh/config` routes `github.com` through `ssh.github.com:443`.
- Commits are authored as `huynq03` (repo-local git config) with no co-author trailer.
