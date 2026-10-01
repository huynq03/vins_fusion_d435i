#!/usr/bin/env bash
# Start the D435i + VINS-Fusion pipeline in a tmux session.
# On the host every pane enters the container through `docker exec`; inside the
# container the panes run the commands directly.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: ./scripts/run.sh MODE [options]

Modes:
  vins     camera + VINS-Fusion
  px4      camera + VINS-Fusion + MAVROS + odometry bridge to PX4
  record   camera (RGB + IR + IMU) + VINS-Fusion + rosbag record
  stop     stop all nodes (Ctrl+C semantics, bag is finalized) and close tmux

Options:
  --fcu URL     MAVROS FCU URL (default: /dev/ttyTHS1:921600, env FCU_URL)
  --reset       reset the D435i once when the camera node starts
  --dry-run     print the pane commands only

tmux: Ctrl+b then arrow = switch pane, Ctrl+b d = detach, `tmux a -t vins` = re-attach.
EOF
}

CONTAINER="vins_d435i_compose"
SESSION="vins"
WS="/home/air/vins_fusion_d435i_local"
CALIB="$WS/bags/realsense_d435i_kalibr_183222"
FCU_URL="${FCU_URL:-/dev/ttyTHS1:921600}"
RESET=false
DRY_RUN=false

MODE="${1:-}"
[[ -n "$MODE" ]] || { usage >&2; exit 2; }
shift
while (($#)); do
    case "$1" in
        --fcu) FCU_URL="$2"; shift 2 ;;
        --reset) RESET=true; shift ;;
        --dry-run) DRY_RUN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

# Run a command in the container, whether this script runs on the host or inside it.
if [[ -f /.dockerenv ]]; then in_container() { "$@"; }; else in_container() { docker exec "$CONTAINER" "$@"; }; fi

ROS="source /opt/ros/noetic/setup.bash"
# roslaunch starts a master when none is running; the other panes wait for it
# so that only one master is ever created.
WAIT_MASTER="until rostopic list >/dev/null 2>&1; do sleep 0.5; done"
# The D435i IMU sometimes stays silent after (re)plugging; tell the user instead of hanging.
WAIT_CAMERA="$WAIT_MASTER
echo 'Waiting for IR images and IMU data...'
for t in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu; do
  timeout 30 rostopic echo -n1 \$t >/dev/null 2>&1 || { echo \"No data on \$t: restart: ./scripts/run.sh stop, then start again.\"; exit 1; }
done"
WAIT_ODOM="$WAIT_MASTER
until rostopic info /vins_estimator/odometry >/dev/null 2>&1; do sleep 0.5; done"

# rs_camera.launch already streams RGB 640x480@30 next to IR 640x480@30 and IMU.
CAMERA="$ROS; source $WS/rs_ros_ws/devel/setup.bash
roslaunch $CALIB/rs_camera.launch initial_reset:=$RESET"

VINS="$ROS; source $WS/catkin_ws/devel/setup.bash
$WAIT_CAMERA
mkdir -p $WS/output/kalibr_183222/pose_graph
rosrun vins vins_node $CALIB/realsense_stereo_imu_config.yaml"

MAVROS="$ROS
$WAIT_MASTER
roslaunch mavros px4.launch fcu_url:=$FCU_URL"

BRIDGE="$ROS; source $WS/catkin_ws/devel/setup.bash
$WAIT_ODOM
roslaunch vins vins_px4_bridge.launch input_topic:=/vins_estimator/odometry \
  output_topic:=/mavros/odometry/out parent_frame_id:=odom child_frame_id:=base_link"

RECORD="$ROS
$WAIT_ODOM
mkdir -p $WS/bags/recordings
rosbag record --lz4 --buffsize=512 -O $WS/bags/recordings/d435i_rgb_vins_\$(date +%Y%m%d_%H%M%S).bag \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/infra1/image_rect_raw /camera/infra1/camera_info \
  /camera/infra2/image_rect_raw /camera/infra2/camera_info \
  /camera/imu /vins_estimator/odometry /vins_estimator/path"

case "$MODE" in
    vins) PANES=(CAMERA VINS) ;;
    px4) PANES=(CAMERA VINS MAVROS BRIDGE) ;;
    record) PANES=(CAMERA VINS RECORD) ;;
    stop)
        # SIGINT lets rosbag write its index and roslaunch shut nodes down cleanly.
        in_container pkill -INT -f 'rosbag/record|record_odom_csv.py|vins_node|roslaunch' || true
        sleep 5
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        # From the host, closing tmux only ends the `docker exec` clients; the pane shells
        # keep running in the container and would start nodes again on the next master.
        in_container pkill -KILL -f '^bash -c source /opt/ros/noetic/setup.bash' || true
        in_container pkill -INT -f 'rosmaster|roscore' || true
        echo "Stopped."
        exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

if [[ "$DRY_RUN" == true ]]; then
    for p in "${PANES[@]}"; do printf '\n[%s]\n%s\n' "$p" "${!p}"; done
    exit 0
fi

command -v tmux >/dev/null || { echo "tmux is not installed: sudo apt install tmux" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists. Attach: tmux a -t $SESSION   Stop: $0 stop" >&2
    exit 1
fi
if [[ ! -f /.dockerenv && "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != true ]]; then
    docker compose -f "$(dirname "$0")/../compose.yaml" up -d
fi

# `exec bash` keeps each pane open after its node exits, so the log stays visible.
pane_cmd() {
    [[ -f /.dockerenv ]] || printf 'docker exec -it %q ' "$CONTAINER"
    printf 'bash -c %q' "${!1}
exec bash"
}

tmux new-session -d -s "$SESSION" -n "$MODE" "$(pane_cmd "${PANES[0]}")"
tmux set-option -w -t "$SESSION" remain-on-exit on  # keep failed panes and their errors
for p in "${PANES[@]:1}"; do
    tmux split-window -t "$SESSION" "$(pane_cmd "$p")"
    tmux select-layout -t "$SESSION" tiled
done
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" mouse on >/dev/null
i=0
for p in "${PANES[@]}"; do tmux select-pane -t "$SESSION:0.$i" -T "$p"; i=$((i + 1)); done

[[ -n "${TMUX:-}" ]] && tmux switch-client -t "$SESSION" || tmux attach -t "$SESSION"
