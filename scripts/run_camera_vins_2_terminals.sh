#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER_NAME="${VINS_CONTAINER:-vins_d435i_local}"
CAMERA_LAUNCH="/home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch"
VINS_CONFIG="/home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml"
DRY_RUN=false
MODE="gui"

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_camera_vins_2_terminals.sh [options]

When run inside the Docker container, start the camera and VINS-Fusion in one
terminal. Ctrl+C stops both processes.

On a host desktop, open two host terminals, each using `docker exec`:
  1. RealSense D435i camera
  2. VINS-Fusion (waits for both IR images and IMU before starting)

Over SSH, use two SSH terminals and run `--camera` in the first one and
`--vins` in the second one. Both commands stay in the foreground, so Ctrl+C
stops the respective ROS node.

Options:
  --container NAME  Docker container name (default: vins_d435i_local).
  --camera          Run only the camera in this terminal (for SSH).
  --vins            Run only VINS-Fusion in this terminal (for SSH).
  --dry-run         Print the commands without opening terminals.
  -h, --help        Show this help.

Environment:
  VINS_CONTAINER    Docker container name.
EOF
}

while (($# > 0)); do
    case "$1" in
        --container)
            [[ $# -ge 2 ]] || { echo "Missing value after --container" >&2; exit 2; }
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --camera)
            MODE="camera"
            shift
            ;;
        --vins)
            MODE="vins"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

CAMERA_RUN_COMMAND="source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash
export PATH=/opt/librealsense/bin:\$PATH
export LD_LIBRARY_PATH=/home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:\$LD_LIBRARY_PATH
exec roslaunch ${CAMERA_LAUNCH}"

VINS_RUN_COMMAND="source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash
for attempt in {1..120}; do
  if rostopic info /camera/infra1/image_rect_raw >/dev/null 2>&1 &&
     rostopic info /camera/infra2/image_rect_raw >/dev/null 2>&1 &&
     rostopic info /camera/imu >/dev/null 2>&1; then
    break
  fi
  if [[ \$attempt -eq 120 ]]; then
    echo 'Camera topics were not ready after 60 seconds.' >&2
    exit 1
  fi
  sleep 0.5
done
echo 'Camera topics are ready; starting VINS-Fusion.'
mkdir -p /home/hann/vins_fusion_d435i_local/output/kalibr_183222/pose_graph
exec rosrun vins vins_node ${VINS_CONFIG}"

CAMERA_COMMAND="${CAMERA_RUN_COMMAND}
exec bash"
VINS_COMMAND="${VINS_RUN_COMMAND}
exec bash"

print_command() {
    local title="$1"
    local command="$2"
    printf '\n[%s]\n%s\n' "$title" "$command"
}

wait_for_camera_topics() {
    source /opt/ros/noetic/setup.bash
    for attempt in {1..120}; do
        if rostopic info /camera/infra1/image_rect_raw >/dev/null 2>&1 &&
           rostopic info /camera/infra2/image_rect_raw >/dev/null 2>&1 &&
           rostopic info /camera/imu >/dev/null 2>&1; then
            return 0
        fi
        if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
            echo 'Camera process exited before its topics became available.' >&2
            return 1
        fi
        sleep 0.5
    done

    echo 'Camera topics were not ready after 60 seconds.' >&2
    return 1
}

stop_process() {
    local pid="$1"
    local name="$2"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        echo "Stopping ${name}..."
        kill -INT "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}

run_inside_container() {
    local CAMERA_PID=''
    local VINS_PID=''

    cleanup() {
        trap - EXIT INT TERM
        stop_process "$VINS_PID" 'VINS-Fusion'
        stop_process "$CAMERA_PID" 'D435i camera'
    }
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM

    if [[ "$MODE" == 'camera' ]]; then
        echo 'Starting D435i camera in this terminal. Press Ctrl+C to stop it.'
        exec bash -lc "$CAMERA_RUN_COMMAND"
    fi
    if [[ "$MODE" == 'vins' ]]; then
        echo 'Starting VINS-Fusion in this terminal. Press Ctrl+C to stop it.'
        exec bash -lc "$VINS_RUN_COMMAND"
    fi

    echo 'Starting D435i camera...'
    bash -lc "$CAMERA_RUN_COMMAND" &
    CAMERA_PID=$!

    if ! wait_for_camera_topics; then
        exit 1
    fi

    echo 'Camera topics are ready; starting VINS-Fusion. Press Ctrl+C to stop both.'
    bash -lc "$VINS_RUN_COMMAND" &
    VINS_PID=$!
    wait "$VINS_PID"
}

if [[ "$DRY_RUN" == true ]]; then
    echo "Container: $CONTAINER_NAME"
    print_command "1 Camera D435i" "$CAMERA_RUN_COMMAND"
    print_command "2 VINS-Fusion" "$VINS_RUN_COMMAND"
    exit 0
fi

if [[ -f /.dockerenv ]]; then
    run_inside_container
    exit 0
fi

command -v docker >/dev/null || {
    echo 'docker is not installed.' >&2
    exit 1
}
if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    echo "Docker container '$CONTAINER_NAME' does not exist." >&2
    exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != true ]]; then
    echo "Starting container $CONTAINER_NAME..."
    docker start "$CONTAINER_NAME" >/dev/null
fi

if [[ "$MODE" == "camera" ]]; then
    echo 'Starting D435i camera in this terminal. Press Ctrl+C to stop it.'
    exec docker exec -it "$CONTAINER_NAME" bash -lc "$CAMERA_RUN_COMMAND"
fi

if [[ "$MODE" == "vins" ]]; then
    echo 'Starting VINS-Fusion in this terminal. Press Ctrl+C to stop it.'
    exec docker exec -it "$CONTAINER_NAME" bash -lc "$VINS_RUN_COMMAND"
fi

ACTIVE_NODES="$(docker exec "$CONTAINER_NAME" bash -lc \
    'source /opt/ros/noetic/setup.bash; rosnode list 2>/dev/null | grep -E "^/(vins_estimator|camera/realsense2_camera|camera/realsense2_camera_manager)$" || true')"
if [[ -n "$ACTIVE_NODES" ]]; then
    echo 'Refusing to start duplicate camera/VINS nodes. Stop these nodes first:' >&2
    echo "$ACTIVE_NODES" >&2
    exit 1
fi

open_terminal() {
    local title="$1"
    local inner_command="$2"
    local quoted_container
    local quoted_inner

    printf -v quoted_container '%q' "$CONTAINER_NAME"
    printf -v quoted_inner '%q' "$inner_command"
    gnome-terminal --title="$title" -- bash -lc \
        "exec docker exec -it ${quoted_container} bash -lc ${quoted_inner}"
}

command -v gnome-terminal >/dev/null || {
    echo 'No graphical terminal is available. Over SSH, use --camera and --vins in two terminals.' >&2
    exit 1
}

[[ -n "${DISPLAY:-}" ]] || {
    echo 'DISPLAY is not set. Over SSH, use --camera and --vins in two terminals.' >&2
    exit 1
}

echo 'Opening Camera and VINS-Fusion terminals...'
open_terminal '1 - D435i Camera' "$CAMERA_COMMAND"
sleep 1
open_terminal '2 - VINS-Fusion' "$VINS_COMMAND"
