#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER_NAME="${VINS_CONTAINER:-vins_d435i_local}"
DEFAULT_FCU_URL="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600"
FCU_URL="${FCU_URL:-$DEFAULT_FCU_URL}"
CAMERA_INITIAL_RESET="${CAMERA_INITIAL_RESET:-false}"
DRY_RUN=false

usage()
{
    cat <<'EOF'
Usage:
  ./scripts/run_vins_px4_4_terminals.sh [options]

Options:
  --fcu-url URL          MAVROS FCU URL.
  --initial-reset        Reset the D435i once when the camera node starts.
  --container NAME       Docker container name (default: vins_d435i_local).
  --dry-run              Print the four terminal commands without running them.
  -h, --help             Show this help.

Environment alternatives:
  FCU_URL, VINS_CONTAINER, CAMERA_INITIAL_RESET

Examples:
  ./scripts/run_vins_px4_4_terminals.sh
  ./scripts/run_vins_px4_4_terminals.sh --initial-reset
  ./scripts/run_vins_px4_4_terminals.sh --fcu-url /dev/ttyACM0:921600
EOF
}

while (($# > 0)); do
    case "$1" in
        --fcu-url)
            [[ $# -ge 2 ]] || { echo "Missing value after --fcu-url" >&2; exit 2; }
            FCU_URL="$2"
            shift 2
            ;;
        --initial-reset)
            CAMERA_INITIAL_RESET=true
            shift
            ;;
        --container)
            [[ $# -ge 2 ]] || { echo "Missing value after --container" >&2; exit 2; }
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
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

CAMERA_COMMAND="source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/setup.bash
export PATH=/opt/librealsense/bin:\$PATH
export LD_LIBRARY_PATH=/home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:\$LD_LIBRARY_PATH
roslaunch /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/rs_camera.launch initial_reset:=${CAMERA_INITIAL_RESET}
exec bash"

VINS_COMMAND="source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash
mkdir -p /home/hann/vins_fusion_d435i_local/output/kalibr_183222/pose_graph
rosrun vins vins_node /home/hann/vins_fusion_d435i_local/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml
exec bash"

MAVROS_COMMAND="source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=${FCU_URL}
exec bash"

BRIDGE_COMMAND="source /opt/ros/noetic/setup.bash
source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash
roslaunch vins vins_px4_bridge.launch input_topic:=/vins_estimator/odometry output_topic:=/mavros/odometry/out parent_frame_id:=odom child_frame_id:=base_link
exec bash"

print_command()
{
    local title="$1"
    local command="$2"
    printf '\n[%s]\n%s\n' "$title" "$command"
}

if [[ "$DRY_RUN" == true ]]; then
    echo "Container: $CONTAINER_NAME"
    echo "FCU URL:   $FCU_URL"
    echo "D435i initial reset: $CAMERA_INITIAL_RESET"
    print_command "1 Camera D435i" "$CAMERA_COMMAND"
    print_command "2 VINS-Fusion" "$VINS_COMMAND"
    print_command "3 MAVROS PX4" "$MAVROS_COMMAND"
    print_command "4 VINS-PX4 ODOMETRY bridge" "$BRIDGE_COMMAND"
    exit 0
fi

command -v docker >/dev/null || { echo "docker is not installed" >&2; exit 1; }
command -v gnome-terminal >/dev/null || {
    echo "gnome-terminal is required. Run this script from the Jetson desktop." >&2
    exit 1
}
[[ -n "${DISPLAY:-}" ]] || {
    echo "DISPLAY is not set. Run this script from a graphical Jetson terminal." >&2
    exit 1
}

if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    echo "Docker container '$CONTAINER_NAME' does not exist." >&2
    exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
    echo "Starting container $CONTAINER_NAME..."
    docker start "$CONTAINER_NAME" >/dev/null
fi

if ! docker exec "$CONTAINER_NAME" bash -lc \
    'source /opt/ros/noetic/setup.bash; rosnode list >/dev/null 2>&1'; then
    echo "Starting a persistent roscore inside $CONTAINER_NAME..."
    docker exec -d "$CONTAINER_NAME" bash -lc \
        'source /opt/ros/noetic/setup.bash; exec roscore >/tmp/vins_px4_roscore.log 2>&1'

    ROS_MASTER_READY=false
    for _ in {1..20}; do
        if docker exec "$CONTAINER_NAME" bash -lc \
            'source /opt/ros/noetic/setup.bash; rosnode list >/dev/null 2>&1'; then
            ROS_MASTER_READY=true
            break
        fi
        sleep 0.5
    done

    if [[ "$ROS_MASTER_READY" != true ]]; then
        echo "roscore did not become ready. Log: /tmp/vins_px4_roscore.log" >&2
        exit 1
    fi
fi

ACTIVE_TARGETS="$(docker exec "$CONTAINER_NAME" bash -lc \
    'source /opt/ros/noetic/setup.bash; rosnode list 2>/dev/null | grep -E "^/(camera/realsense2_camera|camera/realsense2_camera_manager|vins_estimator|mavros|vins_px4_bridge)$" || true')"

if [[ -n "$ACTIVE_TARGETS" ]]; then
    echo "Refusing to open duplicate nodes. Stop these nodes first:" >&2
    echo "$ACTIVE_TARGETS" >&2
    exit 1
fi

ACTIVE_REALSENSE="$(docker exec "$CONTAINER_NAME" bash -lc \
    'ps -eo cmd | grep -E "[n]odelet (manager|load.*RealSenseNodeFactory)" || true')"

if [[ -n "$ACTIVE_REALSENSE" ]]; then
    echo "A RealSense nodelet process is still alive. Stop it and wait before retrying:" >&2
    echo "$ACTIVE_REALSENSE" >&2
    exit 1
fi

open_terminal()
{
    local title="$1"
    local inner_command="$2"
    local quoted_container
    local quoted_inner

    printf -v quoted_container '%q' "$CONTAINER_NAME"
    printf -v quoted_inner '%q' "$inner_command"

    gnome-terminal \
        --title="$title" \
        -- bash -lc "exec docker exec -it ${quoted_container} bash -lc ${quoted_inner}"
}

echo "Opening four terminals..."
echo "  1. Camera D435i"
echo "  2. VINS-Fusion"
echo "  3. MAVROS -> PX4 ($FCU_URL)"
echo "  4. MAVLink ODOMETRY bridge"

open_terminal "1 - D435i Camera" "$CAMERA_COMMAND"
sleep 1
open_terminal "2 - VINS-Fusion" "$VINS_COMMAND"
open_terminal "3 - MAVROS PX4" "$MAVROS_COMMAND"
open_terminal "4 - VINS PX4 ODOMETRY Bridge" "$BRIDGE_COMMAND"

echo "Done. Keep roscore running; it was started independently in the container."
