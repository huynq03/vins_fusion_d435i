#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="${VINS_CONTAINER:-vins_d435i_compose}"
CAMERA_LAUNCH="${PROJECT_DIR}/bags/realsense_d435i_kalibr_183222/rs_camera.launch"
VINS_CONFIG="${PROJECT_DIR}/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml"
OUTPUT_DIR="/work/bags/recordings"
BAG_NAME="d435i_rgb_vins_$(date +%Y%m%d_%H%M%S)"
INITIAL_RESET=false

CAMERA_LOG="/tmp/d435i_rgb_camera.log"
VINS_LOG="/tmp/d435i_vins.log"
ROSBAG_LOG="/tmp/d435i_rosbag.log"

CAMERA_PID=""
VINS_PID=""
ROSBAG_PID=""
CLEANED_UP=false

usage()
{
    cat <<'EOF'
Usage:
  ./scripts/run_rgb_vins_record_bag.sh [options]

Automatically enter the Docker container, start D435i RGB + stereo IR + IMU,
start VINS-Fusion, and record a timestamped rosbag. Press Ctrl+C to stop all
processes and finalize the bag.

Options:
  --container NAME   Docker container (default: vins_d435i_compose).
  --output-dir DIR   Bag directory inside Docker (default: /work/bags/recordings).
  --name NAME        Bag filename without .bag.
  --initial-reset    Reset the D435i once during startup.
  -h, --help         Show this help.
EOF
}

ORIGINAL_ARGS=("$@")
while (($# > 0)); do
    case "$1" in
        --container)
            [[ $# -ge 2 ]] || { echo "Missing value after --container" >&2; exit 2; }
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --output-dir)
            [[ $# -ge 2 ]] || { echo "Missing value after --output-dir" >&2; exit 2; }
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --name)
            [[ $# -ge 2 ]] || { echo "Missing value after --name" >&2; exit 2; }
            BAG_NAME="${2%.bag}"
            shift 2
            ;;
        --initial-reset)
            INITIAL_RESET=true
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

# When invoked from the Jetson host, start/enter Docker and run this same file.
if [[ ! -f /.dockerenv ]]; then
    echo "===== CHECK DOCKER ====="
    command -v docker >/dev/null 2>&1 || {
        echo "ERROR: docker is not installed." >&2
        exit 1
    }

    docker inspect "$CONTAINER_NAME" >/dev/null 2>&1 || {
        echo "ERROR: Docker container '$CONTAINER_NAME' does not exist." >&2
        exit 1
    }

    if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != true ]]; then
        echo "Starting container: $CONTAINER_NAME"
        docker start "$CONTAINER_NAME" >/dev/null
    fi

    echo "Entering container: $CONTAINER_NAME"
    exec docker exec -it \
        -e VINS_CONTAINER="$CONTAINER_NAME" \
        "$CONTAINER_NAME" \
        bash "/home/hann/vins_fusion_d435i_local/scripts/run_rgb_vins_record_bag.sh" \
        "${ORIGINAL_ARGS[@]}"
fi

echo "===== SOURCE ROS1 + WORKSPACES ====="
source /opt/ros/noetic/setup.bash
source "${PROJECT_DIR}/rs_ros_ws/devel/setup.bash" --extend
source "${PROJECT_DIR}/catkin_ws/devel/setup.bash" --extend

export PATH="/opt/librealsense/bin:${PATH}"
export LD_LIBRARY_PATH="${PROJECT_DIR}/rs_ros_ws/devel/lib:/opt/librealsense/lib:${LD_LIBRARY_PATH:-}"

for command_name in roslaunch rosrun rosbag rostopic rosnode timeout; do
    command -v "$command_name" >/dev/null 2>&1 || {
        echo "ERROR: required command not found: $command_name" >&2
        exit 1
    }
done

echo "===== CHECK D435i ====="
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices | grep -E \
        "Name|Serial Number|Firmware Version|Usb Type Descriptor|Product Line" || true
else
    echo "WARN: rs-enumerate-devices not found; continuing with ROS camera check."
fi

if rosnode list >/dev/null 2>&1; then
    ACTIVE_NODES="$(rosnode list | grep -E '^/(camera/realsense2_camera|camera/realsense2_camera_manager|vins_estimator)$' || true)"
    if [[ -n "$ACTIVE_NODES" ]]; then
        echo "ERROR: camera or VINS nodes are already running:" >&2
        echo "$ACTIVE_NODES" >&2
        echo "Stop them before running this script." >&2
        exit 1
    fi
fi

if pgrep -fa 'rs_camera\.launch|RealSenseNodeFactory|vins_node' >/dev/null 2>&1; then
    echo "ERROR: an old camera or VINS process is still alive:" >&2
    pgrep -fa 'rs_camera\.launch|RealSenseNodeFactory|vins_node' >&2 || true
    exit 1
fi

mkdir -p "$OUTPUT_DIR" "${PROJECT_DIR}/output/kalibr_183222/pose_graph"
BAG_PATH="${OUTPUT_DIR}/${BAG_NAME}.bag"

stop_process()
{
    local pid="$1"
    local name="$2"
    local max_wait="${3:-30}"

    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        echo "Stopping ${name}..."
        kill -INT "$pid" 2>/dev/null || true
        for ((i = 0; i < max_wait; i++)); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
        wait "$pid" 2>/dev/null || true
    fi
}

cleanup()
{
    local status=$?
    [[ "$CLEANED_UP" == true ]] && return
    CLEANED_UP=true
    trap - INT TERM EXIT

    echo
    echo "===== STOPPING ====="
    # Give rosbag up to 30 seconds to flush RGB data and write its index.
    stop_process "$ROSBAG_PID" "rosbag recorder" 300
    stop_process "$VINS_PID" "VINS-Fusion"
    stop_process "$CAMERA_PID" "D435i camera"

    if [[ -f "$BAG_PATH" ]]; then
        echo "Saved bag: $BAG_PATH"
        ls -lh "$BAG_PATH"
    elif [[ -f "${BAG_PATH}.active" ]]; then
        echo "WARN: incomplete bag remains: ${BAG_PATH}.active" >&2
    fi
    echo "Stopped."
    exit "$status"
}

trap cleanup INT TERM EXIT

wait_for_live_topic()
{
    local topic="$1"
    local owner_pid="$2"
    local owner_name="$3"
    local max_seconds="$4"

    echo "Waiting for $topic ..."
    for ((second = 1; second <= max_seconds; second++)); do
        if ! kill -0 "$owner_pid" 2>/dev/null; then
            echo "ERROR: $owner_name exited while waiting for $topic." >&2
            return 1
        fi

        if rostopic info "$topic" >/dev/null 2>&1 && \
           timeout 2 rostopic echo -n1 "$topic" >/dev/null 2>&1; then
            echo "OK: $topic is live"
            return 0
        fi
        sleep 1
    done

    echo "ERROR: no live sample from $topic after ${max_seconds} seconds." >&2
    return 1
}

show_log_and_exit()
{
    local title="$1"
    local log_file="$2"
    echo "===== ${title} =====" >&2
    tail -120 "$log_file" >&2 || true
    exit 1
}

echo
echo "===== START D435i RGB + STEREO IR + IMU ====="
roslaunch "$CAMERA_LAUNCH" \
    enable_color:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30 \
    enable_sync:=false \
    initial_reset:="$INITIAL_RESET" \
    >"$CAMERA_LOG" 2>&1 &
CAMERA_PID=$!

echo "Camera PID: $CAMERA_PID"
echo "Camera log: $CAMERA_LOG"

wait_for_live_topic /camera/color/image_raw "$CAMERA_PID" "D435i camera" 30 || \
    show_log_and_exit "CAMERA LOG" "$CAMERA_LOG"
wait_for_live_topic /camera/infra1/image_rect_raw "$CAMERA_PID" "D435i camera" 15 || \
    show_log_and_exit "CAMERA LOG" "$CAMERA_LOG"
wait_for_live_topic /camera/infra2/image_rect_raw "$CAMERA_PID" "D435i camera" 15 || \
    show_log_and_exit "CAMERA LOG" "$CAMERA_LOG"
wait_for_live_topic /camera/imu "$CAMERA_PID" "D435i camera" 20 || \
    show_log_and_exit "CAMERA LOG" "$CAMERA_LOG"

echo
echo "===== START VINS-FUSION ====="
rosrun vins vins_node "$VINS_CONFIG" >"$VINS_LOG" 2>&1 &
VINS_PID=$!

echo "VINS PID: $VINS_PID"
echo "VINS log: $VINS_LOG"
echo "Waiting for /vins_estimator/odometry ..."

for second in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -qx "/vins_estimator/odometry"; then
        echo "OK: /vins_estimator/odometry found"
        break
    fi

    if ! kill -0 "$VINS_PID" 2>/dev/null; then
        show_log_and_exit "VINS LOG" "$VINS_LOG"
    fi

    sleep 1
    if [[ "$second" == 60 ]]; then
        echo "ERROR: /vins_estimator/odometry not found after 60 seconds." >&2
        show_log_and_exit "VINS LOG" "$VINS_LOG"
    fi
done

echo
echo "===== START ROSBAG RECORDING ====="
RECORD_TOPICS=(
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/infra1/image_rect_raw
    /camera/infra1/camera_info
    /camera/infra2/image_rect_raw
    /camera/infra2/camera_info
    /camera/imu
    /vins_estimator/odometry
    /vins_estimator/path
)

rosbag record \
    --lz4 \
    --buffsize=512 \
    -O "$BAG_PATH" \
    "${RECORD_TOPICS[@]}" \
    >"$ROSBAG_LOG" 2>&1 &
ROSBAG_PID=$!

sleep 2
if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
    show_log_and_exit "ROSBAG LOG" "$ROSBAG_LOG"
fi

echo
echo "===== READY ====="
echo "RGB topic:"
echo "  /camera/color/image_raw"
echo "Stereo IR topics:"
echo "  /camera/infra1/image_rect_raw"
echo "  /camera/infra2/image_rect_raw"
echo "IMU topic:"
echo "  /camera/imu"
echo "VINS odometry topic:"
echo "  /vins_estimator/odometry"
echo
echo "Recording bag:"
echo "  $BAG_PATH"
echo
echo "Logs:"
echo "  $CAMERA_LOG"
echo "  $VINS_LOG"
echo "  $ROSBAG_LOG"
echo
echo "Press Ctrl+C here once to stop and save the bag."

wait "$ROSBAG_PID"
