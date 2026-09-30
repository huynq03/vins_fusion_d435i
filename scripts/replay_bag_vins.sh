#!/usr/bin/env bash
# Replay a D435i stereo-IMU ROS1 bag through the local VINS-Fusion build.
# ROS Noetic's setup scripts read optional environment variables before
# assigning defaults, so nounset cannot be enabled while sourcing them.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_NAME="${VINS_CONTAINER:-vins_d435i_compose}"
CONFIG="${PROJECT_DIR}/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml"
RATE=1.0
RECORD_TRACKS=false

usage() {
    cat <<'EOF'
Usage:
  ./scripts/replay_bag_vins.sh BAG_FILE [options]

Replay D435i stereo images and IMU from a ROS1 bag into VINS-Fusion, then
record the reconstructed /vins_estimator/odometry to a separate bag.

Options:
  --output PATH       Output odometry bag (default: BAG_FILE with _vins_odometry.bag).
  --rate RATE         rosbag playback rate, default: 1.0.
  --container NAME    Docker container, default: vins_d435i_compose.
  --config PATH       VINS YAML config, default: D435i Kalibr config.
  --record-tracks     Also record /vins_estimator/image_track (requires show_track: 1).
  -h, --help          Show this help.
EOF
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
case "$1" in
    -h|--help) usage; exit 0 ;;
esac
BAG_FILE="$1"
shift
OUTPUT=""

while (($# > 0)); do
    case "$1" in
        --output) [[ $# -ge 2 ]] || { echo "Missing value after --output" >&2; exit 2; }; OUTPUT="$2"; shift 2 ;;
        --rate) [[ $# -ge 2 ]] || { echo "Missing value after --rate" >&2; exit 2; }; RATE="$2"; shift 2 ;;
        --container) [[ $# -ge 2 ]] || { echo "Missing value after --container" >&2; exit 2; }; CONTAINER_NAME="$2"; shift 2 ;;
        --config) [[ $# -ge 2 ]] || { echo "Missing value after --config" >&2; exit 2; }; CONFIG="$2"; shift 2 ;;
        --record-tracks) RECORD_TRACKS=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

# The project directory is mounted at the same location inside the Compose
# container, so relative paths are resolved before crossing the Docker boundary.
if [[ ! -f /.dockerenv ]]; then
    command -v docker >/dev/null 2>&1 || {
        echo "Docker is required when this script is run on the host." >&2
        exit 1
    }
    docker inspect "$CONTAINER_NAME" >/dev/null 2>&1 || {
        echo "Docker container '$CONTAINER_NAME' does not exist." >&2
        exit 1
    }
    if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != true ]]; then
        docker start "$CONTAINER_NAME" >/dev/null
    fi
    EXTRA_ARGS=()
    if [[ "$RECORD_TRACKS" == true ]]; then EXTRA_ARGS+=(--record-tracks); fi
    exec docker exec -it "$CONTAINER_NAME" bash \
        "/home/hann/vins_fusion_d435i_local/scripts/replay_bag_vins.sh" \
        "$BAG_FILE" --output "${OUTPUT:-}" --rate "$RATE" --config "$CONFIG" "${EXTRA_ARGS[@]}"
fi

if [[ "$BAG_FILE" != /* ]]; then
    BAG_FILE="${PROJECT_DIR}/${BAG_FILE}"
fi
if [[ -z "$OUTPUT" ]]; then
    OUTPUT="${BAG_FILE%.bag}_vins_odometry.bag"
elif [[ "$OUTPUT" != /* ]]; then
    OUTPUT="${PROJECT_DIR}/${OUTPUT}"
fi
if [[ "$CONFIG" != /* ]]; then
    CONFIG="${PROJECT_DIR}/${CONFIG}"
fi

[[ -f "$BAG_FILE" ]] || { echo "Bag does not exist: $BAG_FILE" >&2; exit 1; }
[[ -f "$CONFIG" ]] || { echo "VINS config does not exist: $CONFIG" >&2; exit 1; }
[[ "$OUTPUT" != "$BAG_FILE" ]] || { echo "Output bag must differ from input bag." >&2; exit 2; }

source /opt/ros/noetic/setup.bash
source "${PROJECT_DIR}/catkin_ws/devel/setup.bash" --extend

for command_name in roscore rosbag rosrun rosparam rostopic; do
    command -v "$command_name" >/dev/null 2>&1 || {
        echo "Required ROS command not found: $command_name" >&2
        exit 1
    }
done

mkdir -p "$(dirname "$OUTPUT")" "${PROJECT_DIR}/output/kalibr_183222/pose_graph"

ROSCORE_PID=""
VINS_PID=""
RECORDER_PID=""
PLAYER_PID=""

stop_process() {
    local pid="$1"
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null || return 0
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_process "$PLAYER_PID"
    stop_process "$RECORDER_PID"
    stop_process "$VINS_PID"
    stop_process "$ROSCORE_PID"
    exit "$status"
}
trap cleanup EXIT INT TERM

if ! rostopic list >/dev/null 2>&1; then
    roscore >/tmp/vins_bag_roscore.log 2>&1 &
    ROSCORE_PID=$!
    for _ in {1..50}; do rostopic list >/dev/null 2>&1 && break; sleep 0.1; done
fi
rostopic list >/dev/null 2>&1 || { echo "ROS master did not start." >&2; exit 1; }

rosparam set use_sim_time true

echo "Starting VINS-Fusion with: $CONFIG"
(cd "$(dirname "$CONFIG")" && exec rosrun vins vins_node "$CONFIG") >/tmp/vins_bag_vins.log 2>&1 &
VINS_PID=$!

for _ in {1..100}; do
    rostopic info /vins_estimator/odometry >/dev/null 2>&1 && break
    kill -0 "$VINS_PID" 2>/dev/null || { tail -100 /tmp/vins_bag_vins.log >&2; exit 1; }
    sleep 0.1
done
rostopic info /vins_estimator/odometry >/dev/null 2>&1 || {
    echo "VINS did not advertise /vins_estimator/odometry." >&2
    tail -100 /tmp/vins_bag_vins.log >&2
    exit 1
}

echo "Recording reconstructed odometry: $OUTPUT"
RECORD_TOPICS=(/vins_estimator/odometry /vins_estimator/path)
if [[ "$RECORD_TRACKS" == true ]]; then RECORD_TOPICS+=(/vins_estimator/image_track); fi
rosbag record -O "$OUTPUT" "${RECORD_TOPICS[@]}" >/tmp/vins_bag_record.log 2>&1 &
RECORDER_PID=$!
sleep 0.5

echo "Replaying camera and IMU at ${RATE}x..."
rosbag play --clock --rate "$RATE" "$BAG_FILE" --topics \
    /camera/infra1/image_rect_raw \
    /camera/infra2/image_rect_raw \
    /camera/imu &
PLAYER_PID=$!
while kill -0 "$PLAYER_PID" 2>/dev/null; do
    if ! kill -0 "$VINS_PID" 2>/dev/null; then
        echo "VINS exited during playback." >&2
        tail -100 /tmp/vins_bag_vins.log >&2
        exit 1
    fi
    if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
        echo "Odometry recorder exited during playback." >&2
        tail -100 /tmp/vins_bag_record.log >&2
        exit 1
    fi
    sleep 1
done
wait "$PLAYER_PID"
PLAYER_PID=""

# Let VINS finish processing queued camera measurements before the recorder is closed.
sleep 3
stop_process "$RECORDER_PID"
RECORDER_PID=""

echo "Saved: $OUTPUT"
rosbag info "$OUTPUT" | sed -n '/topics:/,$p'
