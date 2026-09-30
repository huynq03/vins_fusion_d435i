#!/usr/bin/env bash
# Replay a D435i stereo+IMU bag through VINS-Fusion and record the resulting odometry.
# Runs inside the container; from the host it re-executes itself there.
set -eo pipefail

usage() {
    cat <<'EOF'
Usage: ./scripts/replay_bag_vins.sh BAG [--output OUT.bag] [--rate R] [--record-tracks]

  --output          default: BAG with _vins_odometry.bag suffix
  --rate            rosbag play rate (default 1.0)
  --record-tracks   also record /vins_estimator/image_track (needs show_track: 1)
EOF
}

CONTAINER="vins_d435i_compose"
WS="/home/air/vins_fusion_d435i_local"
CONFIG="$WS/bags/realsense_d435i_kalibr_183222/realsense_stereo_imu_config.yaml"

[[ $# -ge 1 && "$1" != -h && "$1" != --help ]] || { usage; exit 2; }

if [[ ! -f /.dockerenv ]]; then
    # The project is mounted at $WS: map host paths of the project into the container.
    ROOT="$(realpath "$(dirname "$0")/..")"
    ARGS=()
    for a in "$@"; do [[ -e "$a" || "$a" == *.bag ]] && a="$(realpath -m "$a")" && a="${a/#$ROOT/$WS}"; ARGS+=("$a"); done
    exec docker exec $([[ -t 0 ]] && echo -it) "$CONTAINER" bash "$WS/scripts/replay_bag_vins.sh" "${ARGS[@]}"
fi

BAG="$(realpath "$1")"; shift
OUTPUT="" RATE=1.0 TOPICS=(/vins_estimator/odometry /vins_estimator/path)
while (($#)); do
    case "$1" in
        --output) OUTPUT="$(realpath -m "$2")"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --record-tracks) TOPICS+=(/vins_estimator/image_track); shift ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
OUTPUT="${OUTPUT:-${BAG%.bag}_vins_odometry.bag}"
[[ -f "$BAG" ]] || { echo "Bag not found: $BAG" >&2; exit 1; }

source /opt/ros/noetic/setup.bash
source "$WS/catkin_ws/devel/setup.bash"
mkdir -p "$(dirname "$OUTPUT")" "$WS/output/kalibr_183222/pose_graph"

PIDS=()
cleanup() { for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do kill -INT "${PIDS[i]}" 2>/dev/null && wait "${PIDS[i]}" 2>/dev/null || true; done; }
trap cleanup EXIT

if ! rostopic list >/dev/null 2>&1; then
    roscore >/tmp/replay_roscore.log 2>&1 & PIDS+=($!)
    until rostopic list >/dev/null 2>&1; do sleep 0.2; done
fi
rosparam set use_sim_time true

rosrun vins vins_node "$CONFIG" >/tmp/replay_vins.log 2>&1 & PIDS+=($!)
until rostopic info /vins_estimator/odometry >/dev/null 2>&1; do sleep 0.2; done

rosbag record -O "$OUTPUT" "${TOPICS[@]}" >/tmp/replay_record.log 2>&1 & PIDS+=($!)
sleep 1

echo "Replaying $BAG at ${RATE}x (VINS log: /tmp/replay_vins.log)"
rosbag play --clock --rate "$RATE" "$BAG" --topics \
    /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu
sleep 3  # let VINS drain its queue before the recorder closes

cleanup; trap - EXIT
echo "Saved: $OUTPUT"
rosbag info "$OUTPUT" | sed -n '/topics:/,$p'
