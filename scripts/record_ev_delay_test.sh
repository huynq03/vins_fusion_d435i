#!/usr/bin/env bash
set -Eeuo pipefail

OUTPUT_DIR="/work/output/ev_delay"
MOVE_SECONDS=5
STOP_SECONDS=3
SKIP_ULOG_CONFIRM=false

usage()
{
    cat <<'EOF'
Usage:
  ./scripts/record_ev_delay_test.sh [options]

Record the ROS side of the PX4 external-vision delay test and guide the
operator through +X, -X, +Y and -Y translations without yaw rotation.

Options:
  --output-dir DIR       Output directory (default: /work/output/ev_delay).
  --move-seconds N       Duration of each slow translation (default: 5).
  --stop-seconds N       Duration of each stop after motion (default: 3).
  --skip-ulog-confirm    Do not wait for the ULog readiness confirmation.
  -h, --help             Show this help.

Run inside the ROS Noetic container after camera, VINS, MAVROS and the bridge
are active. This script never changes a PX4 parameter.
EOF
}

while (($# > 0)); do
    case "$1" in
        --output-dir)
            [[ $# -ge 2 ]] || { echo "Missing value after --output-dir" >&2; exit 2; }
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --move-seconds)
            [[ $# -ge 2 ]] || { echo "Missing value after --move-seconds" >&2; exit 2; }
            MOVE_SECONDS="$2"
            shift 2
            ;;
        --stop-seconds)
            [[ $# -ge 2 ]] || { echo "Missing value after --stop-seconds" >&2; exit 2; }
            STOP_SECONDS="$2"
            shift 2
            ;;
        --skip-ulog-confirm)
            SKIP_ULOG_CONFIRM=true
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

[[ "$MOVE_SECONDS" =~ ^[1-9][0-9]*$ ]] || {
    echo "--move-seconds must be a positive integer" >&2
    exit 2
}
[[ "$STOP_SECONDS" =~ ^[1-9][0-9]*$ ]] || {
    echo "--stop-seconds must be a positive integer" >&2
    exit 2
}

for command_name in rosbag rostopic rosnode date mkdir timeout; do
    command -v "$command_name" >/dev/null || {
        echo "Required command is missing: $command_name" >&2
        exit 1
    }
done

rosnode list >/dev/null 2>&1 || {
    echo "ROS master is not reachable. Source ROS and start the pipeline first." >&2
    exit 1
}

mapfile -t ACTIVE_TOPICS < <(rostopic list | sort)

topic_is_active()
{
    local target="$1"
    local topic
    for topic in "${ACTIVE_TOPICS[@]}"; do
        [[ "$topic" == "$target" ]] && return 0
    done
    return 1
}

REQUIRED_TOPICS=(
    /vins_estimator/odometry
    /mavros/odometry/out
    /mavros/local_position/odom
    /mavros/local_position/velocity_local
)

declare -A REQUIRED_TYPES=(
    [/vins_estimator/odometry]=nav_msgs/Odometry
    [/mavros/odometry/out]=nav_msgs/Odometry
    [/mavros/local_position/odom]=nav_msgs/Odometry
    [/mavros/local_position/velocity_local]=geometry_msgs/TwistStamped
)

MISSING_TOPICS=()
for topic in "${REQUIRED_TOPICS[@]}"; do
    topic_is_active "$topic" || MISSING_TOPICS+=("$topic")
done

if ((${#MISSING_TOPICS[@]} > 0)); then
    echo "Refusing to record an incomplete delay test. Missing topics:" >&2
    printf '  %s\n' "${MISSING_TOPICS[@]}" >&2
    exit 1
fi

for topic in "${REQUIRED_TOPICS[@]}"; do
    actual_type="$(rostopic type "$topic" 2>/dev/null || true)"
    if [[ "$actual_type" != "${REQUIRED_TYPES[$topic]}" ]]; then
        echo "Unexpected type for $topic: '$actual_type' (expected ${REQUIRED_TYPES[$topic]})" >&2
        exit 1
    fi
    if ! timeout 5 rostopic echo -n1 "$topic" >/dev/null 2>&1; then
        echo "Topic is advertised but no live sample arrived within 5 s: $topic" >&2
        exit 1
    fi
done

RECORD_TOPICS=("${REQUIRED_TOPICS[@]}")
OPTIONAL_TOPICS=(
    /mavros/estimator_status
    /mavros/state
    /mavros/extended_state
    /mavros/sys_status
    /mavros/timesync_status
    /mavros/time_reference
    /mavros/statustext/recv
    /mavros/local_position/pose_cov
    /mavros/imu/data
    /diagnostics
    /rosout
    /rosout_agg
)

topic_is_selected()
{
    local target="$1"
    local selected
    for selected in "${RECORD_TOPICS[@]}"; do
        [[ "$selected" == "$target" ]] && return 0
    done
    return 1
}

append_active_topic()
{
    local topic="$1"
    if topic_is_active "$topic" && ! topic_is_selected "$topic"; then
        RECORD_TOPICS+=("$topic")
    fi
}

for topic in "${OPTIONAL_TOPICS[@]}"; do
    append_active_topic "$topic"
done

for topic in "${ACTIVE_TOPICS[@]}"; do
    case "$topic" in
        /mavros/*estimator*|/mavros/*status*|/mavros/state|/mavros/extended_state|/mavros/time_reference)
            append_active_topic "$topic"
            ;;
    esac
done

mkdir -p "$OUTPUT_DIR"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
BAG_PREFIX="${OUTPUT_DIR}/ev_delay_${RUN_ID}"
PHASE_FILE="${BAG_PREFIX}_phases.csv"
TOPIC_FILE="${BAG_PREFIX}_topics.txt"

printf '%s\n' "${RECORD_TOPICS[@]}" > "$TOPIC_FILE"
printf 'phase,event,wall_epoch_s,wall_iso8601\n' > "$PHASE_FILE"

echo "ROS topics to record:"
printf '  %s\n' "${RECORD_TOPICS[@]}"
echo
echo "PX4 ULog must run for the same test."
echo "In the QGroundControl MAVLink Console (PX4 v1.14.x), check:"
echo "  logger status"
echo "If logger is running but waiting for arming, start logging now with:"
echo "  logger on"
echo "If the logger module is not running, start it and log immediately with:"
echo "  logger start -e -t"
echo "After the test, run 'logger status' and retain the matching .ulg file."
echo

if [[ "$SKIP_ULOG_CONFIRM" != true ]]; then
    read -r -p "Press Enter only after ULog is confirmed active and yaw can stay fixed. "
fi

rosbag record \
    --lz4 \
    --buffsize=256 \
    -O "$BAG_PREFIX" \
    "${RECORD_TOPICS[@]}" &
ROSBAG_PID=$!

stop_recorder()
{
    local status=$?
    local recorder_status=0
    local recorder_was_running=false
    trap - EXIT INT TERM
    if kill -0 "$ROSBAG_PID" 2>/dev/null; then
        recorder_was_running=true
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
    fi
    if wait "$ROSBAG_PID" 2>/dev/null; then
        recorder_status=0
    else
        recorder_status=$?
    fi
    if [[ "$recorder_was_running" != true && "$status" -eq 0 ]]; then
        status="$recorder_status"
        [[ "$status" -ne 0 ]] || status=1
    fi
    echo
    if [[ "$status" -eq 0 ]]; then
        echo "Recording finalized successfully."
    else
        echo "Test incomplete; rosbag or operator sequence exited with status $status." >&2
    fi
    echo "ROS bag prefix: $BAG_PREFIX"
    echo "Phase log:      $PHASE_FILE"
    echo "Topic list:     $TOPIC_FILE"
    exit "$status"
}
trap stop_recorder EXIT
trap 'exit 130' INT TERM

ensure_recorder_alive()
{
    if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "rosbag record exited before the test completed" >&2
        return 1
    fi
}

sleep 2
ensure_recorder_alive

mark_phase()
{
    local phase="$1"
    local event="$2"
    printf '%s,%s,%s,%s\n' \
        "$phase" \
        "$event" \
        "$(date +%s.%N)" \
        "$(date --iso-8601=ns)" >> "$PHASE_FILE"
}

countdown()
{
    local duration="$1"
    local remaining
    for ((remaining=duration; remaining>0; remaining--)); do
        ensure_recorder_alive
        printf '\r  %2d s remaining ' "$remaining"
        sleep 1
    done
    ensure_recorder_alive
    printf '\r  phase complete  \a\n'
}

run_phase()
{
    local name="$1"
    local instruction="$2"
    local duration="$3"
    echo
    echo "NEXT: $instruction"
    ensure_recorder_alive
    read -r -p "Press Enter when ready to start '$name'. "
    ensure_recorder_alive
    echo "START $name -- keep yaw fixed."
    mark_phase "$name" start
    countdown "$duration"
    mark_phase "$name" end
}

run_phase stationary_initial "Keep the camera completely still" 5
run_phase move_pos_x "Move slowly along world +X; do not rotate yaw" "$MOVE_SECONDS"
run_phase stop_after_pos_x "Stop and hold still" "$STOP_SECONDS"
run_phase move_neg_x "Move slowly along world -X; do not rotate yaw" "$MOVE_SECONDS"
run_phase stop_after_neg_x "Stop and hold still" "$STOP_SECONDS"
run_phase move_pos_y "Move slowly along world +Y; do not rotate yaw" "$MOVE_SECONDS"
run_phase stop_after_pos_y "Stop and hold still" "$STOP_SECONDS"
run_phase move_neg_y "Move slowly along world -Y; do not rotate yaw" "$MOVE_SECONDS"
run_phase stop_final "Stop and hold still" "$STOP_SECONDS"

echo
echo "Test sequence complete. Stopping rosbag cleanly..."
exit 0
