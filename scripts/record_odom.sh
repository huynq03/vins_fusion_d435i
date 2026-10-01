#!/usr/bin/env bash
# Record raw VINS odometry and the PX4 (EKF2) fused odometry to CSV files in
# output/odom_logs/<time>/ while the pipeline (./scripts/run.sh px4) is running.
# Runs inside the container; from the host it re-executes itself there.
set -eo pipefail

usage() {
    cat <<'EOF'
Usage: ./scripts/record_odom.sh [--output DIR] [--duration SEC] [--topic NAME=TOPIC]...

  --output     output directory (default: output/odom_logs/<YYYYmmdd_HHMMSS>)
  --duration   stop after SEC seconds (default: until Ctrl+C)
  --topic      extra nav_msgs/Odometry topic, written to NAME.csv
               e.g. --topic fc_mavlink_odom=/mavros/odometry/in

Files: vins_odom.csv (/vins_estimator/odometry), vins_flu_odom.csv (/mavros/odometry/out),
       fc_odom.csv (/mavros/local_position/odom)
EOF
}

CONTAINER="vins_d435i_compose"
WS="/home/air/vins_fusion_d435i_local"

[[ "${1:-}" != -h && "${1:-}" != --help ]] || { usage; exit 0; }

if [[ ! -f /.dockerenv ]]; then
    # The project is mounted at $WS: map a host --output path into the container.
    ROOT="$(realpath "$(dirname "$0")/..")"
    ARGS=()
    while (($#)); do
        if [[ "$1" == --output && $# -ge 2 ]]; then
            dir="$(realpath -m "$2")"; ARGS+=(--output "${dir/#$ROOT/$WS}"); shift 2
        else
            ARGS+=("$1"); shift
        fi
    done
    # Without a TTY, Ctrl+C / kill only ends the `docker exec` client: stop the recorder too.
    trap 'docker exec "$CONTAINER" pkill -INT -f record_odom_csv.py >/dev/null 2>&1 || true' EXIT
    docker exec $([[ -t 0 ]] && echo -it) "$CONTAINER" bash "$WS/scripts/record_odom.sh" "${ARGS[@]}"
    exit
fi

source /opt/ros/noetic/setup.bash
rostopic list >/dev/null 2>&1 || echo "Waiting for the ROS master (start the pipeline: ./scripts/run.sh px4)..."
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
exec python3 "$WS/scripts/record_odom_csv.py" "$@"
