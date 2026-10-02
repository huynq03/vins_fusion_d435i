#!/usr/bin/env bash
# Record raw VINS odometry, the PX4 (EKF2) fused odometry and the raw GPS of the FC to CSV files in
# output/odom_logs/<time>/ while the pipeline (./scripts/run.sh px4) is running.
# Runs inside the container; from the host it re-executes itself there.
# The recorder runs in its own tmux session "odom_rec", separate from the pipeline session.
set -eo pipefail

usage() {
    cat <<'EOF'
Usage: ./scripts/record_odom.sh [--output DIR] [--duration SEC] [--topic NAME=TOPIC]... [--no-tmux] [--no-fc-log]

Run it after ./scripts/run.sh px4: it records in its own tmux session "odom_rec".
Stop with Ctrl+C in that session or ./scripts/run.sh stop (both save the files); the session
then closes and the summary is printed in the calling terminal.
tmux: Ctrl+b d = detach, `tmux a -t odom_rec` = re-attach.

  --no-tmux    record in this terminal instead of a tmux session
  --no-fc-log  do not start SD-card logging (ULog) on the FC for the recording
  --output     output directory (default: output/odom_logs/<YYYYmmdd_HHMMSS>)
  --duration   stop after SEC seconds (default: until Ctrl+C)
  --topic      extra nav_msgs/Odometry topic, written to NAME_<time>.csv
               e.g. --topic fc_mavlink_odom=/mavros/odometry/in

Files (each named NAME_<YYYYmmdd_HHMMSS>.csv): vins_odom (/vins_estimator/odometry),
       vins_flu_odom (/mavros/odometry/out), fc_odom (/mavros/local_position/odom),
       gps_raw (/mavros/gpsstatus/gps1/raw, GPS / UWB before EKF2: lat/lon plus local x/y/z)
EOF
}

CONTAINER="vins_d435i_compose"
SESSION="odom_rec"
WS="/home/air/vins_fusion_d435i_local"

[[ "${1:-}" != -h && "${1:-}" != --help ]] || { usage; exit 0; }

USE_TMUX=true
ARGS=()
for arg in "$@"; do
    if [[ "$arg" == --no-tmux ]]; then USE_TMUX=false; else ARGS+=("$arg"); fi
done
set -- "${ARGS[@]}"

# The session runs this script again with --no-tmux and closes itself when the recorder ends,
# so the calling terminal comes back; the pane output is kept in $PANE_LOG for the summary.
if [[ "$USE_TMUX" == true ]] && command -v tmux >/dev/null; then
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Already recording in tmux session '$SESSION'. Attach: tmux a -t $SESSION" >&2
        exit 1
    fi
    PANE_LOG="${TMPDIR:-/tmp}/$SESSION.log"
    : > "$PANE_LOG"
    tmux new-session -d -s "$SESSION" -n record -c "$PWD" "$(printf '%q ' bash "$(realpath "$0")" --no-tmux "$@")"
    tmux pipe-pane -t "$SESSION" "cat > $(printf '%q' "$PANE_LOG")"
    tmux set-option -t "$SESSION" mouse on >/dev/null
    # Inside tmux, go back to the previous session instead of leaving tmux when this one ends.
    tmux set-option -t "$SESSION" detach-on-destroy off >/dev/null
    echo "Recording in tmux session '$SESSION' (tmux a -t $SESSION)."
    echo "Stop: Ctrl+C in that session, or ./scripts/run.sh stop."
    # Without a terminal (scripts, ssh without -t) the recording just keeps running detached.
    [[ -t 1 ]] || exit 0
    if [[ -n "${TMUX:-}" ]]; then tmux switch-client -t "$SESSION"; exit 0; fi
    tmux attach -t "$SESSION" || true
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Still recording (detached). Attach: tmux a -t $SESSION"
    else
        # Recording ended: show its summary here, or its last lines if it failed before saving.
        tr -d '\r' < "$PANE_LOG" | grep -A100 '^FC log: stopped\|^Saved:' || tail -n 15 "$PANE_LOG"
    fi
    exit 0
fi

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
    # Pass the host time zone so file names use local time even in a container created
    # before compose.yaml mounted /etc/localtime (its clock is UTC).
    docker exec -e TZ="${TZ:-$(cat /etc/timezone 2>/dev/null || echo UTC)}" $([[ -t 0 ]] && echo -it) "$CONTAINER" bash "$WS/scripts/record_odom.sh" "${ARGS[@]}"
    exit
fi

source /opt/ros/noetic/setup.bash
rostopic list >/dev/null 2>&1 || echo "Waiting for the ROS master (start the pipeline: ./scripts/run.sh px4)..."
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
exec python3 "$WS/scripts/record_odom_csv.py" "$@"
