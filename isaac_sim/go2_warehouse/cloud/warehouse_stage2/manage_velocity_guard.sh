#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- bash "$0" "$@"
fi
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_session=go2-velocity-guard
case "${1:-status}" in
    start)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            echo "Velocity guard session already exists; use status or stop."
            exit 1
        fi
        source "$go2_script_dir/nav_env.sh"
        go2_nodes=$(bash "$GO2_CLOUD_ROOT/work/cloud/warehouse_stage1/ros2_cloud.sh" node list --no-daemon)
        if [[ "$go2_nodes" == *"/go2_sim_velocity_guard"* ]]; then
            echo "Existing velocity guard node; refusing duplicate startup."
            exit 1
        fi
        go2_log="$GO2_CLOUD_ROOT/logs/velocity_guard_$(date +%Y%m%d_%H%M%S).log"
        printf -v go2_command '%q ' env "GO2_GUARD_BOXES=${GO2_GUARD_BOXES:-$GO2_CLOUD_ROOT/nav_stage2/maps/warehouse_dev_v2/collision_boxes.json}" bash "$go2_script_dir/start_velocity_guard.sh"
        printf -v go2_log_quoted '%q' "$go2_log"
        tmux new-session -d -s "$go2_session" "$go2_command > $go2_log_quoted 2>&1"
        echo "Session: $go2_session"
        echo "Log: $go2_log"
        ;;
    status)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}'
        else
            echo "Velocity guard session is not running."
        fi
        ;;
    stop)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux send-keys -t "$go2_session:0.0" C-c
            echo "Requested velocity guard stop; simulator, scan and planner remain running."
        else
            echo "Velocity guard session is not running."
        fi
        ;;
    *) echo "Usage: $0 {start|status|stop}"; exit 2 ;;
esac
