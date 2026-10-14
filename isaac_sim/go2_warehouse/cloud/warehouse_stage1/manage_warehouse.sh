#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- env DISPLAY=:1 XAUTHORITY=/home/admin/.Xauthority bash "$0" "$@"
fi
source "$(dirname "${BASH_SOURCE[0]}")/cloud_env.sh"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/home/admin/.Xauthority}"
go2_action="${1:-status}"
if [[ $# -gt 0 ]]; then shift; fi
go2_session=go2-warehouse
case "$go2_action" in
    start)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            echo "Session $go2_session already exists; use status or stop."
            exit 1
        fi
        go2_run_id="$(date +%Y%m%d_%H%M%S)"
        go2_output="${GO2_CLOUD_ROOT}/artifacts/${go2_run_id}"
        go2_log="${GO2_CLOUD_ROOT}/logs/warehouse_${go2_run_id}.log"
        go2_extra_args=("$@")
        for ((go2_arg_idx=0; go2_arg_idx<${#go2_extra_args[@]}; go2_arg_idx++)); do
            if [[ ${go2_extra_args[go2_arg_idx]} == --artifact-dir ]]; then
                if ((go2_arg_idx + 1 >= ${#go2_extra_args[@]})); then
                    echo "--artifact-dir requires a path."
                    exit 2
                fi
                go2_output="${go2_extra_args[go2_arg_idx+1]}"
            fi
        done
        mkdir -p "$go2_output"
        printf -v go2_command '%q ' bash "$(dirname "${BASH_SOURCE[0]}")/start_warehouse.sh" --artifact-dir "$go2_output" "$@"
        printf -v go2_log_quoted '%q' "$go2_log"
        tmux new-session -d -s "$go2_session" "$go2_command > $go2_log_quoted 2>&1"
        echo "Session: $go2_session"
        echo "Log: $go2_log"
        echo "Artifacts: $go2_output"
        ;;
    stop)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux send-keys -t "$go2_session:0.0" C-c
            echo "Requested graceful simulator stop. Cloud machine remains on."
        else
            echo "Simulator session is not running."
        fi
        ;;
    status)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}'
        else
            echo "Simulator session is not running."
        fi
        ;;
    *) echo "Usage: $0 {start [simulator arguments...]|stop|status}"; exit 2 ;;
esac
