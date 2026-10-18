#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- bash "$0" "$@"
fi
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_session=go2-scan-m1
case "${1:-status}" in
    start)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            echo "Scan session already exists; use status or stop."
            exit 1
        fi
        source "$go2_cloud_root/work/cloud/warehouse_stage1/cloud_env.sh"
        # Refuse an existing /scan publisher, including one outside this manager.
        if go2_info=$(bash "$go2_cloud_root/work/cloud/warehouse_stage1/ros2_cloud.sh" topic info /scan --no-daemon 2>&1); then
            if [[ "$go2_info" != *"Publisher count: 0"* ]]; then
                echo "Existing /scan publisher; refusing duplicate conversion."
                exit 1
            fi
        elif [[ "$go2_info" != *"Unknown topic"* ]]; then
            echo "$go2_info"
            exit 1
        fi
        go2_log="$go2_cloud_root/logs/scan_$(date +%Y%m%d_%H%M%S).log"
        printf -v go2_command '%q ' env "GO2_SCAN_PARAMS=${GO2_SCAN_PARAMS:-$go2_script_dir/scan_converter.yaml}" bash "$go2_script_dir/start_scan.sh"
        printf -v go2_log_quoted '%q' "$go2_log"
        tmux new-session -d -s "$go2_session" "$go2_command > $go2_log_quoted 2>&1"
        echo "Session: $go2_session"
        echo "Log: $go2_log"
        ;;
    status)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}'
        else
            echo "Scan session is not running."
        fi
        ;;
    stop)
        if tmux has-session -t "=$go2_session" 2>/dev/null; then
            tmux send-keys -t "$go2_session:0.0" C-c
            echo "Requested scan-only stop; simulator remains running."
        else
            echo "Scan session is not running."
        fi
        ;;
    *) echo "Usage: $0 {start|status|stop}"; exit 2 ;;
esac
