#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
go2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_session=go2-slam-m2
source "$go2_dir/slam_env.sh"
case "${1:-status}" in
 start)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then echo 'SLAM session exists; refusing duplicate'; exit 1; fi
  go2_nodes=$(bash "$GO2_CLOUD_ROOT/work/cloud/warehouse_stage1/ros2_cloud.sh" node list --no-daemon)
  if [[ "$go2_nodes" == *'/slam_toolbox'* || "$go2_nodes" == *'/amcl'* || "$go2_nodes" == *'/map_server'* ]]; then echo 'Existing mapping/localization node; refusing duplicate'; exit 1; fi
  go2_log="$GO2_CLOUD_ROOT/logs/slam_$(date +%Y%m%d_%H%M%S).log"
  printf -v go2_command '%q ' bash "$go2_dir/start_slam.sh"
  printf -v go2_log_q '%q' "$go2_log"
  tmux new-session -d -s "$go2_session" "$go2_command > $go2_log_q 2>&1"
  echo "Session: $go2_session"; echo "Log: $go2_log";;
 stop)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then tmux send-keys -t "$go2_session:0.0" C-c; echo 'Requested SLAM stop; simulator and cloud machine remain on'; else echo 'SLAM session is not running'; fi;;
 status)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}'; else echo 'SLAM session is not running'; fi;;
 *)exit 2;;
esac
