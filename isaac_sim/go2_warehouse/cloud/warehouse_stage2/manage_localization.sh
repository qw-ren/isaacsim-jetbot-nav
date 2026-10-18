#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
go2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)";source "$go2_dir/nav_env.sh";go2_session=go2-localization-m2
case "${1:-status}" in
 start)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then echo 'Localization already exists';exit 1;fi
  go2_nodes=$(bash "$GO2_CLOUD_ROOT/work/cloud/warehouse_stage1/ros2_cloud.sh" node list --no-daemon)
  if [[ "$go2_nodes" == *'/slam_toolbox'* || "$go2_nodes" == *'/amcl'* || "$go2_nodes" == *'/map_server'* ]]; then echo 'Existing map/localization source; refusing duplicate';exit 1;fi
  go2_map="${GO2_SAVED_MAP:-$GO2_CLOUD_ROOT/nav_stage2/maps/slam_m2_20261015_v1/map_nav.yaml}";go2_pose="${GO2_INITIAL_POSE:-$GO2_CLOUD_ROOT/artifacts/m2_slam_20261015/localization_initial_pose.json}"
  test -f "$go2_map";test -f "$go2_pose"
  go2_log="$GO2_CLOUD_ROOT/logs/localization_$(date +%Y%m%d_%H%M%S).log"
  printf -v go2_cmd '%q ' env "GO2_SAVED_MAP=$go2_map" "GO2_INITIAL_POSE=$go2_pose" bash "$go2_dir/start_localization.sh";printf -v go2_log_q '%q' "$go2_log"
  tmux new-session -d -s "$go2_session" "$go2_cmd > $go2_log_q 2>&1";echo "Session: $go2_session";echo "Log: $go2_log";;
 stop) if tmux has-session -t "=$go2_session" 2>/dev/null;then tmux send-keys -t "$go2_session:0.0" C-c;echo 'Requested localization stop';else echo 'Localization is not running';fi;;
 status) if tmux has-session -t "=$go2_session" 2>/dev/null;then tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}';else echo 'Localization is not running';fi;;
 *)exit 2;;
esac
