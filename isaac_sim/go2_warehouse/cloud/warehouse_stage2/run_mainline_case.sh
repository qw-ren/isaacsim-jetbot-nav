#!/usr/bin/env bash
# Run on the cloud. Existing singleton managers own the actual simulator/nodes.
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
go2_source=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_root=/home/admin/go2-warehouse
go2_session=go2-mainline-task
go2_action=${1:-help}
case "$go2_action" in
 help|--help|-h)
  echo 'Usage: run_mainline_case.sh start {branch|closed} [unique_case_id] | status | stop'
  exit 0;;
 status)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then
   tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_start_command}'
  else echo 'Mainline supervisor is not running'; fi
  bash "$go2_root/work/cloud/warehouse_stage1/manage_warehouse.sh" status
  exit 0;;
 stop)
  if tmux has-session -t "=$go2_session" 2>/dev/null; then
   tmux send-keys -t "$go2_session:0.0" C-c
   echo 'Requested task stop; observer cleanup and singleton-manager shutdown will follow.'
  else echo 'Mainline supervisor is not running'; fi
  exit 0;;
 start)
  go2_case=${2:-}
  [[ "$go2_case" == branch || "$go2_case" == closed ]] || { echo 'Choose branch or closed'; exit 2; }
  go2_id=${3:-$(date +%Y%m%d_%H%M%S)_$go2_case}
  [[ "$go2_id" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Case ID may contain only letters, digits, _ and -'; exit 2; }
  for go2_check in "$go2_session" go2-warehouse go2-scan-m1 go2-localization-m2 go2-obstacle-plan-m3 go2-velocity-guard go2-navigation-m1; do
   if tmux has-session -t "=$go2_check" 2>/dev/null; then echo "Existing session $go2_check; refusing duplicate startup"; exit 1; fi
  done
  if pgrep -u "$(id -u)" -f '(^|[ /])isaac_go2_ros2[.]py([ ]|$)' >/dev/null; then echo 'Existing simulator process; inspect before starting'; exit 1; fi
  if [[ "$go2_case" == closed && ! -d "$go2_root/motion_live_closed_20261019" ]]; then
   python3 "$go2_source/prepare_closed_route_copy.py"
  fi
  go2_artifacts="$go2_root/artifacts/mainline_$go2_id"
  python3 "$go2_source/prepare_mainline_case.py" --case "$go2_case" --output "$go2_artifacts"
  printf -v go2_command '%q ' bash "$go2_artifacts/source/run_mainline_case.sh" __run "$go2_artifacts"
  printf -v go2_log '%q' "$go2_artifacts/runner.log"
  tmux new-session -d -s "$go2_session" "$go2_command > $go2_log 2>&1"
  echo "Artifacts and log: $go2_artifacts"
  exit 0;;
 __run)
  go2_artifacts=$2
  go2_source="$go2_artifacts/source"
  mapfile -t go2_config < <(python3 - "$go2_artifacts/CASE.json" <<'PY'
import json,sys
v=json.load(open(sys.argv[1]));print(v['case']);print(v['runtime']);print(v['map'])
PY
)
  go2_case=${go2_config[0]};go2_runtime="$go2_root/${go2_config[1]}";go2_map=${go2_config[2]}
  go2_stops=()
  go2_finish() {
   go2_exit=$1
   trap - EXIT INT TERM
   for ((go2_idx=${#go2_stops[@]}-1;go2_idx>=0;go2_idx--)); do bash "${go2_stops[go2_idx]}" stop || true; done
   # Graceful shutdown can finish after Ctrl-C acknowledgement.
   for ((go2_idx=0;go2_idx<75;go2_idx++)); do
    if ! tmux has-session -t '=go2-warehouse' 2>/dev/null; then break; fi
    sleep .2
   done
   python3 - "$go2_artifacts" "$go2_exit" <<'PY'
from pathlib import Path
import datetime,json,sys,subprocess
p=Path(sys.argv[1]);result=dict(exit_code=int(sys.argv[2]),time=datetime.datetime.now().astimezone().isoformat(),power_operation=False)
sessions=subprocess.run(['tmux','list-sessions','-F','#{session_name}'],capture_output=True,text=True).stdout.splitlines()
result['remaining_component_sessions']=[s for s in sessions if s!='go2-mainline-task']
(p/'RUNNER_RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
PY
   exit "$go2_exit"
  }
  trap 'go2_finish $?' EXIT
  trap 'exit 130' INT TERM
  go2_start() { bash "$1" start;go2_stops+=("$1"); }
  bash "$go2_runtime/cloud/warehouse_stage1/manage_warehouse.sh" start --spawn-x 1 --spawn-y 0 --run-seconds 900 --artifact-dir "$go2_artifacts/simulation" freq=50 ++publish_map_to_odom=False
  go2_stops+=("$go2_runtime/cloud/warehouse_stage1/manage_warehouse.sh")
  GO2_SCAN_PARAMS="$go2_source/scan_converter_live.yaml" go2_start "$go2_source/manage_scan.sh"
  GO2_SAVED_MAP="$go2_map" GO2_INITIAL_POSE="$go2_artifacts/initial_pose.json" go2_start "$go2_source/manage_localization.sh"
  GO2_PLAN_PARAMS="$go2_source/nav_obstacle_plan_safe.yaml" go2_start "$go2_source/manage_obstacle_plan.sh"
  source "$go2_source/nav_env.sh"
  if [[ "$go2_case" == branch ]]; then
   "$GO2_ISAAC_ROOT/python.sh" "$go2_source/check_branch_plan.py" --output "$go2_artifacts/planning" --fixture "$go2_artifacts/simulation" --map "$go2_map"
  fi
  GO2_GUARD_BOXES="$go2_artifacts/safety_boxes.json" go2_start "$go2_source/manage_velocity_guard.sh"
  GO2_NAV_PARAMS="$go2_source/nav_obstacle_region_arrival.yaml" GO2_NAV_BT="$go2_source/navigate_valid_path.xml" go2_start "$go2_source/manage_navigation.sh"
  if [[ "$go2_case" == closed ]]; then
   "$GO2_ISAAC_ROOT/python.sh" "$go2_source/check_closed_route_task.py" --output "$go2_artifacts/task" --fixture "$go2_artifacts/simulation" --map "$go2_map"
  else
   "$GO2_ISAAC_ROOT/python.sh" "$go2_source/check_live_branch_navigation.py" --output "$go2_artifacts/navigation" --fixture "$go2_artifacts/simulation" --map "$go2_map" --boxes "$go2_artifacts/safety_boxes.json" --forward-goal 5.5
  fi
  exit 0;;
 *) echo 'Unknown action; use --help';exit 2;;
esac
