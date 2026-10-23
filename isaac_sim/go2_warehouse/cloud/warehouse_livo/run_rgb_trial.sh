#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} != start ]];then echo 'Usage: run_rgb_trial.sh start UNIQUE_ID [short|long]';exit 2;fi
for go2_s in go2-lio-nav-task go2-warehouse go2-mainline-task;do
 if tmux has-session -t "=$go2_s" 2>/dev/null;then echo "Existing session $go2_s";exit 1;fi
done
if pgrep -u "$(id -u)" -f '(^|[ /])isaac_go2_ros2[.]py([ ]|$)' >/dev/null;then echo 'Existing simulator';exit 1;fi
go2_case=$(python3 "$go2_here/prepare_rgb_trial.py" --case "${2:?Unique case}" --route "${3:-short}" --backend "${4:-lio}")
printf -v go2_command '%q ' timeout -s INT 1800 bash "$go2_case/source/warehouse_lio/run_nav_trial.sh" __run "$go2_case"
printf -v go2_log '%q' "$go2_case/runner.log"
tmux new-session -d -s go2-lio-nav-task "$go2_command > $go2_log 2>&1"
echo "$go2_case"
