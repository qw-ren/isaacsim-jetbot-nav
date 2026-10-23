#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[[ ${1:-} == start ]] || { echo 'Usage: run_moving_trial.sh start ID A|B|C short|long SELECTION.json';exit 2; }
for go2_s in go2-lio-nav-task go2-warehouse go2-mainline-task;do
 if tmux has-session -t "=$go2_s" 2>/dev/null;then echo "Existing $go2_s";exit 1;fi
done
if pgrep -u "$(id -u)" -f '(^|[ /])isaac_go2_ros2[.]py([ ]|$)' >/dev/null;then echo 'Existing simulator';exit 1;fi
go2_map_args=(--profile "${7:-standard}")
if [[ -n ${6:-} ]];then go2_map_args+=(--map-directory "$6");fi
go2_case=$(python3 "$go2_here/prepare_dynamic_trial.py" --case "${2:?ID}" --kind "${3:?Kind}" --route "${4:?Route}" --selection "${5:?Selection JSON}" "${go2_map_args[@]}")
printf -v go2_command '%q ' timeout -s INT 1800 bash "$go2_case/source/warehouse_lio/run_nav_trial.sh" __run "$go2_case"
printf -v go2_log '%q' "$go2_case/runner.log"
tmux new-session -d -s go2-lio-nav-task "$go2_command > $go2_log 2>&1"
echo "$go2_case"
