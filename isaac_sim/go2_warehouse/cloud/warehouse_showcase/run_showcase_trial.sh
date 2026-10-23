#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_showcase_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
for go2_s in go2-lio-nav-task go2-warehouse go2-mainline-task go2-navigation-m1 go2-velocity-guard;do
 if tmux has-session -t "=$go2_s" 2>/dev/null;then echo "Existing $go2_s";exit 1;fi
done
if pgrep -u "$(id -u)" -f '(^|[ /])isaac_go2_ros2[.]py([ ]|$)' >/dev/null;then echo 'Existing simulator';exit 1;fi
[[ ${1:?unique ID} =~ ^[A-Za-z0-9_]+$ ]] || exit 2
python3 "$go2_showcase_dir/prepare_showcase_trial.py" --case "$1" --speed-cap "${2:-1.0}"
go2_case=/home/admin/go2-warehouse/artifacts/moving_$1
printf -v go2_command '%q ' timeout -s INT 1800 bash "$go2_case/source/warehouse_lio/run_nav_trial.sh" __run "$go2_case"
printf -v go2_log '%q' "$go2_case/runner.log"
tmux new-session -d -s go2-lio-nav-task "$go2_command > $go2_log 2>&1"
echo "$go2_case"
