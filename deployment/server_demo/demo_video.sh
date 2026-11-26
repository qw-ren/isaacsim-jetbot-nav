#!/usr/bin/env bash
set -euo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
ROOT=/home/admin/atongmu_deploy_20261120
id=${1:?usage: demo_video.sh NEW_CASE_ID [record|gui]}
mode=${2:-record}
[[ "$id" =~ ^[A-Za-z0-9_]+$ && ( "$mode" == record || "$mode" == gui ) ]]
bash "$ROOT/start_demo.sh" "$id" "$mode"
case_dir=/home/admin/jetbot-warehouse/artifacts/$id
status=0
ROOT="$ROOT" case_dir="$case_dir" bash -c '
set -eo pipefail
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
/home/admin/venvs/atongmu_ros/bin/python "$ROOT/demo_tools/capture_route.py" --case "$case_dir"
' > "$ROOT/logs/$id.capture.log" 2>&1 &
capture_pid=$!
python3 "$ROOT/demo_tools/run_navigation_demo.py" --case "$case_dir" || status=$?
wait "$capture_pid" || status=1
/home/admin/venvs/atongmu_ros/bin/python "$ROOT/demo_tools/summarize_route.py" --case "$case_dir" || status=1
if [[ "$mode" == record || $status -ne 0 ]]; then
    bash "$ROOT/stop_demo.sh"
    for attempt in $(seq 1 90); do
        [[ -f "$case_dir/RUNNER_RESULT.json" ]] && break
        sleep 1
    done
    [[ -f "$case_dir/RUNNER_RESULT.json" ]]
    if [[ "$mode" == record ]]; then
        python3 "$ROOT/demo_tools/encode_observer_video.py" --case "$case_dir" --speed 1
    fi
fi
exit "$status"
