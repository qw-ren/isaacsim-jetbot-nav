#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_root=/home/admin/go2-warehouse
go2_case=${1:?Captured case directory}
go2_output=${2:?New replay directory}
go2_image=${3:-1}
go2_end=${4:-0}
go2_begin=${5:-3}
go2_imu=${6:-/lio/imu}
[[ "$go2_imu" == /lio/imu || "$go2_imu" == /evaluation/imu_ideal ]] || exit 2
[[ "$go2_image" == 0 || "$go2_image" == 1 ]] || exit 2
test -f "$go2_case/RUNNER_RESULT.json"
test ! -e "$go2_output"
source "$go2_here/../warehouse_stage2/nav_env.sh"
export ROS_DOMAIN_ID=47
export LD_LIBRARY_PATH="$go2_root/nav_stage2/deps/fastlivo_trial_20261022/lib:$go2_root/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH"
exec 9>/tmp/go2-lio-replay.lock
flock -n 9 || { echo 'Another isolated replay is active';exit 1; }
go2_params="${go2_output}_params.yaml"
"$GO2_ISAAC_ROOT/python.sh" "$go2_here/generate_livo_params.py" --contract "$go2_case/simulation/rgb/camera_contract.json" --output "$go2_params" --image-enabled "$go2_image"
go2_pid=
go2_stop() {
 trap - EXIT INT TERM
 if [[ -n "$go2_pid" ]] && kill -0 "$go2_pid" 2>/dev/null;then kill -INT -- "-$go2_pid";fi
 if [[ -n "$go2_pid" ]];then wait "$go2_pid" || true;fi
}
trap go2_stop EXIT
trap 'exit 130' INT TERM
setsid /usr/bin/time -v -o "${go2_output}_resource.txt" "$go2_root/nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping" --ros-args --params-file "$go2_params" -r __node:=fastlivo_recorded_replay -r /tf:=/livo/evaluation_tf -r /tf_static:=/livo/evaluation_tf_static -r /aft_mapped_to_init:=/lio/odometry > "${go2_output}_fastlivo.log" 2>&1 &
go2_pid=$!
"$GO2_ISAAC_ROOT/python.sh" "$go2_here/replay_livo_dataset.py" --database "$go2_case/recording/sensors.sqlite3" --output "$go2_output" --imu-topic "$go2_imu" --rate 1 --begin "$go2_begin" --end "$go2_end"
