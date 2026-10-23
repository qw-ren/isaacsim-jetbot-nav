#!/usr/bin/env bash
# Offline replay only, in a separate ROS domain. No simulator or velocity gate.
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_source=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_root=/home/admin/go2-warehouse
go2_database=${1:?Provide capture SQLite path}
go2_output=${2:?Provide new replay directory}
go2_imu=${3:?Provide /lio/imu or /evaluation/imu_ideal}
test -f "$go2_database"
test ! -e "$go2_output"
source "$go2_source/../warehouse_stage2/nav_env.sh"
export ROS_DOMAIN_ID=47
exec 9>/tmp/go2-lio-replay.lock
flock -n 9 || { echo 'Another replay is already running';exit 1; }
go2_pid=
go2_stop() {
 trap - EXIT INT TERM
 if [[ -n "$go2_pid" ]] && kill -0 "$go2_pid" 2>/dev/null;then kill -INT "$go2_pid";fi
 if [[ -n "$go2_pid" ]];then wait "$go2_pid" || true;fi
}
trap go2_stop EXIT
trap 'exit 130' INT TERM
env LD_LIBRARY_PATH="$go2_root/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH" "$go2_root/nav_stage2/deps/fastlio_reliable_20261021/bin/fastlio_mapping" --ros-args --params-file "$go2_source/fastlio_frame20ms_trial.yaml" -p reliable_lidar:=true -r __node:=fastlio_recorded_replay -r /tf:=/lio/evaluation_tf -r /tf_static:=/lio/evaluation_tf_static -r /Odometry:=/lio/odometry > "${go2_output}_fastlio.log" 2>&1 &
go2_pid=$!
"$GO2_ISAAC_ROOT/python.sh" "$go2_source/replay_lio_dataset.py" --database "$go2_database" --output "$go2_output" --imu-topic "$go2_imu" --rate 1
