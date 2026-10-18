#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- env DISPLAY=:1 XAUTHORITY=/home/admin/.Xauthority bash "$0"
fi
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_script_dir/nav_env.sh"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/home/admin/.Xauthority}"
exec "$GO2_CLOUD_ROOT/deps/ros_humble/lib/rviz2/rviz2" \
    -d "$go2_script_dir/plan_view.rviz" --ros-args -p use_sim_time:=true
