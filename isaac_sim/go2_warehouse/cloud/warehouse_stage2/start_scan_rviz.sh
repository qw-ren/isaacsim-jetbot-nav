#!/usr/bin/env bash
set -eo pipefail
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- env DISPLAY=:1 XAUTHORITY=/home/admin/.Xauthority bash "$0"
fi
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_cloud_root/work/cloud/warehouse_stage1/cloud_env.sh"
export LD_LIBRARY_PATH="$go2_cloud_root/nav_stage2/deps/rviz_runtime/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/home/admin/.Xauthority}"
exec "$go2_cloud_root/deps/ros_humble/lib/rviz2/rviz2" \
    -d "$go2_script_dir/scan_view.rviz" --ros-args -p use_sim_time:=true
