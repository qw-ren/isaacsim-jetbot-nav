#!/usr/bin/env bash
set -eo pipefail
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${go2_cloud_root}/work/cloud/warehouse_stage1/cloud_env.sh"
"$GO2_ISAAC_ROOT/python.sh" "$go2_script_dir/wait_live_sim.py"
go2_params="${GO2_SCAN_PARAMS:-$go2_script_dir/scan_converter.yaml}"
test -f "$go2_params"
go2_children=()
go2_cleanup() {
    trap - EXIT INT TERM
    for go2_child in "${go2_children[@]}"; do
        if kill -0 "$go2_child" 2>/dev/null; then
            kill -TERM -- "-$go2_child" 2>/dev/null || true
        fi
    done
    wait || true
}
trap go2_cleanup EXIT
trap 'exit 0' INT TERM
setsid "$GO2_ISAAC_ROOT/python.sh" "$go2_script_dir/scan_ground_frame.py" \
    --ros-args --params-file "$go2_params" &
go2_children+=("$!")
setsid "$go2_cloud_root/deps/ros_humble/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node" \
    --ros-args -r __node:=go2_cloud_to_scan --params-file "$go2_params" \
    -r cloud_in:=/unitree_go2/lidar/point_cloud -r scan:=/scan &
go2_children+=("$!")
wait -n "${go2_children[@]}"
