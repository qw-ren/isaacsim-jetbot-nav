#!/usr/bin/env bash
# Source for planning-only nodes; never alters the existing simulator process.
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
source "$go2_cloud_root/work/cloud/warehouse_stage1/cloud_env.sh"
export LD_LIBRARY_PATH="$go2_cloud_root/nav_stage2/deps/bt_runtime/usr/lib/x86_64-linux-gnu:$go2_cloud_root/nav_stage2/deps/nav_runtime/usr/lib:$go2_cloud_root/nav_stage2/deps/nav_runtime/usr/lib/x86_64-linux-gnu:$go2_cloud_root/nav_stage2/deps/rviz_runtime/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
