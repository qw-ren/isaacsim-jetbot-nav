#!/usr/bin/env bash
# Start only map, global costmap, planner and lifecycle manager. No controller.
set -eo pipefail
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_script_dir/nav_env.sh"
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
go2_ros_lib="$GO2_CLOUD_ROOT/deps/ros_humble/lib"
go2_params="$go2_script_dir/nav_plan.yaml"
go2_map="${GO2_DEV_MAP:-$GO2_CLOUD_ROOT/nav_stage2/maps/warehouse_dev_v2/map.yaml}"
test -f "$go2_map"
setsid "$go2_ros_lib/nav2_map_server/map_server" --ros-args --params-file "$go2_params" \
    -p "yaml_filename:=$go2_map" &
go2_children+=("$!")
setsid "$go2_ros_lib/nav2_planner/planner_server" --ros-args --params-file "$go2_params" &
go2_children+=("$!")
setsid "$go2_ros_lib/nav2_lifecycle_manager/lifecycle_manager" --ros-args \
    -r __node:=lifecycle_manager_plan --params-file "$go2_params" &
go2_children+=("$!")
wait -n "${go2_children[@]}"
