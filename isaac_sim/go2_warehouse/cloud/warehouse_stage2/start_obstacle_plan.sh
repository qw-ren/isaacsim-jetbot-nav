#!/usr/bin/env bash
# Reuse the active AMCL/map server; never start another map source.
set -eo pipefail
go2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)";source "$go2_dir/nav_env.sh"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/wait_localization_services.py" --require-active
go2_children=()
go2_cleanup() { trap - EXIT INT TERM;for go2_child in "${go2_children[@]}";do kill -TERM -- "-$go2_child" 2>/dev/null || true;done;wait || true; }
trap go2_cleanup EXIT
trap 'exit 0' INT TERM
setsid "$GO2_CLOUD_ROOT/deps/ros_humble/lib/nav2_planner/planner_server" --ros-args --params-file "${GO2_PLAN_PARAMS:-$go2_dir/nav_obstacles.yaml}" &
go2_children+=("$!")
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/wait_localization_services.py" --nodes planner_server
setsid "$GO2_CLOUD_ROOT/deps/ros_humble/lib/nav2_lifecycle_manager/lifecycle_manager" --ros-args -r __node:=lifecycle_manager_plan --params-file "${GO2_PLAN_PARAMS:-$go2_dir/nav_obstacles.yaml}" -p 'node_names:=[planner_server]' -p autostart:=false &
go2_children+=("$!")
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/activate_lifecycle_manager.py" --manager lifecycle_manager_plan --nodes planner_server
wait -n "${go2_children[@]}"
