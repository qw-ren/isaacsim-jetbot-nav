#!/usr/bin/env bash
set -eo pipefail
go2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_dir/nav_env.sh"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/wait_live_sim.py"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/check_slam_inputs.py"
go2_params="$GO2_CLOUD_ROOT/nav_stage2/generated/amcl_$(date +%Y%m%d_%H%M%S).yaml"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/make_localization_params.py" --map "$GO2_SAVED_MAP" --pose "$GO2_INITIAL_POSE" --output "$go2_params"
go2_children=()
go2_cleanup() { trap - EXIT INT TERM; for go2_child in "${go2_children[@]}"; do kill -TERM -- "-$go2_child" 2>/dev/null || true; done; wait || true; }
trap go2_cleanup EXIT
trap 'exit 0' INT TERM
for go2_entry in nav2_map_server/map_server nav2_amcl/amcl; do setsid "$GO2_CLOUD_ROOT/deps/ros_humble/lib/$go2_entry" --ros-args --params-file "$go2_params" & go2_children+=("$!"); done
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/wait_localization_services.py"
setsid "$GO2_CLOUD_ROOT/deps/ros_humble/lib/nav2_lifecycle_manager/lifecycle_manager" --ros-args -r __node:=lifecycle_manager_localization --params-file "$go2_params" -p autostart:=false &
go2_children+=("$!")
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/activate_lifecycle_manager.py" --manager lifecycle_manager_localization --nodes map_server amcl
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/seed_amcl_pose.py" --pose "$GO2_INITIAL_POSE" --report "${go2_params%.yaml}_seed.json"
wait -n "${go2_children[@]}"
