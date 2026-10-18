#!/usr/bin/env bash
set -eo pipefail
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_script_dir/nav_env.sh"
/home/admin/isaacsim/python.sh "$go2_script_dir/wait_navigation_ready.py"
go2_children=()
go2_cleanup() {
 trap - EXIT INT TERM
 for go2_child in "${go2_children[@]}"; do
  kill -TERM -- "-$go2_child" 2>/dev/null || true
 done
 wait || true
}
trap go2_cleanup EXIT
trap 'exit 0' INT TERM
go2_ros_lib="$GO2_CLOUD_ROOT/deps/ros_humble/lib"
go2_params="${GO2_NAV_PARAMS:-$go2_script_dir/nav_motion.yaml}"
test -f "$go2_params"
setsid "$go2_ros_lib/nav2_controller/controller_server" --ros-args --params-file "$go2_params" -r cmd_vel:=/nav_m1/cmd_vel &
go2_children+=("$!")
setsid "$go2_ros_lib/nav2_bt_navigator/bt_navigator" --ros-args --params-file "$go2_params" -p "default_nav_to_pose_bt_xml:=${GO2_NAV_BT:-$go2_script_dir/navigate_to_pose.xml}" -p "default_nav_through_poses_bt_xml:=$go2_script_dir/navigate_through_poses.xml" &
go2_children+=("$!")
setsid "$go2_ros_lib/nav2_lifecycle_manager/lifecycle_manager" --ros-args -r __node:=lifecycle_manager_navigation --params-file "$go2_params" -p autostart:=false &
go2_children+=("$!")
"$GO2_ISAAC_ROOT/python.sh" "$go2_script_dir/activate_lifecycle_manager.py" --manager lifecycle_manager_navigation --nodes controller_server bt_navigator
wait -n "${go2_children[@]}"
