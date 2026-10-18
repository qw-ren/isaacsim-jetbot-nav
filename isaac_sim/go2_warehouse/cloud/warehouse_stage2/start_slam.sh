#!/usr/bin/env bash
set -eo pipefail
go2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_dir/slam_env.sh"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/wait_live_sim.py"
"$GO2_ISAAC_ROOT/python.sh" "$go2_dir/check_slam_inputs.py"
exec "$GO2_CLOUD_ROOT/deps/ros_humble/lib/slam_toolbox/async_slam_toolbox_node" --ros-args -r __node:=slam_toolbox --params-file "$go2_dir/slam_mapping.yaml"
