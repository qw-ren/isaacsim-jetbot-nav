#!/usr/bin/env bash
# Source on the cloud machine before simulation or ROS commands.
export GO2_CLOUD_ROOT="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
export GO2_ISAAC_ROOT="${GO2_ISAAC_ROOT:-/home/admin/isaacsim}"
export GO2_ASSET_ROOT="${GO2_ASSET_ROOT:-/home/admin/isaacsim_assets/Assets/Isaac/4.5}"
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
source "${GO2_CLOUD_ROOT}/deps/ros_humble/local_setup.bash"
export PYTHONPATH="${GO2_CLOUD_ROOT}/deps/python:${PYTHONPATH:-}"
for go2_package in isaaclab isaaclab_assets isaaclab_rl isaaclab_tasks; do
    export PYTHONPATH="${GO2_CLOUD_ROOT}/deps/IsaacLab/source/${go2_package}:${PYTHONPATH}"
done
export ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-41}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export XDG_CACHE_HOME="${GO2_CLOUD_ROOT}/cache/xdg"
export TORCH_HOME="${GO2_CLOUD_ROOT}/cache/torch"
export HF_HOME="${GO2_CLOUD_ROOT}/cache/huggingface"
export PIP_CACHE_DIR="${GO2_CLOUD_ROOT}/cache/pip"
export TMPDIR="${GO2_CLOUD_ROOT}/cache/tmp"
export ROS_LOG_DIR="${GO2_CLOUD_ROOT}/logs/ros"
mkdir -p "$XDG_CACHE_HOME" "$TORCH_HOME" "$HF_HOME" "$PIP_CACHE_DIR" "$TMPDIR" "$ROS_LOG_DIR"
