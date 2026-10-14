#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- bash "$0" "$@"
fi
source "$(dirname "${BASH_SOURCE[0]}")/cloud_env.sh"
exec "${GO2_ISAAC_ROOT}/python.sh" "${GO2_CLOUD_ROOT}/deps/ros_humble/bin/ros2" "$@"
