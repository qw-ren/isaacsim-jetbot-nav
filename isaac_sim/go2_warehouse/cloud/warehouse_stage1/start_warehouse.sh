#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/cloud_env.sh"
cd "${GO2_CLOUD_ROOT}/work"
go2_kit_args="--/persistent/isaac/asset_root/default=${GO2_ASSET_ROOT} --/persistent/isaac/asset_root/cloud=${GO2_ASSET_ROOT}"
exec "${GO2_ISAAC_ROOT}/python.sh" isaac_go2_ros2.py \
    --enable_cameras --spawn-y -1.0 --spawn-yaw 0.0 --kit_args "$go2_kit_args" \
    env_name=warehouse-shelves sensor.semantic_segmentation=False "$@"
