#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- bash "$0"
fi
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$go2_script_dir/nav_env.sh"
exec "$GO2_ISAAC_ROOT/python.sh" "$go2_script_dir/velocity_guard.py" \
    --boxes "${GO2_GUARD_BOXES:-$GO2_CLOUD_ROOT/nav_stage2/maps/warehouse_dev_v2/collision_boxes.json}"
