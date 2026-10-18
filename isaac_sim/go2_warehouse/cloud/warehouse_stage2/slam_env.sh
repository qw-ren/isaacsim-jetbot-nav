#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/nav_env.sh"
export LD_LIBRARY_PATH="$GO2_CLOUD_ROOT/nav_stage2/deps/slam_runtime/usr/lib/x86_64-linux-gnu/lapack:$GO2_CLOUD_ROOT/nav_stage2/deps/slam_runtime/usr/lib/x86_64-linux-gnu/blas:$GO2_CLOUD_ROOT/nav_stage2/deps/slam_runtime/usr/lib:$GO2_CLOUD_ROOT/nav_stage2/deps/slam_runtime/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
