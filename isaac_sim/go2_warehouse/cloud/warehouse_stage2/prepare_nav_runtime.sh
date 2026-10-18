#!/usr/bin/env bash
# Download/extract only. Run before starting map/planner nodes.
set -euo pipefail
if [[ $(id -u) -eq 0 ]]; then
    exec runuser -u admin -- bash "$0"
fi
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
if pgrep -u "$(id -u)" -f "$go2_cloud_root/deps/ros_humble/lib/nav2_(map_server/map_server|planner/planner_server)" >/dev/null; then
    echo "Stop the planning session before extracting its libraries."
    exit 1
fi
go2_debs="$go2_cloud_root/nav_stage2/deps/nav_debs"
go2_runtime="$go2_cloud_root/nav_stage2/deps/nav_runtime"
mkdir -p "$go2_debs" "$go2_runtime"
cd "$go2_debs"
go2_packages=(libgraphicsmagick++-q16-12 libgraphicsmagick-q16-3 libde265-0 libheif1)
go2_versions=('1.4+really1.3.38-1ubuntu0.1' '1.4+really1.3.38-1ubuntu0.1' '1.0.8-1ubuntu0.3' '1.12.0-2build1')
for go2_idx in "${!go2_packages[@]}"; do
    go2_package="${go2_packages[$go2_idx]}"
    go2_version="${go2_versions[$go2_idx]}"
    go2_found=false
    for go2_deb in "${go2_package}"_*.deb; do
        [[ -f "$go2_deb" ]] || continue
        if [[ $(dpkg-deb -f "$go2_deb" Version) == "$go2_version" && $(dpkg-deb -f "$go2_deb" Architecture) == amd64 ]]; then
            go2_found=true
            break
        fi
    done
    if [[ "$go2_found" == false ]]; then
        apt-get download "${go2_package}=${go2_version}"
    fi
    go2_found=false
    for go2_deb in "${go2_package}"_*.deb; do
        if [[ $(dpkg-deb -f "$go2_deb" Version) == "$go2_version" && $(dpkg-deb -f "$go2_deb" Architecture) == amd64 ]]; then
            dpkg-deb -x "$go2_deb" "$go2_runtime"
            go2_found=true
            break
        fi
    done
    [[ "$go2_found" == true ]] || exit 1
done
sha256sum ./*.deb > packages.sha256
echo "Isolated map-server libraries: $go2_runtime/usr/lib"
