#!/usr/bin/env bash
# Ubuntu 22.04 amd64 only. Download/extract small RViz runtime libraries;
# no apt install, no system or Isaac library replacement.
set -euo pipefail
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
go2_debs="$go2_cloud_root/nav_stage2/deps/rviz_debs"
go2_runtime="$go2_cloud_root/nav_stage2/deps/rviz_runtime"
mkdir -p "$go2_debs" "$go2_runtime"
cd "$go2_debs"
go2_packages=(libassimp5 libdraco4 libminizip1 libyaml-cpp0.7 libignition-math6-6 liborocos-kdl1.5)
go2_versions=('5.2.2~ds0-1' '1.5.2+dfsg-2' '1.1-8build1' '0.7.0+dfsg-8build1' '6.10.0+ds3-7' '1.5.1-2build1')
for go2_idx in "${!go2_packages[@]}"; do
    go2_name="${go2_packages[$go2_idx]}"
    go2_version="${go2_versions[$go2_idx]}"
    go2_found=false
    for go2_deb in "${go2_name}"_*.deb; do
        [[ -f "$go2_deb" ]] || continue
        if [[ $(dpkg-deb -f "$go2_deb" Version) == "$go2_version" ]] && \
           [[ $(dpkg-deb -f "$go2_deb" Architecture) == amd64 ]]; then
            dpkg-deb -x "$go2_deb" "$go2_runtime"
            go2_found=true
            break
        fi
    done
    if [[ "$go2_found" == false ]]; then
        apt-get download "${go2_name}=${go2_version}"
        for go2_deb in "${go2_name}"_*.deb; do
            if [[ $(dpkg-deb -f "$go2_deb" Version) == "$go2_version" ]]; then
                dpkg-deb -x "$go2_deb" "$go2_runtime"
                go2_found=true
                break
            fi
        done
    fi
    [[ "$go2_found" == true ]] || exit 1
done
sha256sum ./*.deb > packages.sha256
echo "Isolated RViz libraries: $go2_runtime/usr/lib/x86_64-linux-gnu"
