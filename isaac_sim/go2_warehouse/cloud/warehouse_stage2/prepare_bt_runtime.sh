#!/usr/bin/env bash
# Extract the single missing BT coroutine runtime into stage2, without apt install.
set -eo pipefail
go2_cloud_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
go2_debs="$go2_cloud_root/nav_stage2/deps/bt_debs"
go2_runtime="$go2_cloud_root/nav_stage2/deps/bt_runtime"
mkdir -p "$go2_debs" "$go2_runtime"
cd "$go2_debs"
apt-get download libboost-context1.74.0=1.74.0-14ubuntu3
sha256sum ./*.deb > packages.sha256
for go2_package in ./*.deb; do dpkg-deb -x "$go2_package" "$go2_runtime"; done
