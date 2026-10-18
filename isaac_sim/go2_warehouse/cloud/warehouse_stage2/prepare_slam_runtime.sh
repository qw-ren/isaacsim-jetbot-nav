#!/usr/bin/env bash
set -euo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0"; fi
go2_root="${GO2_CLOUD_ROOT:-/home/admin/go2-warehouse}"
go2_debs="$go2_root/nav_stage2/deps/slam_debs"
go2_runtime="$go2_root/nav_stage2/deps/slam_runtime"
mkdir -p "$go2_debs" "$go2_runtime"
cd "$go2_debs"
for go2_spec in libboost-serialization1.74.0=1.74.0-14ubuntu3 libtbb12=2021.5.0-7ubuntu2 libtbbmalloc2=2021.5.0-7ubuntu2 libceres2=2.0.0+dfsg1-5 libgoogle-glog0v5=0.5.0+really0.4.0-2 libgflags2.2=2.2.2-2 libspqr2=1:5.10.1+dfsg-4build1 libcholmod3=1:5.10.1+dfsg-4build1 libcxsparse3=1:5.10.1+dfsg-4build1 libamd2=1:5.10.1+dfsg-4build1 libcamd2=1:5.10.1+dfsg-4build1 libccolamd2=1:5.10.1+dfsg-4build1 libmetis5=5.1.0.dfsg-7build2 liblapack3=3.10.0-2ubuntu1 libblas3=3.10.0-2ubuntu1 libgfortran5=12-20220319-1ubuntu1; do
    go2_pkg="${go2_spec%%=*}"; go2_ver="${go2_spec#*=}"; go2_found=false
    for go2_file in "${go2_pkg}"_*.deb; do
        [[ -f "$go2_file" ]] || continue
        if [[ $(dpkg-deb -f "$go2_file" Version) == "$go2_ver" ]]; then go2_found=true; break; fi
    done
    if [[ "$go2_found" == false ]]; then apt-get download "$go2_spec"; fi
    for go2_file in "${go2_pkg}"_*.deb; do
        [[ -f "$go2_file" ]] || continue
        if [[ $(dpkg-deb -f "$go2_file" Version) == "$go2_ver" && $(dpkg-deb -f "$go2_file" Architecture) == amd64 ]]; then dpkg-deb -x "$go2_file" "$go2_runtime"; break; fi
    done
done
sha256sum ./*.deb > packages.sha256
