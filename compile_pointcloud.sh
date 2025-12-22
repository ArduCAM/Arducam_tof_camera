#!/bin/sh
# compile script
workpath=$(cd "$(dirname "$0")" && pwd)

echo "workpath: $workpath"

if ! cmake -B "$workpath/build" -S "$workpath"; then
    echo "== CMake failed"
    exit 1
fi

build_dir="$workpath/build"
if [ ! -f "$build_dir/CMakeCache.txt" ]; then
    # for old version compatibility
    rm -f "$workpath/CMakeCache.txt"
    mkdir -p "$workpath/build"
    cd "$workpath/build" || exit 1
    cmake "$workpath"
fi

if cmake --build "$build_dir" --config Release --target preview_pointcloud -j 4; then
    echo "== Build success"
    echo "== Run $build_dir/open3d_preview/preview_pointcloud"
else
    echo "== Retry build without -j 4"
    if cmake --build "$build_dir" --config Release --target preview_pointcloud; then
        echo "== Build success"
        echo "== Run $build_dir/open3d_preview/preview_pointcloud"
    else
        echo "== Build failed"
    fi
fi
