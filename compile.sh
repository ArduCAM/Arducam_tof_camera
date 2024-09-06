#!/bin/sh
# compile script
workpath=$(cd "$(dirname "$0")" && pwd)

echo "workpath: $workpath"

if ! cmake -B $workpath/build -S $workpath; then
    echo "== CMake failed"
    exit 1
fi

if cmake --build $workpath/build --config Release --target example -j 4 &&
    cmake --build $workpath/build --config Release --target capture_raw -j 4 &&
    cmake --build $workpath/build --config Release --target preview_depth -j 4 &&
    cmake --build $workpath/build --config Release --target preview_depth_c -j 4; then
    echo "== Build success"
    echo "== Run $workpath/build/example/cpp/example"
else
    echo "== Retry build without -j 4"
    if cmake --build $workpath/build --config Release --target example &&
        cmake --build $workpath/build --config Release --target capture_raw &&
        cmake --build $workpath/build --config Release --target preview_depth &&
        cmake --build $workpath/build --config Release --target preview_depth_c; then
        echo "== Build success"
        echo "== Run $workpath/build/example/cpp/example"
    else
        echo "== Build failed"
    fi
fi
