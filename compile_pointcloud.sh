#!/bin/sh
# compile script
workpath=$(cd "$(dirname "$0")" && pwd)

echo "workpath: $workpath"
cmake -B $workpath/build -S $workpath \
&& cmake --build $workpath/build --config Release --target preview_pointcloud -j 4 \
&& echo "== Run $workpath/build/open3d_preview/preview_pointcloud" \
|| echo "== Build failed"