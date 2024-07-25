#!/bin/sh
# compile script
workpath=$(cd "$(dirname "$0")" && pwd)

echo "workpath: $workpath"
cmake -B $workpath/build -S $workpath \
&& cmake --build $workpath/build --config Release --target example -j 4 \
&& cmake --build $workpath/build --config Release --target preview_depth_c -j 4 \
&& cmake --build $workpath/build --config Release --target preview_depth -j 4 \
&& cmake --build $workpath/build --config Release --target capture_raw -j 4 \
&& echo "== Run $workpath/build/example/cpp/example" \
|| echo "== Build failed"