#/bin/bash
# if "Radxa ROCK" in `cat /sys/firmware/devicetree/base/model`
if [ -f /sys/firmware/devicetree/base/model ]; then
    if grep -q "Radxa ROCK" /sys/firmware/devicetree/base/model; then
        echo 0 0 0 0 | sudo tee /sys/devices/platform/rkcif-mipi-lvds2/compact_test
    fi
fi
