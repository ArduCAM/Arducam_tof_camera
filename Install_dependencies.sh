#!/bin/sh

FIND_FILE=""
HAS_DTOVERLAY="true"
if [ -f "/boot/firmware/config.txt" ]; then # Bookworm
    FIND_FILE="/boot/firmware/config.txt"
    if [ $(grep -c -e '^dtoverlay=arducam-pivariety$' $FIND_FILE) -lt '1' ]; then
        HAS_DTOVERLAY="false"
    fi
elif [ -f "/boot/config.txt" ]; then # Bullseye and earlier
    FIND_FILE="/boot/config.txt"
    if [ $(grep -c -e '^dtoverlay=arducam-pivariety$' $FIND_FILE) -lt '1' ]; then
        HAS_DTOVERLAY="false"
    fi
fi

if [ "$FIND_FILE" = "" ]; then
    echo -e "\033[31m[ERR] No config.txt file found."
    exit 1
fi

if [ "$HAS_DTOVERLAY" = "false" ]; then
    echo -e "\033[31m[WARN]\033[0m dtoverlay=arducam-pivariety not found in $FIND_FILE."
    # remove all line which has dtoverlay=arducam-pivariety
    sudo sed -i '/dtoverlay=arducam-pivariety/d' $FIND_FILE
    echo "dtoverlay=arducam-pivariety" | sudo tee -a $FIND_FILE
    # if "Raspberry Pi 5" in `cat /sys/firmware/devicetree/base/model`
    if [ -f /sys/firmware/devicetree/base/model ]; then
        if grep -q "Raspberry Pi 5" /sys/firmware/devicetree/base/model; then
            echo "dtoverlay=arducam-pivariety,cam0" | sudo tee -a $FIND_FILE
        fi
    fi
    echo -e "\033[32m[INFO]\033[0m dtoverlay=arducam-pivariety added to $FIND_FILE."
fi

if [ $(grep -c "camera_auto_detect=1" $FIND_FILE) -ne '0' ]; then
    sudo sed -i "s/\(^camera_auto_detect=1\)/camera_auto_detect=0/" $FIND_FILE
fi
if [ $(grep -c "camera_auto_detect=0" $FIND_FILE) -lt '1' ]; then
    sudo bash -c "echo camera_auto_detect=0 >> $FIND_FILE"
fi

sudo apt update
if ! sudo apt-get install -y cmake curl libopencv-dev python3-pip python3-opencv python3-numpy >/dev/null 2>&1; then
    echo -e "\033[31m[ERR]\033[0m Failed to install dependencies."
    echo -e "\033[31m[ERR]\033[0m Please check your network connection."
    exit 1
fi

if [ $(dpkg -l | grep -c arducam-tof-sdk-dev) -lt 1 ]; then
    echo "Add Arducam_ppa repositories."
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
fi

# install dependency
sudo apt update
if ! sudo apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev >/dev/null 2>&1; then
    echo -e "\033[31m[ERR]\033[0m Failed to install tof sdk."
    echo -e "\033[31m[ERR]\033[0m Please check your network connection."
    exit 1
fi
if ! sudo python -m pip install ArducamDepthCamera >/dev/null 2>&1; then
    if ! sudo python -m pip install ArducamDepthCamera --break-system-packages >/dev/null 2>&1; then
        echo -e "\033[31m[ERR]\033[0m Failed to install ArducamDepthCamera."
    fi
fi
# python -m pip install ArducamDepthCamera opencv-python "numpy<2.0.0"
echo -e "\033[32m[INFO]\033[0m If you want to install for python venv."
echo -e "  please run: python -m pip install ArducamDepthCamera opencv-python \"numpy<2.0.0\""

echo "reboot now? (y/n):"
read -r USER_INPUT
case $USER_INPUT in
'y' | 'Y')
    echo "reboot"
    sudo reboot
    ;;
*)
    echo "cancel"
    echo "The script settings will only take effect after restarting. Please restart yourself later."
    exit 1
    ;;
esac
