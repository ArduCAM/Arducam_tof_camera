#!/bin/sh

FIND_FILE=""
if [ -f "/boot/firmware/config.txt" ]; then  # Bookworm
    FIND_FILE="/boot/firmware/config.txt"
    if [ $(grep -c "dtoverlay=arducam-pivariety" $FIND_FILE) -lt '1' ];then
        echo "dtoverlay=arducam-pivariety" | sudo tee -a $FIND_FILE
    fi
elif [ -f "/boot/config.txt" ]; then         # Bullseye and earlier
    FIND_FILE="/boot/config.txt"
    if [ $(grep -c "dtoverlay=arducam-pivariety" $FIND_FILE) -lt '1' ];then
        echo "dtoverlay=arducam-pivariety" | sudo tee -a $FIND_FILE
    fi
fi

if [ "$FIND_FILE" = "" ]; then
    echo "No config.txt file found."
    exit 1
fi

if [ $(grep -c "camera_auto_detect=1" $FIND_FILE) -ne '0' ];then
    sudo sed -i "s/\(^camera_auto_detect=1\)/camera_auto_detect=0/" $FIND_FILE
fi
if [ $(grep -c "camera_auto_detect=0" $FIND_FILE) -lt '1' ];then
    sudo bash -c "echo camera_auto_detect=0 >> $FIND_FILE"
fi

if [ $(dpkg -l | grep -c arducam-tof-sdk-dev) -lt 1 ]; then
    echo "Add Arducam_ppa repositories."
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
fi

# install dependency
sudo apt update
sudo apt install -y arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
sudo apt-get install cmake -y
sudo apt install libopencv-dev -y

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
