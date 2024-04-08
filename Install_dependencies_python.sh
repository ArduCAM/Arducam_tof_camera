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

if [ `grep -c "camera_auto_detect=1" $FIND_FILE` -ne '0' ];then
    sudo sed -i "s/\(^camera_auto_detect=1\)/camera_auto_detect=0/" $FIND_FILE
fi
if [ `grep -c "camera_auto_detect=0" $FIND_FILE` -lt '1' ];then
    sudo bash -c "echo camera_auto_detect=0 >> $FIND_FILE"
fi

sudo apt update
sudo apt install -y libopencv-dev
sudo apt install -y libcblas-dev
sudo apt install -y libhdf5-dev
sudo apt install -y libhdf5-serial-dev
sudo apt install -y libatlas-base-dev
sudo apt install -y libjasper-dev 
sudo apt install -y libqtgui4 
sudo apt install -y libqt4-test

sudo pip3 install opencv-python ArducamDepthCamera
sudo pip3 install numpy --upgrade

echo "reboot now?(y/n):"
read -r USER_INPUT
case $USER_INPUT in
'y'|'Y')
    echo "reboot"
    sudo reboot
;;
*)
    echo "cancel"
    echo "The script settings will only take effect after restarting, please restart yourself later."
    exit 1
;;
esac
