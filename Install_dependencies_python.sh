#!/bin/sh

modfiy_config() {
    # modfiy config
    sudo sed -i "s/\(^camera_auto_detect=*\)/#\1/" /boot/config.txt
    sudo bash -c 'echo camera_auto_detect=0 >> /boot/config.txt'
    sudo sed -i "s/\(^dtoverlay=*\)/#\1/" /boot/config.txt
    sudo bash -c 'echo dtoverlay=vc4-fkms-v3d >> /boot/config.txt'
    sudo bash -c 'echo dtoverlay=arducam,media-controller=0 >> /boot/config.txt'
}

if [ $(lsmod | grep -c arducam_pivariety) -ge 5 ]; then
    echo "Arducam tof camera driver already installed!"
else
    wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
    chmod +x install_pivariety_pkgs.sh
    ./install_pivariety_pkgs.sh -p kernel_driver
fi

sudo apt update
sudo apt install libopencv-dev -y
sudo apt-get -y install libcblas-dev
sudo apt-get -y install libhdf5-dev
sudo apt-get -y install libhdf5-serial-dev
sudo apt-get -y install libatlas-base-dev
sudo apt-get -y install libjasper-dev 
sudo apt-get -y install libqtgui4 
sudo apt-get -y install libqt4-test

sudo pip3 install opencv-python ArduCamDepthCamera
sudo pip3 numpy --upgrade

modfiy_config

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
