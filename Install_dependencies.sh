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

if [ $(dpkg -l | grep -c arducam) -lt 1 ]; then
    echo "Add Arducam_ppa repositories."
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
fi

# install dependency
sudo apt update
sudo apt install -y arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
sudo apt-get install cmake -y
sudo apt install libopencv-dev -y

modfiy_config

# sudo apt-get update

echo "reboot now?(y/n):"
read -r USER_INPUT
case $USER_INPUT in
'y' | 'Y')
    echo "reboot"
    sudo reboot
    ;;
*)
    echo "cancel"
    echo "The script settings will only take effect after restarting, please restart yourself later."
    exit 1
    ;;
esac
