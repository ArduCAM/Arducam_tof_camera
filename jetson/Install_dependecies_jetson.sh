#!/bin/sh

if [ $(dpkg -l | grep -c arducam) -lt 1 ]; then
    echo "Add Arducam_ppa repositories."
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
fi

# install dependency
sudo apt update
sudo apt install -y arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev

# sudo apt-get update
sudo apt-get install cmake curl -y
if [ $(dpkg -l | grep libopencv-dev -c) -lt 1 ] && [ $(dpkg -l | grep nvidia-opencv -c) -lt 1 ]; then
    sudo apt install -y nvidia-opencv
fi

wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m arducam
