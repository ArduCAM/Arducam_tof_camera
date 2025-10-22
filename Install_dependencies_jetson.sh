#!/bin/sh

# install dependency
sudo apt update
if ! sudo apt install -y cmake curl python3-pip python3-opencv python3-numpy >/dev/null 2>&1; then
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
if ! sudo apt install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev >/dev/null 2>&1; then
    echo -e "\033[31m[ERR]\033[0m Failed to install tof sdk."
    echo -e "\033[31m[ERR]\033[0m Please check your network connection."
    exit 1
fi
if ! sudo python -m pip install ArducamDepthCamera >/dev/null 2>&1; then
    if ! sudo python -m pip install ArducamDepthCamera --break-system-packages >/dev/null 2>&1; then
        echo -e "\033[31m[ERR]\033[0m Failed to install ArducamDepthCamera."
    fi
fi
echo -e "\033[32m[INFO]\033[0m To install for python venv."
echo -e "  please run: python -m pip install ArducamDepthCamera opencv-python \"numpy<2.0.0\""

# sudo apt-get update
if [ $(dpkg -l | grep libopencv-dev -c) -lt 1 ] && [ $(dpkg -l | grep nvidia-opencv -c) -lt 1 ]; then
    sudo apt install -y nvidia-opencv
fi

wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m arducam
