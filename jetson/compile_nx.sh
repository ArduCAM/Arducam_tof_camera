sudo apt-get update
sudo apt-get install cmake curl -y
sudo apt install -y  nvidia-opencv

curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
sudo apt update
sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev

# compile
cd `find ~ -name Arducam_tof_camera`
cd `find . -name jetson`

mkdir build && cd build
cmake ..
make 
./nx_preview 