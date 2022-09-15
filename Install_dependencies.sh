
# modfiy config
sudo sed -i "s/\(^camera_auto_detect=*\)/#\1/" /boot/config.txt
sudo bash -c 'echo camera_auto_detect=0 >> /boot/config.txt'
sudo sed -i "s/\(^dtoverlay=*\)/#\1/" /boot/config.txt
sudo bash -c 'echo dtoverlay=vc4-fkms-v3d >> /boot/config.txt'
sudo bash -c 'echo dtoverlay=arducam,media-controller=0 >> /boot/config.txt'

curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
sudo apt update
sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev

# install dependency
sudo apt-get update
sudo apt-get install cmake -y
sudo apt install libopencv-dev -y
echo "reboot now?(y/n):"
read USER_INPUT
case $USER_INPUT in
'y'|'Y')
    echo "reboot"
    sudo reboot
;;
*)
    echo "cancel"
    echo "The script settings will only take effect after restarting, please restart yourself later."
    exit -1
;;
esac