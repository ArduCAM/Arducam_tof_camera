
# modfiy config
sudo sed -i "s/\(^camera_auto_detect=*\)/#\1/" /boot/config.txt
sudo bash -c 'echo camera_auto_detect=0 >> /boot/config.txt'
sudo sed -i "s/\(^dtoverlay=*\)/#\1/" /boot/config.txt
sudo bash -c 'echo dtoverlay=vc4-fkms-v3d >> /boot/config.txt'
sudo bash -c 'echo dtoverlay=arducam,media-controller=0 >> /boot/config.txt'

# install dependency
sudo apt-get update
sudo apt-get install cmake -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install libtiff5-dev -y
sudo apt-get install li.jpg12-dev -y
sudo apt-get install libqtgui4 libqt4-test -y
sudo apt-get install libjasper-dev -y
sudo apt-get install libopencv-dev -y
sudo pip3 install opencv-python
sudo pip3 install numpy --upgrade
sudo ldconfig
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