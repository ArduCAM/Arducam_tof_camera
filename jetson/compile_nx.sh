sudo apt-get update
sudo apt-get install cmake -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install libtiff5-dev -y
sudo apt-get install li.jpg12-dev -y
sudo apt-get install libqtgui4 libqt4-test -y
sudo apt-get install libjasper-dev -y
# compile
cd `find ~ -name Arducam_tof_camera`
mkdir build && cd build
cmake ..
make 
./jetson/nx_preview 