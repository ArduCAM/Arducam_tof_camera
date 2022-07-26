## Install dependencies
### project dependencies
```Shell
  sudo apt update
  sudo apt-get install cmake
```
### driver Install
```Shell
  wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
  chmod +x install_pivariety_pkgs.sh
  ./install_pivariety_pkgs.sh -p kernel_driver
```
## Configuration
You need to alter the camera configuration in your /boot/config.txt file.to add dtoverlay.
```Shell
  camera_auto_detect=0
  dtoverlay=arducam,media-controller=0
```
### example_cpp
```Shell
  sudo apt update
  sudo apt-get install build-essential
  sudo apt-get install libopencv-dev
```
### example_python
```Shell
  sudo apt update
  sudo pip3 install opencv-python
  sudo pip3 install numpy --upgrade
```
## Compilation
```Shell
  mkdir build && cd build
  cmake .. &&  sudo make install
  make
```
## Start
### example_c
```Shell
  ./example_c/test_c
```
### example_cpp
```Shell
  ./example_c/test_cpp
```
### example_python
```Shell
  cd ../example/example_python/
  python test.py
```