## Run project 
### 1. driver Install
```Shell
  wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
  chmod +x install_pivariety_pkgs.sh
  ./install_pivariety_pkgs.sh -p kernel_driver
```
### 2.Install dependencies
> Run in the Arducam_tof_camera folder
```Shell
  chmod +x Install_dependencies.sh
  ./Install_dependencies.sh
```
### 3.compile && run
> Run in the Arducam_tof_camera folder
```Shell
  chmod +x compile.sh
  ./compile.sh
```
> You can also manually install and compile according to the following steps
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
  dtoverlay=arducam,media-controller=0
```
> To override the automatic camera detection, Bullseye users will also need to delete the entry camera_auto_detect=1 if present in the config.txt file. Your Raspberry Pi will need to be rebooted after editing this file
### example_cpp
```Shell
  sudo apt-get update
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev 
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
  cd Arducam_tof_camera/example
  mkdir build && cd build
  cmake .. &&  sudo make install
  make
```
## Start
### example_c
> Run in the build folder
```Shell
  ./example_c/test_c
```
### example_cpp
> Run in the build folder
```Shell
  ./example_cpp/preview_depth
  #OR
  ./example_cpp/capture_raw
```
### example_python
> Run in the example folder
```Shell
  cd ../example/example_python/
  python preview_depth.py
  #or
  python capture_raw.py
```
