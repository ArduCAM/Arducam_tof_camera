## Arducam Depth Camera 
### Overview
This project is a use example based on arducam's depth camera. It includes basic image rendering using opencv, displaying 3D point clouds using PCL, and publishing depth camera data through the ROS2 system.
The depth camera is the depth data obtained by calculating the phase difference based on the transmitted modulated pulse. The resolution of the camera is 240*180. Currently, it has two range modes: 2 meters and 4 meters. The measurement error is within 2 cm.
The depth camera supports CSI and USB two connection methods, and needs an additional 5V 2A current power supply for the camera.
### Run project on Raspberry Pi
### Run the example in the example folder
#### Install dependencies
> Run in the Arducam_tof_camera folder
```Shell
  ./Install_dependencies.sh
```
### compile && run
> Run in the Arducam_tof_camera folder
```Shell
  ./compile.sh
```
> You can also manually install and compile according to the following steps  

#### 1.Driver Install
> Note: Since 5.15.38, the arducam-pivariety driver has been merged into the Raspberry Pi kernel and the name of the device tree is changed to arducam-pivariety, so dtoverlay=arducam-pivariety is required to set the overlay
<s>

```Shell
  $ wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
  $ chmod +x install_pivariety_pkgs.sh
  $ ./install_pivariety_pkgs.sh -p kernel_driver
```

</s>

### 2.Configuration
You need to alter the camera configuration in your /boot/firmware/config.txt file.to add dtoverlay.
```Shell
  dtoverlay=arducam-pivariety,media-controller=0
```
> To override the automatic camera detection, Bullseye users will also need to delete the entry camera_auto_detect=1 if present in the config.txt file. Your Raspberry Pi will need to be rebooted after editing this file
#### 3.SDK install
```Shell
  curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
  sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  sudo apt update
  sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
```
### c&cpp example
#### project dependencies
```Shell
  sudo apt-get update
  sudo apt-get install build-essential cmake 
  sudo apt-get install libopencv-dev
```
##### Compilation
```Shell
  cd Arducam_tof_camera/example
  mkdir build && cd build
  cmake .. && make
```
### Start
#### c test
> Run in the example/build folder
```Shell
  ./c/test_c
```
#### cpp example
> Run in the example/build folder
```Shell
  ./cpp/preview_depth
  #or
  ./cpp/capture_raw
```
