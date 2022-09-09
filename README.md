## ArduCam Depth Camera 
### Overview
This project is a use example based on arducam's depth camera. It includes basic image rendering using opencv, displaying 3D point clouds using PCL, and publishing depth camera data through the ROS2 system.
The depth camera is the depth data obtained by calculating the phase difference based on the transmitted modulated pulse. The resolution of the camera is 240*180. Currently, it has two range modes: 2 meters and 4 meters. The measurement error is within 2 cm.
The depth camera supports CSI and USB two connection methods, and needs an additional 5V 2A current power supply for the camera.
### Run project 
#### SDK install
```Shell
  curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
  sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  sudo apt update
  sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
```
### Run the example in the exmaple folder
#### 1.Install dependencies
> Run in the Arducam_tof_camera folder
```Shell
  ./Install_dependencies.sh
```
### 3.compile && run
> Run in the Arducam_tof_camera folder
```Shell
  ./compile.sh
```
> You can also manually install and compile according to the following steps
### RassperyPi
#### a.Driver Install
```Shell
  wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
  chmod +x install_pivariety_pkgs.sh
  ./install_pivariety_pkgs.sh -p kernel_driver
```
### b.Configuration
You need to alter the camera configuration in your /boot/config.txt file.to add dtoverlay.
```Shell
  dtoverlay=arducam,media-controller=0
```
> To override the automatic camera detection, Bullseye users will also need to delete the entry camera_auto_detect=1 if present in the config.txt file. Your Raspberry Pi will need to be rebooted after editing this file
### c&cpp example
#### project dependencies
```Shell
  sudo apt-get update
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev 
  sudo apt-get install libopencv-dev
```
##### Compilation
```Shell
  cd Arducam_tof_camera/example
  mkdir build && cd build
  cmake ..
  make
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
    #or
  ./cpp/preview_pointcloud
```
### python example
#### project dependencies
```Shell
  sudo apt update
  sudo pip3 install opencv-python
  sudo pip3 install numpy --upgrade
```
#### example_python
> Run in the example folder
```Shell
  cd ../example/example_python/
  python preview_depth.py
  #or
  python capture_raw.py
```
### Run the example in the pcl_preview folder
> Please refer to the instructions above for  _*a. Driver Install*_ ' and camera _*b.Configuration*_
#### project dependencies
```Shell
  sudo apt update
  sudo apt install cmake libpcl-dev
```
#### Compilation
```Shell
  cd Arducam_tof_camera/pcl_preview
  mkdir build && cd build
  cmake ..
  make
```
#### Run
> Run in the pcl_preview/build folder
```Shell
  ./preview_pointcloud
```
### Run the example in the ros2_publisher folder
> Please refer to the instructions above for  _*a. Driver Install*_ ' and camera _*b.Configuration*_
#### Ros2 install
The example is based on ros2, please refer to [ros2 installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and install it yourself

```Shell
# Install the compilation tool colcon
sudo apt install python3-colcon-common-extensions
# Environment variable configuration,Take the humble version as an example
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc 
source ~/.bashrc 
```
#### Compilation
```Shell
  cd Arducam_tof_camera/ros2_publisher
  colcon build --merge-install
```
#### Run
```Shell
  . install/setup.bash 
  ros2 run arducam tof_pointcl
```
>You can preview by running rviz2 on the host in the LAN
