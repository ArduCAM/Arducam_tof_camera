## ArduCam Depth Camera by Jetson
### Run project 
### Driver install 
```Shell
  wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
  chmod +x install_full.sh
  ./install_full.sh -m arducam
```
### Compile && Run
> Run in the Arducam_tof_camera folder
```Shell
  ./compile_nano.sh
  # or
  ./compile_nx.sh
```
> You can also manually install and compile according to the following steps
### c&cpp example
#### project dependencies
```Shell
  sudo apt-get update
  sudo apt-get install build-essential 
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  nvidia-opencv
```
> The project depends on opencv,If it is not installed, please install it manually.
```Shell
  sudo apt-get update
  sudo apt-get nvidia-opencv
```
#### SDK install
```Shell
  curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
  sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  sudo apt update
  sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
```
##### Compilation
```Shell
  cd Arducam_tof_camera/example
  mkdir build && cd build
  cmake ..
  make
```
### Start
#### cpp example
> Run in the example/build folder
```Shell
  ./jetson/nx_preview   
  #or
  ./jetson/nano_preview   
```
### Run the example in the ros2_publisher folder
> Please refer to the instructions above for  _* Driver Install*_ .
#### Ros2 installed on RaspberryPI
>The example is based on ros2,We found the installation script on github, you can install it as follows, or you can install it yourself.
```Shell
# default : (humble, arm7l)
curl -s https://raw.githubusercontent.com/Ar-Ray-code/rpi-bullseye-ros2/main/install.bash | bash
```
> author : [Ar-Ray](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)
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