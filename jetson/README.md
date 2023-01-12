## Arducam Depth Camera
### Run project on Jetson
#### Install dependencies
> Run in the Arducam_tof_camera/jetson folder
```Shell
  ./Install_dependecies_jetson.sh
```
### Compile && Run
> Run in the Arducam_tof_camera/jetson folder
```Shell
  ./compile.sh
```
> You can also manually install and compile according to the following steps
#### Driver install 
```Shell
  wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
  chmod +x install_full.sh
  ./install_full.sh -m arducam
```
#### project dependencies
```Shell
  sudo apt-get update
  sudo apt-get install build-essential cmake
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
  cd Arducam_tof_camera
  mkdir build && cd build
  cmake ..
  make
```
#### Start Jetson Preview
> Run in the example/build folder
```Shell
  ./jetson/jetson_preview 
```
