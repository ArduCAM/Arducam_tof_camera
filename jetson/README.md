## ArduCam Depth Camera by Jetson
### Run project 
#### SDK install
```Shell
  curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
  sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  sudo apt update
  sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev arducam-tof-sdk-dev
```
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
> The project depends on opencv
```Shell
  sudo apt-get update
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev 
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