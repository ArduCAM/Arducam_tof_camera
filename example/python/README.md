
### python example
### Run the example in the exmaple folder
#### Install dependencies
> Run in the Arducam_tof_camera folder
```Shell
  ./Install_dependencies_python.sh
```
#### example_python
> Run in the example folder
```Shell
  cd ../example/example_python/
  python preview_depth.py
  #or
  python capture_raw.py
```
> You can also manually install dependencies according to the following steps
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
#### Install python dependencies
```Shell
  sudo apt update
  sudo pip3 install opencv-python ArducamDepthCamera
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