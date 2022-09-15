## ArduCam Depth Camera 
### Run the example in the pcl_preview folder
> Please refer to the instructions above for  _*a. Driver Install*_ ' and camera _*b.Configuration*_
#### project dependencies
```Shell
  sudo apt update
  sudo apt-get install build-essential cmake 
  sudo apt-get install libopencv-dev libpcl-dev
```
#### Compilation
```Shell
  cd Arducam_tof_camera/pcl_preview
  mkdir build && cd build
  
  # If the platform running the program is Jetson nano
  cmake -DPLATFORM:STRING=NANO ..
  # Else if the platform running the program is Jetson NX
  cmake -DPLATFORM:STRING=NANO ..
  # Else
  cmake -DPLATFORM:STRING=DEFAULT ..

  make
```
#### Run
> Run in the pcl_preview/build folder
```Shell
  ./preview_pointcloud
```