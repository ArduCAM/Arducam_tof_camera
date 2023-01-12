## Arducam Depth Camera 
### Run the example in the pcl_preview folder
> Please refer to the instructions above for  _*a. Driver Install*_ , _*b.Configuration*_ and _*c.SDK install*_ whice run on RaspberryPi

> Please refer to the instructions above for  _*project dependencies*_ and _*SDK install*_ whice run on Jetson
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
  
  cmake ..

  make
```
#### Run
> Run in the pcl_preview/build folder
```Shell
  ./preview_pointcloud
```