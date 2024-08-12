# Arducam Depth Camera

## Overview

This project is a use example based on arducam's depth camera. It includes basic image rendering using opencv, displaying 3D point clouds using PCL, and publishing depth camera data through the ROS2 system.
The depth camera is the depth data obtained by calculating the phase difference based on the transmitted modulated pulse. The resolution of the camera is 240*180. Currently, it has two range modes: 2 meters and 4 meters. The measurement error is within 2 cm.
The depth camera supports CSI and USB two connection methods, and needs an additional 5V 2A current power supply for the camera.

## Quick Start

### Clone this repository

Clone this repository and enter the directory.

```shell
  git clone https://github.com/ArduCAM/Arducam_tof_camera.git
  cd Arducam_tof_camera
```

### Install dependencies for Raspberry Pi

> Run in the Arducam_tof_camera folder
> Whatever you want to run the C/C++ examples or Python examples, you need to install the dependencies.

```shell
  ./Install_dependencies.sh
```

### Install dependencies for Jetson

> Run in the Arducam_tof_camera folder
> Whatever you want to run the C/C++ examples or Python examples, you need to install the dependencies.

```shell
  ./Install_dependencies_jetson.sh
```

### Run the examples on Raspberry Pi or Jetson

#### C/C++ Examples

##### Compile

> Run in the Arducam_tof_camera folder

```shell
  ./compile.sh
```

##### Run

###### C Example

> Run in the build/example/c folder

```shell
  cd build/example/c
```

```shell
  ./preview_depth_c
```

###### C++ Example

> Run in the build/example/cpp folder

```shell
  cd build/example/cpp
```

```shell
  ./preview_depth
  #or
  ./capture_raw
```

#### PCL Examples

##### Dependencies

```Shell
  sudo apt update
  sudo apt-get install build-essential cmake 
  sudo apt-get install libopencv-dev libpcl-dev
```

##### Compile

> Run in the Arducam_tof_camera folder

```shell
  ./compile_pcl.sh
```

##### Run

> Run in the build/pcl_preview folder

```shell
  cd build/pcl_preview
```

```shell
  ./preview_pointcloud
```

#### Python Examples

##### Run

###### Python Example

> Run in the example/python folder

```shell
  cd example/python
```

```shell
  python3 preview_depth.py
  #or
  python3 capture_raw.py
```