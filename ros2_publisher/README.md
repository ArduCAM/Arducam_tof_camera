### TOF_PointCloud_Publisher
___
本工程基于arducam_tof_camera的sdk,通过ROS发布tof相机的深度数据和点云数据
### 安装依赖

ROS2 
> 开发环境如果是 Ubuntu18.04 请参考此[链接](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages)进行安装ROS2.
### 设置环境变量
```Shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc 
source ~/.bashrc 
```
### 安装SDK
```Shell
git clone https://github.com/dennis-ard/arducam_tof_camera.git
```
> 请根据仓库中README进行安装SDK,在此不在赘述

### 编译
```Shell
git clone https://github.com/dennis-ard/tof_ros2_publisher.git
cd tof_ros2_publisher
colcon build --symlink-install
```
### 运行
```Shell
. install/setup.bash 
ros2 run arducam tof_pointcloud
```
>可以通过局域网内主机运行rviz2来进行预览