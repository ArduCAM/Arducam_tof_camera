### Run the example in the ros2_publisher folder
> Please refer to the instructions above for  _*a. Driver Install*_ and camera _*b.Configuration*_
#### Ros2 installed on RaspberryPI
>The example is based on ros2,We found the installation script on github, you can install it as follows, or you can install it yourself. author : [Ar-Ray](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)
```Shell
# default : (humble, arm7l)
curl -s https://raw.githubusercontent.com/Ar-Ray-code/rpi-bullseye-ros2/main/install.bash | bash
```

```Shell
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