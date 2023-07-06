# naoqi_driver2

This repo defines the __naoqi_driver__ package for ROS2. The driver is in charge of providing bridging capabilities between ROS2 and NAOqiOS.

## Dependencies
To run, the driver requires the `naoqi_libqi`, `naoqi_libqicore` and `naoqi_bridge_msgs` packages. 
> due to an error inside libqicore official package, we need to build this package from source

### Using the binaries
To install the binaries, use the following commands:
```sh
sudo apt-get install ros-<your_distro>-naoqi-libqi
# go to your colcon_ws/src
git clone https://github.com/Effibot/naoqi_bridge_msgs2.git
git clone -b ros2 https://github.com/ros-naoqi/libqi.git
git clone https://github.com/Effibot/naoqi_driver2.git

# inside colcon_ws folder
colcon build --merge-install --symlink-install 
```

## Launch
The driver can be launched using the following command:
```sh
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<ip> nao_port:=<port> network_interface:=<interface> username:=<name> password:=<passwd>
```
Note that the username and password arguments are only required for robots running naoqi 2.9 or greater.
