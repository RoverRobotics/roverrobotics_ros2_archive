# roverrobotics_ros2
## About:
- This is a ROS2 wrapper to interface with roverrobotics' robots
- Librover is required in order to use this wrapper

## Installation instructions

1. Cloning this repository into your workspace
```
cd workspace/src/
git clone https://github.com/RoverRobotics/roverrobotics_ros2 
```
2. Install Udev rules for robot
```
cd workspace/src/roverrobotics_ros2/udev
sudo cp 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules && sudo udevadm control --reload-rules && udevadm trigger
```
3. Install shared library
``` 
cd ~/
mkdir library/
cd library/
git clone https://github.com/RoverRobotics/librover
cd librover/
cmake .
make
sudo make install 
```
4. Rebuild your workspace
```
cd workspace/
colcon build
```
5. Update env variables and configuration files 
```
source install/setup.bash
```
6. Launch Robot (replace <launch file name> with your robot config.)
```
ros2 launch roverrobotics_driver <launch file name>
```
