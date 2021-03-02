# Roverrobotics_ros2_b
## About:
- This is a ROS2 wrapper to interface with roverrobotics' robots
- Librover is required in other to use this wrapper

## Installation instructions

1. Cloning this repository into your workspace
```
cd workspace/src/
git clone https://github.com/RoverRobotics/roverrobotics_ros2_b 
```
2. Install shared library
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
2. Rebuild your workspace
```
cd workspace/
colcon build
```
3. Launch Robot (replace <launch file name> with your robot config.)
```
ros2 launch roverrobotics_driver <launch file name>
```
