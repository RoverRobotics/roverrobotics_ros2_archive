# roverrobotics_ros2
## About:
- This is a ROS2 wrapper to interface with roverrobotics' robots.
- Librover is required in order to use this wrapper
- This is being used for the development of the rover robotics mini on ros2

## Installation instructions

1. Cloning this repository into your workspace
```
cd workspace/src/
git clone https://github.com/jackarivera/roverrobotics_mini_ros2 
```
2. Install Udev rules for robot
```
cd workspace/src/roverrobotics_mini_ros2/udev
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
source install/setup.sh
```
6. Launch Robot (replace <launch file name> with your robot config.)
```
ros2 launch roverrobotics_driver <launch file name>
```
  ```
  Launch Files:
  <model>.launch.py: Launches robot configuration for specific rover robot. (i.e. Mini, Mega, Pro2, Zero2)
  <model>_teleop.launch.py: Launches robot configuration with teleop controller enabled.
  rover_slam_mapping.launch.py: Launches lidar and slam toolbox in asynchronous mapping mode(Requires slam package).
  rover_slam_localization.launch.py: Launches lidar and slam toolbox in localization mapping mode.
  ```
## Detailed Google Doc with Installation/Setup Instructions
[Setup Instructions](https://docs.google.com/document/d/1U1tGi4dLHvz1jjJ7PQ9zkREM9P-C8m2a0YD0BKWqybM/edit?usp=sharing)
    
Additional ROS2 Packages:
    
[Intel Realsense d435i Driver](https://github.com/IntelRealSense/realsense-ros)
    
[Slamtec SlLidar Driver](https://github.com/Slamtec/sllidar_ros2)
