# FYP Project 

## Build and Installation
1.  In a catkin workspace execute the following commands
```
cd catkin_ws/src
git clone https://github.com/ros-geographic-info/unique_identifier.git
git clone https://github.com/ros-geographic-info/geographic_info.git
```
2. Build the workspace
```
cd ..
catkin_make
```
3. Clone the following again
```
cd src/
git clone https://github.com/RAFALAMAO/hector-quadrotor-noetic.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
4. Repeat Step 2 

### Basic Simulation Setup
* Append these following commands in the bashrc
```
source path_to_workspace/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```
* For launching turtlebot and drone together in an environment
```
roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
```
### Installing RTAB 
1. Install RTAB-Map standalone libraries. Do not clone in your Catkin workspace.
```
cd ~
git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake ..
make -j6
sudo make install
```
2. Install RTAB-Map ros-pkg in your src folder of your Catkin workspace.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/introlab/rtabmap_ros.git 
$ catkin_make -j4
```
Use catkin_make -j1 if compilation requires more RAM than you have (e.g., some files require up to ~2 GB to build depending on gcc version).

### AMCL
```
$ roslaunch hector_quadrotor_gazebo tbot_amcl.launch 
``` 
Launch the same file above for simultaenous simulation

For teleop 
```
$ rosrun turtlebot3_gazebo tutb_teleop.py
```

For Loading Map
```
$ cd FYP/src/map
$ rosrun map_server map_server drone_map.yaml
```
If AMCL gives error of not receiving the scans from the LiDAR of tbot then you have to change these files :
1. in `/opt/ros/noetic/share/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro` line 102 change name to `laser`, 
line 129 change topic name to `tbot/scan`, line 130 change frameName to `/tbot/laser`.

2. in `/opt/ros/noetic/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro` line 140 change link name to `laser`.


