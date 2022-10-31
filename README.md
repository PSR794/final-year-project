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
cd ~/catkin_ws/src
git clone https://github.com/introlab/rtabmap_ros.git 
catkin_make -j4
```
Use catkin_make -j1 if compilation requires more RAM than you have (e.g., some files require up to ~2 GB to build depending on gcc version).
### Running RTAB Mapping
Launch the following in different terminals
```
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
rosrun hector_ui ui_hector_quad.py
roslaunch rtabmap_ros rgbd_mapping.launch
```
