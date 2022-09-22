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