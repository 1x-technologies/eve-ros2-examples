# eve-ros2-examples
will@halodi.com

A ROS2 package full of C++ examples that show the usage of all of EVE's ROS2 API

## Setup
Follow the install instructions [here](https://github.com/Halodi/halodi-controller-simulation-api) to setup your EVE ROS2 workspace. Then, clone this repo into your eve_ws/src folder
and build again to start using the examples
```bash
cd ~/eve_ws/src
git clone https://github.com/Halodi/eve-ros2-examples.git
cd ~/eve_ws
colcon build
. install/setup.bash
```
Then launch the gazebo sim of EVE. 
```bash
ros2 launch halodi-controller-gazebo halodi-controller-gazebo.launch.py
```

Once it's running, open a different terminal and try the different commands for the different examples below.

## Driving Command Example
This example makes EVE drive in a circle using the DrivingCommand ROS2 Message. Run the example using the following:
```bash
ros2 run eve-ros2-examples drive_command_publisher
```
You can find the code for this example in [drive_command_publisher.cpp](./src/drive_command_publisher.cpp)

## Wholebody Trajectory Example
This example makes EVE move her right hand in a 5 point trajectory using the WholeBody trajectory ROS2 Message. Run the example using the following:
```bash
ros2 run eve-ros2-examples whole_body_trajectory_publisher
```
You can find the code for this example in [whole_body_trajectory_publisher.cpp](./src/whole_body_trajectory_publisher.cpp)
