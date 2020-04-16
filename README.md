# eve-ros2-examples
will@halodi.com

A ROS2 package full of C++ examples that show the usage of all of EVE's ROS2 API

## Setup
Follow the install instructions [here](https://github.com/Halodi/halodi-controller-simulation-api) to setup your EVE ROS2 workspace. Then, clone this repo into your eve_ws/src folder
and build again to start using the examples
```bash
cd ~/eve_ws/src
git clone https://github.com/Halodi/eve-ros2-examples.git
```

## Driving Command Example
This example makes EVE drive in a circle using the DrivingCommand ROS2 Message. Run the example using the following:
```bash
ros2 run eve-ros2-examples drive_command_publisher
```
You can find the code for this example in [drive_command_publisher.cpp](./src/drive_command_publisher.cpp)
