# eve-ros2-examples

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

## Example structure
The EVE ROS2 API is essentially just the construction and publishing of our custom generated ROS2 message types found [here](https://github.com/Halodi/halodi-messages). To do this you must create a ROS2 node and publish a desired trajectory msg to the relevant topic. We illustrate the procedure for this in the examples present here, with the aim of exploring the API to its full extent. 

We use the simplest template of a ROS2 node, an explanation for which can be found [here](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/). Although given the simplicity, you should have few problems following this tutorial if you come from a ROS1 only background.

## Waving the hand
This example makes EVE wave her right hand using the Whole-body trajectory ROS2 Message.

The WholeBodyTrajectory msg is composed of a sequence of WholeBodyTrajectoryPoint(s) (3 in this case). Each WholeBodyTrajectoryPoint can be composed of desired task space commands, eg. desired end-effector positions, and/or desired joint space commands, eg. arm or leg joint angles, along with a desired time to get there.

This examples results in EVE waving her right hand using a sequence of joint space commands only. The trajectory is repeated until the node is terminated after which EVE will remain in the final configuration specified in the trajectory message.

Run the example using the following:
```bash
ros2 run eve-ros2-examples wave_right_hand
```

## Return to default
In the previous example, EVE remains in final pose specified in the published trajectory. This node resets EVE to the default pose by publishing once a WholeBodyTrajectory msg composed of default arm joint positions and a default pelvis pose. The node terminates after the desired pose is reached.

Run it using the following;
```bash
ros2 run eve-ros2-examples go_to_default
```

## Move Left Hand with Task Space Example
This example makes EVE move her left hand in a 5 point trajectory using the WholeBody trajectory ROS2 Message composed of TaskSpaceCommand msgs.
Run the example using the following:
```bash
ros2 run eve-ros2-examples move_left_hand
```

## Driving Command Example
This example makes EVE drive in a circle using the DrivingCommand ROS2 Message. Run the example using the following:
```bash
ros2 run eve-ros2-examples drive_in_circle
```
