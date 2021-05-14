# eve-ros2-examples

A ROS2 package full of C++ examples that show the usage of all of EVE's ROS2 API. Each example is a ROS2 node which publishes a trajectory for a real or simulated Eve to perform.

## Download and Build

To download and compile this repository on its own follow the instructions below. You will first require at least the base install of [ros2](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html).

```bash
mkdir -p ~/eve_ws/src
cd ~/eve_ws/src

# Clone this repo and dependencies
## ssh
git clone git@gitlab.com:halodi/controls/ros2/eve-ros2-examples.git
git clone git@gitlab.com:halodi/controls/halodi-messages.git

## Building
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

To test on a simulated robot follow the install instructions [here](https://github.com/Halodi/halodi-controller/) to setup your EVE ROS2 workspace. Then, clone this repo into your eve_ws/src folder
and follow the same build steps as above.

Launch the gazebo sim of EVE.

```bash
ros2 launch halodi-controller-gazebo halodi-controller-gazebo.launch.py
```

Once it's running, open a different terminal and try the different commands for the different examples below.

## Examples

### Structure

The EVE ROS2 API is essentially just the construction and publishing of our custom generated ROS2 message types found [here](https://github.com/Halodi/halodi-messages). To do this you must create a ROS2 node and publish a desired trajectory msg to the relevant topic. We illustrate the procedure for this in the examples present here, with the aim of exploring the API to its full extent.

We use the simplest template of a ROS2 node, an explanation for which can be found [here](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/). Although given the simplicity, you should have few problems following this tutorial if you come from a ROS1 only background.

### Waving the right hand with joint space trajectory

This example makes Eve wave her right hand using the Whole-body trajectory ROS2 Message.

The WholeBodyTrajectory msg is composed of a sequence of WholeBodyTrajectoryPoint(s) (3 in this case). Each WholeBodyTrajectoryPoint can be composed of desired task space commands, eg. desired end-effector positions, and/or desired joint space commands, eg. arm or leg joint angles, along with a desired time to get there.

This examples results in EVE waving her right hand using a sequence of joint space commands only using the JointSpaceCommand msg type. The trajectory is repeated until the node is terminated after which Eve will remain in the final configuration specified in the trajectory message.

Run the example using the following:

```bash
ros2 run eve-ros2-examples wave_right_hand
```

### Return to default

In these examples, Eve remains in final pose specified in the published trajectory, which may not be the default pose. This node resets Eve to the default pose by publishing once a WholeBodyTrajectory msg composed of default arm joint positions and a default pelvis pose. The node terminates after the desired pose is reached. These default joint angles can be parsed from the [urdf](https://gitlab.com/halodi/controls/halodi-robot-models), although we hard code them here.

Run it using the following;

```bash
ros2 run eve-ros2-examples default_pose
```

### Moving the left hand with task space trajectory

This example makes Eve move her left hand in a box shaped 5 point trajectory using the WholeBody trajectory ROS2 Message composed of TaskSpaceCommand msgs.
Run the example using the following:

```bash
ros2 run eve-ros2-examples left_hand_task_space_box
```

### Moving the neck up and down

This example moves the neck up and down. The position gain may have to be increased in the trajectory msg as the default is quite low. This example is handy for exploring the visible area of the camera mounted in the head.

```bash
ros2 run eve-ros2-examples neck_up_down
```

### Small random arm motions around default

This example create a sequence of arm joint angles chosen uniformly at random around their default configuration. The limits of the distribution are *+- 0.2 radians*. The pelvis height is also varied by a few centimeters only. This example is handy for having the robot appear somewhat lifelike during demos.

```bash
ros2 run eve-ros2-examples random_arm_motion_low_range
```

### Driving and turning

**Do not run this example while the robot is tethered AND if you cannot quickly kill the node!!**
This example makes Eve drive in a circle using the DrivingCommand ROS2 Message. It is useful for testing changes in the balancing/driving controller in **simulation**.
Run the example using the following:

```bash
ros2 run eve-ros2-examples drive_and_turn
```

### Full body extreme motions

**Do not run this example if the robot is tethered tightly AND if you cannot quickly kill the node!!**
This example makes Eve move her whole body via a sequence of arm joints and pelvis height trajectories. The motions are rapid and engage the end stops of some joints. This example is intended for exploring the response of the balancing controller.
Run the example using the following:

```bash
ros2 run eve-ros2-examples full_body_extreme
```

## Testing

In order to test the code in this repository or new code that you want to contribute, you must first run and pass a number of given tests that are specified in the CMakeLists.txt file
These tests include a mixture of formatting a linting tools that span the c++ and python languages.

First clone our clang configurations into your workspace

```bash
cd ~/eve_ws/src
git clone git@gitlab.com:halodi/controls/halodi-clang-tooling.git
```

You then need to build again and run the test scripts:

```bash
cd ~/eve_ws/
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --event-handlers console_cohesion+ --packages-select eve_ros2_examples
```

Please fix any errors before making a PR
