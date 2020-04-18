// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <boost/math/constants/constants.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/reference_frame_name.hpp"
#include "halodi_msgs/msg/trajectory_interpolation.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/task_space_command.hpp"


using namespace std::chrono_literals;
using namespace halodi_msgs::msg;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WholeBodyTrajectoryPublisher : public rclcpp::Node
{
public:
  WholeBodyTrajectoryPublisher()
  : Node("whole_body_trajectory_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&WholeBodyTrajectoryPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto trajectory_msg = WholeBodyTrajectory();
    trajectory_msg.append_trajectory = true;
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;

    add_hand_target(trajectory_msg, 1, 0.25, -0.35, 0.20, 0.0, -pi/2.0, 0.0);
    add_hand_target(trajectory_msg, 2, 0.25, -0.35, 0.05, 0.0, -pi/2.0, 0.0);
    add_hand_target(trajectory_msg, 3, 0.25, -0.15, 0.05, 0.0, -pi/2.0, 0.0);
    add_hand_target(trajectory_msg, 4, 0.25, -0.15, 0.20, 0.0, -pi/2.0, 0.0);
    add_hand_target(trajectory_msg, 5, 0.25, -0.35, 0.20, 0.0, -pi/2.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "WholeBodyTrajectory Executing...");
    publisher_->publish(trajectory_msg);
  }

  void add_hand_target(WholeBodyTrajectory trajectory, int32_t t, double x, double y, double z, double yaw, double pitch, double roll){
    WholeBodyTrajectoryPoint target = WholeBodyTrajectoryPoint();
    TaskSpaceCommand rightHandCommand = TaskSpaceCommand();

    geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
    geometry_msgs::msg::Point point = geometry_msgs::msg::Point();
    point.x = x; point.y = y; point.z = z;
    geometry_msgs::msg::Quaternion quat_msg = geometry_msgs::msg::Quaternion();
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    quat_msg = tf2::toMsg(quat_tf);
    pose.position = point; pose.orientation = quat_msg;
    rightHandCommand.pose = pose;

    ReferenceFrameName::Type body_frame, reference_frame;
    body_frame.frame_id = ReferenceFrameName::RIGHT_HAND;
    reference_frame.frame_id = ReferenceFrameName::PELVIS;
    rightHandCommand.body_frame = body_frame;
    rightHandCommand.expressed_in_frame = reference_frame;
    rightHandCommand.express_in_z_up = true;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    target.time_from_start = duration;

    target.task_space_commands.push_back(rightHandCommand);
    trajectory.trajectory_points.push_back(target);
  }

  const double pi = boost::math::constants::pi<double>();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodyTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
