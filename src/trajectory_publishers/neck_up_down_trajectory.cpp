// Copyright 2021 Halodi Robotics
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
#include <vector>

#include <boost/math/constants/constants.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "action_msgs/msg/goal_status.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "halodi_msgs/msg/reference_frame_name.hpp"
#include "halodi_msgs/msg/trajectory_interpolation.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"

#include "eve_ros2_examples/utils.h"

namespace eve_ros2_examples {

using namespace std::chrono_literals;
using halodi_msgs::msg::JointName;
using halodi_msgs::msg::ReferenceFrameName;
using halodi_msgs::msg::TrajectoryInterpolation;
using halodi_msgs::msg::WholeBodyTrajectory;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;
using std::placeholders::_1;

class TaskSpaceTrajectoryPublisher : public rclcpp::Node {
 public:
  TaskSpaceTrajectoryPublisher() : Node("task_space_trajectory_publisher") {
    // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is not up when
    // publishTrajectory is called the first time. Note: If the trajectory manager starts after this node, it'll execute immediatly.
    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();

    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", latching_qos);
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>(
        "/eve/whole_body_trajectory_status", 10, std::bind(&TaskSpaceTrajectoryPublisher::statusCallback, this, _1));

    // Send the first trajectory command. The subscriber will send additional commands to loop the same command in the subscriber
    // statusCallback
    uuidMsg_ = createRandomUuidMsg();
    publishTrajectory(uuidMsg_);
  }

 private:
  void statusCallback(const action_msgs::msg::GoalStatus::SharedPtr msg) const {
    switch (msg->status) {
      case 1:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_ACCEPTED");
        break;
      case 2:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_EXECUTING");
        break;
      case 4:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
        // If the uuid of the received GoalStatus STATUS_SUCCEEDED Msg is the same as the uuid of the command we sent out, let's send
        // another command
        if (msg->goal_info.goal_id.uuid == uuidMsg_.uuid) {
          uuidMsg_ = createRandomUuidMsg();
          publishTrajectory(uuidMsg_);
        }
        break;
      default:
        break;
    }
  }

  void publishTrajectory(unique_identifier_msgs::msg::UUID uuid_msg) const {
    WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    // neck limits lower="-0.335" upper="0.506"
    std::vector<double> neck_angles = {-0.33, 0.0, 0.5};
    std::vector<double> neck_timing = {2, 4, 6};

    for (std::vector<double>::size_type i = 0; i < neck_angles.size(); ++i) {
      WholeBodyTrajectoryPoint msg;
      builtin_interfaces::msg::Duration duration;
      duration.sec = neck_timing.at(i);
      msg.time_from_start = duration;
      msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::NECK_PITCH, neck_angles.at(i)));
      trajectory_msg.trajectory_points.push_back(msg);
    }

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);
  }

  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  mutable unique_identifier_msgs::msg::UUID uuidMsg_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::TaskSpaceTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
