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

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include "action_msgs/msg/goal_status.hpp"
#include "eve_ros2_examples/utils.h"

namespace eve_ros2_examples {

using halodi_msgs::msg::JointName;
using halodi_msgs::msg::WholeBodyTrajectory;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;
using std::placeholders::_1;

using namespace std::chrono_literals;

class DefaultPosePublisher : public rclcpp::Node {
 public:
  DefaultPosePublisher() : Node("default_pose_publisher") {
    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<halodi_msgs::msg::WholeBodyTrajectory>("/eve/whole_body_trajectory", 10);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10,
                                                                            std::bind(&DefaultPosePublisher::statusCallback, this, _1));

    // Create a UUID for the first message.
    uuidMsg_ = createRandomUuidMsg();

    // Because publishers and subscribers connect asynchronously, we cannot guarantee that a message that is sent immediatly arrives at the
    // trajectory manager. Therefore, we use a timer and send the message every second till it it is accepted.
    timer = this->create_wall_timer(1000ms, [this]() { publishTrajectory(uuidMsg_); });
  }

 private:
  void statusCallback(action_msgs::msg::GoalStatus::SharedPtr msg) {
    // Check if the trajectory message accepted our message.
    // If you know for sure you only have one publisher, you can ignore the UUID and just accept the message.
    if (msg->goal_info.goal_id.uuid == uuidMsg_.uuid) {
      // Our message is accepted, we can cancel the timer now.
      timer->cancel();

      switch (msg->status) {
        case 1:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_ACCEPTED");
          break;
        case 2:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_EXECUTING");
          break;
        case 4:
          // after the robot has returned to the default pose, we shutdown the node
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
          RCLCPP_INFO(this->get_logger(), "Shutting down...");
          rclcpp::shutdown();
          break;
        default:
          break;
      }
    }
  }

  void publishTrajectory(unique_identifier_msgs::msg::UUID uuid_msg) {
    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(genDefaultMsg(uuid_msg));
  }

 private:
  rclcpp::Publisher<halodi_msgs::msg::WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  unique_identifier_msgs::msg::UUID uuidMsg_;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::DefaultPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
