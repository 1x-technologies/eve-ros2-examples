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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"

#include "eve_ros2_examples/utils.h"

namespace eve_ros2_examples {

using halodi_msgs::msg::JointName;
using halodi_msgs::msg::JointSpaceCommand;
using halodi_msgs::msg::TrajectoryInterpolation;
using halodi_msgs::msg::WholeBodyTrajectory;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;
using std::placeholders::_1;

using namespace std::chrono_literals;

class WaveRightHandPublisher : public rclcpp::Node {
 public:
  WaveRightHandPublisher() : Node("waving_hand_trajectory_publisher") {
    // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is
    // not up when publishTrajectory is called the first time. Note: If the trajectory manager starts after this node,
    // it'll execute immediatly.
    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();

    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/internal/whole_body_trajectory", latching_qos);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>(
        "/eve/whole_body_trajectory_status", 10, std::bind(&WaveRightHandPublisher::statusCallback, this, _1));

    // Create a UUID for the first message.
    uuidMsg_ = createRandomUuidMsg();

    // Because publishers and subscribers connect asynchronously, we cannot guarantee that a message that is sent
    // immediatly arrives at the trajectory manager. Therefore, we use a timer and send the message every second till it
    // it is accepted.
    timer_ = this->create_wall_timer(1000ms, [this]() { publishTrajectory(uuidMsg_); });
  }

 private:
  void statusCallback(action_msgs::msg::GoalStatus::SharedPtr msg) {
    // If the uuid of the received GoalStatus STATUS_SUCCEEDED Msg is the same as the uuid of the command we sent out,
    // let's send another command
    if (msg->goal_info.goal_id.uuid == uuidMsg_.uuid) {
      // Our message is accepted, we can cancel the timer now.
      timer_->cancel();

      switch (msg->status) {
        case 1:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_ACCEPTED");
          break;
        case 2:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_EXECUTING");
          break;
        case 4:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
          uuidMsg_ = createRandomUuidMsg();
          publishTrajectory(uuidMsg_);
          break;
        default:
          break;
      }
    }
  }

  void publishTrajectory(unique_identifier_msgs::msg::UUID uuid_msg) {
    // begin construction of the publsihed msg
    WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint
    // velocities and accelerations between each waypoint
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    // begin adding waypoint targets, the desired times {2, 4, 6} (ses) are provided in terms of
    // offset from time at which this published message is received
    trajectory_msg.trajectory_points.push_back(target1(2));
    trajectory_msg.trajectory_points.push_back(target2(4));
    trajectory_msg.trajectory_points.push_back(target3(6));

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);
  }

  /*
  Each target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of desired joint
  configurations, with no more than one desired value per joint.

  The desired time at which we want to reach these joint targets is also specified.
  */
  WholeBodyTrajectoryPoint target1(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target2(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -0.6));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target3(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.85));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  unique_identifier_msgs::msg::UUID uuidMsg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::WaveRightHandPublisher>());
  rclcpp::shutdown();
  return 0;
}
