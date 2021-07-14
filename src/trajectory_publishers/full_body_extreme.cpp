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

#include <functional>
#include <memory>

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "eve_ros2_examples/utils.h"
#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"

namespace eve_ros2_examples {

using halodi_msgs::msg::JointName;
using halodi_msgs::msg::ReferenceFrameName;
using halodi_msgs::msg::TrajectoryInterpolation;
using halodi_msgs::msg::WholeBodyTrajectory;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;
using std::placeholders::_1;

using namespace std::chrono_literals;

const double DEFAULT_HEIGHT_ = 0.91;
const double SQUAT_HEIGHT_ = 0.7;
const int TIME_INCREMENT_ = 3;

class FullBodyExtremePublisher : public rclcpp::Node {
 public:
  FullBodyExtremePublisher() : Node("full_body_extreme_publisher") {
    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", 10);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10,
                                                                            std::bind(&FullBodyExtremePublisher::statusCallback, this, _1));

    // Create a UUID for the first message.
    uuidMsg_ = createRandomUuidMsg();

    // Because publishers and subscribers connect asynchronously, we cannot guarantee that a message that is sent immediatly arrives at the
    // trajectory manager. Therefore, we use a timer and send the message every second till it it is accepted.
    timer_ = this->create_wall_timer(1000ms, [this]() { publishTrajectory(uuidMsg_); });
  }

 private:
  void statusCallback(action_msgs::msg::GoalStatus::SharedPtr msg) {
    // If the uuid of the received GoalStatus STATUS_SUCCEEDED Msg is the same as the uuid of the command we sent out, let's send
    // another command
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

    int t = 0;
    // we give the robot 3 seconds to return to its default configuration
    trajectory_msg.trajectory_points.push_back(genDefaultTarget(t += 1));
    trajectory_msg.trajectory_points.push_back(target1(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(target2(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(genDefaultTarget(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(target3(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(target4(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(genDefaultTarget(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(target5(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(target6(t += TIME_INCREMENT_));
    trajectory_msg.trajectory_points.push_back(genDefaultTarget(t += TIME_INCREMENT_));

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);
  }

  WholeBodyTrajectoryPoint target1(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, SQUAT_HEIGHT_));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, 0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

    return ret_msg;
  }

  WholeBodyTrajectoryPoint target2(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, SQUAT_HEIGHT_));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, -0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, -0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

    return ret_msg;
  }

  WholeBodyTrajectoryPoint target3(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, SQUAT_HEIGHT_));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, -0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

    return ret_msg;
  }

  WholeBodyTrajectoryPoint target4(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, DEFAULT_HEIGHT_));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, -0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, 0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

    return ret_msg;
  }

  WholeBodyTrajectoryPoint target5(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    // the 1.0472 value is the joint limit of j_hip_z. Publishing values outside of the limits will not cause things to fail, although will
    // result in non-smooth motions
    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, SQUAT_HEIGHT_, 0.0, 0.0, 1.0472));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, -2.42409));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 2.0944));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, -0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

    return ret_msg;
  }

  WholeBodyTrajectoryPoint target6(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.task_space_commands.push_back(
        generateTaskSpaceCommand(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, SQUAT_HEIGHT_, 0.0, 0.0, -1.0472));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, -0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_PITCH, 0.98105));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_ROLL, 1.0));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_SHOULDER_YAW, 0.541053));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.1));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_WRIST_PITCH, -0.15));

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
  rclcpp::spin(std::make_shared<eve_ros2_examples::FullBodyExtremePublisher>());
  rclcpp::shutdown();
  return 0;
}
