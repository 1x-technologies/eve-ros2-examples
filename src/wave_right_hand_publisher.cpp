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

#include <memory>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/joint_name.hpp"

using namespace halodi_msgs::msg;
using std::placeholders::_1;

class WavingRightHandPublisher : public rclcpp::Node
{
public:
  WavingRightHandPublisher()
  : Node("waving_hand_trajectory_publisher")
  {

      // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is not up when publish_trajectory is called the first time.
      // Note: If the trajectory manager starts after this node, it'll execute immediatly.
      rclcpp::QoS latching_qos(10);
      latching_qos.transient_local();

    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", latching_qos);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10, std::bind(&WavingRightHandPublisher::status_callback, this, _1));

    // send the first trajectory command. The subscriber will send the commands again using the logic in status_callback(msg)
    uuid_msg_ = create_random_uuid();
    publish_trajectory(uuid_msg_);
  }

private:
  void status_callback(action_msgs::msg::GoalStatus::SharedPtr msg)
  {
    switch(msg->status){
      case 1:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_ACCEPTED");
        break;
      case 2:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_EXECUTING");
        break;
      case 4:
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
        //If the uuid of the received GoalStatus STATUS_SUCCEEDED Msg is the same as the uuid of the command we sent out, let's send another command
        if(msg->goal_info.goal_id.uuid==uuid_msg_.uuid){
          uuid_msg_ = create_random_uuid();
          publish_trajectory(uuid_msg_);
        }
        break;
      default:
        break;
    }
  }

  unique_identifier_msgs::msg::UUID create_random_uuid()
  {
    //Create a random uuid to track msgs
    boost::uuids::random_generator gen; boost::uuids::uuid u = gen();
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::array<uint8_t, 16> uuid; std::copy(std::begin(u.data), std::end(u.data), uuid.begin());
    uuid_msg.uuid = uuid;
    return uuid_msg;
  }

  void publish_trajectory(unique_identifier_msgs::msg::UUID uuid_msg)
  {
    // begin construction of the publsihed msg
    WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint
    // velocities and accelerations between each waypoint
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    // begin adding waypoint targets, the desired times {2, 4, 6} (ses) are provided in terms of
    // offset from time at which this published message is received
    trajectory_msg.trajectory_points.push_back(target1_(2));
    trajectory_msg.trajectory_points.push_back(target2_(4));
    trajectory_msg.trajectory_points.push_back(target3_(6));

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);
  }

  /*
  This generates the individual single joint command
  */
  JointSpaceCommand generate_joint_space_command(int32_t joint_id, double q_des, double qd_des = 0.0, double qdd_des = 0.0)
  {
    JointSpaceCommand ret_msg;
    JointName name;
    name.joint_id = joint_id;
    ret_msg.joint = name;
    ret_msg.q_desired = q_des;
    ret_msg.qd_desired = qd_des;
    ret_msg.qdd_desired = qdd_des;
    ret_msg.use_default_gains = true;
    return ret_msg;
  }

  /*
  Each target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of desired joint configurations,
  with no more than one desired value per joint.

  The desired time at which we want to reach these joint targets is also specified.
  */
  WholeBodyTrajectoryPoint target1_(int32_t t)
  {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target2_(int32_t t)
  {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -0.6));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target3_(int32_t t)
  {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -1.85));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  unique_identifier_msgs::msg::UUID uuid_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WavingRightHandPublisher>());
  rclcpp::shutdown();
  return 0;
}
