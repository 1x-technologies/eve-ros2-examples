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
#include <boost/math/constants/constants.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/joint_name.hpp"

using namespace std::chrono_literals;
using namespace halodi_msgs::msg;
using std::placeholders::_1;

class Waving_Right_Hand_Publisher : public rclcpp::Node
{
public:
  Waving_Right_Hand_Publisher()
  : Node("waving_hand_trajectory_publisher")
  {
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", 10);
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10, std::bind(&Waving_Right_Hand_Publisher::topic_callback, this, _1));

    //Send the first trajectory command. The subscriber will send additional commands to loop the same command in the subscriber topic_callback
    uuid_msg_ = create_random_uuid();
    publish_joint_space_trajectory(uuid_msg_);

  }

private:
  void topic_callback(action_msgs::msg::GoalStatus::SharedPtr msg)
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
          std::cerr << "Yes" << std::endl;
          uuid_msg_ = create_random_uuid();
          publish_joint_space_trajectory(uuid_msg_);
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

  void publish_joint_space_trajectory(unique_identifier_msgs::msg::UUID uuid_msg)
  {
    WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    add_joint_target(&trajectory_msg, 3, target1());
    add_joint_target(&trajectory_msg, 5, target2());
    add_joint_target(&trajectory_msg, 7, target3());

    RCLCPP_INFO(this->get_logger(), "Sending whole_body_trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);

  }

  void add_joint_target(WholeBodyTrajectory * trajectory, int32_t t, WholeBodyTrajectoryPoint joint_target) {

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    joint_target.time_from_start = duration;

    trajectory->trajectory_points.push_back(joint_target);
  }

  JointSpaceCommand generate_joint_space_command(int32_t joint_id, double q_des, double qd_des = 0.0, double qdd_des = 0.0) {
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

  WholeBodyTrajectoryPoint target1() {
    WholeBodyTrajectoryPoint ret_msg;
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target2()  {
    WholeBodyTrajectoryPoint ret_msg;
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -0.6));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target3()  {
    WholeBodyTrajectoryPoint ret_msg;
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
  rclcpp::spin(std::make_shared<Waving_Right_Hand_Publisher>());
  rclcpp::shutdown();
  return 0;
}
