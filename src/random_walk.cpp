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
#include <functional>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status.hpp"

#include "eve_ros2_examples/utils.h"
#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"

namespace eve_ros2_examples {

using std::placeholders::_1;
using halodi_msgs::msg::JointName;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;

const int TIME_INCREMENT_ = 3;
const int NUM_TARGETS = 10;


class RandomWalk : public rclcpp::Node
{
public:
  RandomWalk()
  : Node("random_trajectory_publisher")
  {

      // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is not up when publish_trajectory is called the first time.
      // Note: If the trajectory manager starts after this node, it'll execute immediatly.
      rclcpp::QoS latching_qos(10);
      latching_qos.transient_local();


    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<halodi_msgs::msg::WholeBodyTrajectory>("/eve/whole_body_trajectory", latching_qos);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10, std::bind(&RandomWalk::status_callback, this, _1));

    // send the first trajectory command. The subscriber will send the commands again using the logic in status_callback(msg)
    uuid_msg_ = create_random_uuid();

    gen_random_numbers();
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
          // gen_random_numbers();
          publish_trajectory(uuid_msg_);
        }
        break;
      default:
        break;
    }
  }

  void gen_random_numbers() {
    std::default_random_engine generator;
    // std::normal_distribution<double> distribution(0.0,0.05);
    std::uniform_real_distribution<> distribution(-0.2, 0.2);
    std::uniform_real_distribution<> distribution_ph(-0.05, 0.05);

    traj_points_.clear();
    traj_point_default_ = std::vector<double>{0.91, 0.3, 0.0, 0.0, -1.3, 0.0, 0.2, 0.0, 0.3, 0.0, 0.0, -1.3, 0.0, 0.2, 0.0, 0.0};
    traj_points_.push_back(traj_point_default_);

    std::vector<double> traj_point_temp; 
    for (int i = 1; i < NUM_TARGETS; i++) {
      traj_point_temp.clear();
      for (std::vector<double>::size_type j = 0; j < traj_point_default_.size(); j++) {
        // traj_point_temp.push_back(traj_points_[i-1][j] + distribution(generator));
        if (j==0) { // pelvis height
          traj_point_temp.push_back(traj_points_[0][j] + distribution_ph(generator));
        } else {
          traj_point_temp.push_back(traj_points_[0][j] + distribution(generator));
        }
      }
      traj_points_.push_back(traj_point_temp);
    }
  }

  void publish_trajectory(unique_identifier_msgs::msg::UUID uuid_msg)
  {
    // begin construction of the publsihed msg
    halodi_msgs::msg::WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint 
    // velocities and accelerations between each waypoint
    trajectory_msg.interpolation_mode.value = halodi_msgs::msg::TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    // begin adding waypoint targets, the desired times {2, 4, 6} (ses) are provided in terms of
    // offset from time at which this published message is received
    int t = 0;
    for (int i = 0; i < NUM_TARGETS; i++) {
      trajectory_msg.trajectory_points.push_back(gen_target_from_vector(traj_points_[i], t+=4));
    }
    trajectory_msg.trajectory_points.push_back(gen_default_target(t+=4));

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);
  }

  /* 
  Each target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of desired joint configurations, 
  with no more than one desired value per joint.
  
  The desired time at which we want to reach these joint targets is also specified.
  */
  WholeBodyTrajectoryPoint gen_target_from_vector(std::vector<double>& vec, int t)
  {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    int ind = 0;
    ret_msg.task_space_commands.push_back(generate_task_space_command(halodi_msgs::msg::ReferenceFrameName::PELVIS, halodi_msgs::msg::ReferenceFrameName::BASE, true, 0.0, 0.0, vec[ind++]));

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_ROLL, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_YAW, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_ELBOW_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_ELBOW_YAW, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_WRIST_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_WRIST_ROLL, vec[ind++]));

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_ROLL, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_YAW, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_ELBOW_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_ELBOW_YAW, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_WRIST_PITCH, vec[ind++]));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_WRIST_ROLL, vec[ind++]));

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::NECK_PITCH, vec[ind++]));

    return ret_msg;
  }

  rclcpp::Publisher<halodi_msgs::msg::WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  unique_identifier_msgs::msg::UUID uuid_msg_;
  std::vector<double> traj_point_default_;
  std::vector<std::vector<double>> traj_points_;
};

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    rclcpp::spin(std::make_shared<eve_ros2_examples::RandomWalk>());
  }
  rclcpp::shutdown();
  return 0;
}
