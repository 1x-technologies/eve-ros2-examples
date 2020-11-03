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
#include "rclcpp/rclcpp.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/task_space_command.hpp"
#include "halodi_msgs/msg/reference_frame_name.hpp"


using namespace halodi_msgs::msg;
using std::placeholders::_1;

class Default_Pose_Publisher : public rclcpp::Node
{
public:
  Default_Pose_Publisher()
  : Node("default_pose_publisher")
  {
    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", 10);

    // subscribe to the tractory status topic
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10, std::bind(&Default_Pose_Publisher::status_callback, this, _1));

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
        // after the robot has returned to the default pose, we shutdown the node
        RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
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

    // we give the robot 3 seconds to return to its default configuration
    trajectory_msg.trajectory_points.push_back(default_target_(3));

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
  This generates an individual task space command
  */
  TaskSpaceCommand generate_task_space_command(int32_t body_frame_id, int32_t expressed_in_frame_id, bool use_z_up, double px_des, double py_des, double pz_des, double roll_des = 0.0, double pitch_des = 0.0, double yaw_des = 0.0)
  {
    TaskSpaceCommand ret_msg;

    ReferenceFrameName body_frame, expressed_in_frame;
    body_frame.frame_id = body_frame_id;
    expressed_in_frame.frame_id = expressed_in_frame_id;
    ret_msg.body_frame = body_frame;
    ret_msg.expressed_in_frame = expressed_in_frame;
    ret_msg.express_in_z_up = use_z_up;

    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Point position;
    position.x = px_des; position.y = py_des; position.z = pz_des;
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll_des, pitch_des, yaw_des);
    quat_msg = tf2::toMsg(quat_tf);
    pose.position = position; pose.orientation = quat_msg;
    ret_msg.pose = pose;

    return ret_msg;
  } 

  /* 
  The target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of a desired pelvis pose (Task Space) and desired joint configurations for the arms, 
  with no more than one desired value per joint.
  
  The desired time at which we want to reach the target is also specified.
  */
  WholeBodyTrajectoryPoint default_target_(int32_t t)
  {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    // A TaskSpaceCommand is composed of a pose of a 'body_frame', expressed relative to an 'expressed_in_frame'.
    // In this example we express a desired pelvis pose (the 'floating base'), relative to the frame at the centre of the wheeled base
    // which we generally want to have zero planar offset from when static.
    // The boolean express_in_z_up rotates the expressed_in_frame such that its unit_z axis is aligned with the gravity vector,
    // for this example this part is only effective if the wheeled base (ReferenceFrameName::BASE) is rested on a slope. 
    //
    // For more info on the TaskSpaceCommand msgs check out "halodi-messages/halodi_msgs/msg/TaskSpaceCommand.idl"
    ret_msg.task_space_commands.push_back(generate_task_space_command(ReferenceFrameName::PELVIS, ReferenceFrameName::BASE, true, 0.0, 0.0, 0.91));

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_SHOULDER_PITCH, 0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_SHOULDER_ROLL, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_SHOULDER_YAW, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_ELBOW_PITCH, -1.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_ELBOW_YAW, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_WRIST_PITCH, 0.2));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::LEFT_WRIST_ROLL, 0.0));

    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_PITCH, 0.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_ROLL, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_SHOULDER_YAW, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_PITCH, -1.3));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_ELBOW_YAW, 0.0));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_PITCH, 0.2));
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::RIGHT_WRIST_ROLL, 0.0));
    
    ret_msg.joint_space_commands.push_back(generate_joint_space_command(JointName::NECK_PITCH, 0.0));

    return ret_msg;
  }

  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  unique_identifier_msgs::msg::UUID uuid_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Default_Pose_Publisher>());
  rclcpp::shutdown();
  return 0;
}
