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
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/reference_frame_name.hpp"
#include "halodi_msgs/msg/trajectory_interpolation.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/task_space_command.hpp"

using namespace std::chrono_literals;
using namespace halodi_msgs::msg;
using std::placeholders::_1;

class CameraCalibrationTrajectoryPublisher : public rclcpp::Node
{
public:
  CameraCalibrationTrajectoryPublisher()
  : Node("camera_calibration_trajectory_publisher")
  {
      // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is not up when publish_trajectory is called the first time.
      // Note: If the trajectory manager starts after this node, it'll execute immediatly.
      rclcpp::QoS latching_qos(1);
      latching_qos.transient_local();

    publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", latching_qos);
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10, std::bind(&CameraCalibrationTrajectoryPublisher::status_callback, this, _1));

    //Send the first trajectory command. The subscriber will send additional commands to loop the same command in the subscriber status_callback
    uuid_msg_ = create_random_uuid();
    publish_trajectory(uuid_msg_);

  }

private:
  void status_callback(const action_msgs::msg::GoalStatus::SharedPtr msg) const
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

  unique_identifier_msgs::msg::UUID create_random_uuid() const
  {
    //Create a random uuid to track msgs
    boost::uuids::random_generator gen; boost::uuids::uuid u = gen();
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::array<uint8_t, 16> uuid; std::copy(std::begin(u.data), std::end(u.data), uuid.begin());
    uuid_msg.uuid = uuid;
    return uuid_msg;
  }

  void publish_trajectory(unique_identifier_msgs::msg::UUID uuid_msg) const
  {
    WholeBodyTrajectory trajectory_msg;
    trajectory_msg.append_trajectory = false;
    trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
    trajectory_msg.trajectory_id = uuid_msg;

    // neck limits lower="-0.335" upper="0.506" 

    //Add targets for hand motions to pick up a box
    add_hand_and_neck_target(&trajectory_msg, 2, 0.45, -0.1, 0.5,  0.0,  -pi_/2.0,  pi_/3.5, ReferenceFrameName::RIGHT_HAND, 0.3);
    add_hand_and_neck_target(&trajectory_msg, 3, 0.55, -0.2, 0.4,  0.0,  -pi_/2.0,  pi_/3.0, ReferenceFrameName::RIGHT_HAND, 0.4);
    add_hand_and_neck_target(&trajectory_msg, 4, 0.65, -0.3, 0.2,  0.0,  -pi_/2.0,  pi_/2.0, ReferenceFrameName::RIGHT_HAND, 0.3);
    add_hand_and_neck_target(&trajectory_msg, 5, 0.55, -0.3, 0.4,  0.0,  -pi_/2.0,  pi_/3.0, ReferenceFrameName::RIGHT_HAND, 0.2);
    add_hand_and_neck_target(&trajectory_msg, 6, 0.45, -0.2, 0.35, 0.0,  -pi_/2.0,  pi_/3.5, ReferenceFrameName::RIGHT_HAND, 0.2);
    add_hand_and_neck_target(&trajectory_msg, 7, 0.45, -0.1, 0.25, 0.0,  -pi_/2.0,  pi_/3.5, ReferenceFrameName::RIGHT_HAND, 0.15);

    RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
    publisher_->publish(trajectory_msg);

  }

  void add_hand_and_neck_target(WholeBodyTrajectory * trajectory, int32_t t, double x, double y, double z, double yaw, double pitch, double roll, ReferenceFrameName::_frame_id_type frame, double neck_angle) const
  {
    WholeBodyTrajectoryPoint target;
    TaskSpaceCommand hand_command;

    //We specify the link frame to move and what reference frame the target pose will be described in here
    ReferenceFrameName::Type body_frame, reference_frame;
    body_frame.frame_id = frame;
    reference_frame.frame_id = ReferenceFrameName::PELVIS;
    hand_command.body_frame = body_frame;
    hand_command.expressed_in_frame = reference_frame;
    hand_command.express_in_z_up = true;

    //Setting a pose requires an x,y,z point and an orientation in the form of a quaternion
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Point point;
    point.x = x; point.y = y; point.z = z;
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    quat_msg = tf2::toMsg(quat_tf);
    pose.position = point; pose.orientation = quat_msg;
    hand_command.pose = pose;

    //We specify the desired time at which the hand should be at its target. Time is seconds after the start of the trajectory
    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    target.time_from_start = duration;

    //Add the hand command to the list of targets, add the target list to the trajectory points
    target.task_space_commands.push_back(hand_command);

    //Add neck angle target
    target.joint_space_commands.push_back(generate_joint_space_command(JointName::NECK_PITCH, neck_angle));

    trajectory->trajectory_points.push_back(target);
  }

  /*
  This generates the individual single joint command
  */
  JointSpaceCommand generate_joint_space_command(int32_t joint_id, double q_des, double qd_des = 0.0, double qdd_des = 0.0) const
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

  const double pi_ = boost::math::constants::pi<double>();
  rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
  mutable unique_identifier_msgs::msg::UUID uuid_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCalibrationTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
