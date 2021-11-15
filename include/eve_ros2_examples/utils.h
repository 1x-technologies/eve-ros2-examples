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

#pragma once

#include <algorithm>
#include <random>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/task_space_command.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"

namespace eve_ros2_examples {

/**
 * @brief createRandomUuidMsg Create a randomly generated UUID using the c++ <random> library
 * @return A UUID message for ROS2
 */
unique_identifier_msgs::msg::UUID createRandomUuidMsg() {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::array<uint8_t, 16> uuid{};
  for (int i = 0; i < 16; i++) {
    uuid[i] = dis(gen);
  }
  uuid_msg.uuid = uuid;
  return uuid_msg;
}

/**
 * @brief generateJointSpaceCommand This generates the individual single joint command
 * @param [in] joint_id Joint name identifier
 * @param [in] q_des Desired angle
 * @param [in] qd_des Desired velocity
 * @param [in] qdd_des Desired acceleration
 * @return JointSpaceCommand message
 */
halodi_msgs::msg::JointSpaceCommand generateJointSpaceCommand(int32_t joint_id, double q_des, double qd_des = 0.0,
                                                              double qdd_des = 0.0) {
  halodi_msgs::msg::JointSpaceCommand ret_msg;
  halodi_msgs::msg::JointName name;
  name.joint_id = joint_id;
  ret_msg.joint = name;
  ret_msg.q_desired = q_des;
  ret_msg.qd_desired = qd_des;
  ret_msg.qdd_desired = qdd_des;
  ret_msg.use_default_gains = true;
  return ret_msg;
}

/**
 * @brief generateTaskSpaceCommand This generates the individual single task command
 * @param [in] body_frame_id Body frame id
 * @param [in] expressed_in_frame_id Expressed in frame id
 * @param [in] use_z_up Use z-up or not
 * @param [in] px_des Desired x position
 * @param [in] py_des Desired y position
 * @param [in] pz_des Desired z position
 * @param [in] roll_des Desired roll
 * @param [in] pitch_des Desired pitch
 * @param [in] yaw_des Desired yaw
 * @return TaskSpaceCommand message
 */
halodi_msgs::msg::TaskSpaceCommand generateTaskSpaceCommand(int32_t body_frame_id, int32_t expressed_in_frame_id,
                                                            bool use_z_up, double px_des, double py_des, double pz_des,
                                                            double roll_des = 0.0, double pitch_des = 0.0,
                                                            double yaw_des = 0.0) {
  halodi_msgs::msg::TaskSpaceCommand ret_msg;

  halodi_msgs::msg::ReferenceFrameName body_frame, expressed_in_frame;
  body_frame.frame_id = body_frame_id;
  expressed_in_frame.frame_id = expressed_in_frame_id;
  ret_msg.body_frame = body_frame;
  ret_msg.expressed_in_frame = expressed_in_frame;
  ret_msg.express_in_z_up = use_z_up;

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Point position;
  position.x = px_des;
  position.y = py_des;
  position.z = pz_des;
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(roll_des, pitch_des, yaw_des);
  quat_msg = tf2::toMsg(quat_tf);
  pose.position = position;
  pose.orientation = quat_msg;
  ret_msg.pose = pose;

  return ret_msg;
}

/**
 * @brief genDefaultTarget Generate a whole body target with default angles for each joint
 * @param [in] t Time in seconds
 * @return WholeBodyTrajectoryPoint message
 */
halodi_msgs::msg::WholeBodyTrajectoryPoint genDefaultTarget(int32_t t) {
  halodi_msgs::msg::WholeBodyTrajectoryPoint ret_msg;

  builtin_interfaces::msg::Duration duration;
  duration.sec = t;
  ret_msg.time_from_start = duration;

  // A TaskSpaceCommand is composed of a pose of a 'body_frame', expressed relative to an 'expressed_in_frame'.
  // In this example we express a desired pelvis pose (the 'floating base'), relative to the frame at the centre of the
  // wheeled base which we generally want to have zero planar offset from when static. The boolean express_in_z_up
  // rotates the expressed_in_frame such that its unit_z axis is aligned with the gravity vector, for this example this
  // part is only effective if the wheeled base (ReferenceFrameName::BASE) is rested on a slope.
  //
  // For more info on the TaskSpaceCommand msgs check out "halodi-messages/halodi_msgs/msg/TaskSpaceCommand.idl"
  ret_msg.task_space_commands.push_back(generateTaskSpaceCommand(
      halodi_msgs::msg::ReferenceFrameName::PELVIS, halodi_msgs::msg::ReferenceFrameName::BASE, true, 0.0, 0.0, 0.91));

  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_SHOULDER_PITCH, 0.3));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_SHOULDER_ROLL, 0.0));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_SHOULDER_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_ELBOW_PITCH, -1.3));
  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_ELBOW_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_WRIST_PITCH, 0.2));
  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::LEFT_WRIST_ROLL, 0.0));

  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_SHOULDER_PITCH, 0.3));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_SHOULDER_ROLL, 0.0));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_SHOULDER_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_ELBOW_PITCH, -1.3));
  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_ELBOW_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(
      generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_WRIST_PITCH, 0.2));
  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::RIGHT_WRIST_ROLL, 0.0));

  ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(halodi_msgs::msg::JointName::NECK_PITCH, 0.0));

  return ret_msg;
}

/**
 * @brief genTarget Generate a whole body msg with default angles for each joint
 *
 * The target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of a desired pelvis
 * pose (Task Space) and desired joint configurations for the arms, with no more than one desired value per joint. The
 * desired time at which we want to reach the target is also specified.
 *
 * @param [in] uuid_msg The UUID of the message
 * @return WholeBodyTrajectory message with a single point decribing default pose
 */
halodi_msgs::msg::WholeBodyTrajectory genDefaultMsg(
    unique_identifier_msgs::msg::UUID uuid_msg = createRandomUuidMsg()) {
  // begin construction of the publsihed msg
  halodi_msgs::msg::WholeBodyTrajectory trajectory_msg;
  trajectory_msg.append_trajectory = false;
  // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint
  // velocities and accelerations between each waypoint
  trajectory_msg.interpolation_mode.value = halodi_msgs::msg::TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
  trajectory_msg.trajectory_id = uuid_msg;

  // we give the robot 3 seconds to return to its default configuration
  trajectory_msg.trajectory_points.push_back(genDefaultTarget(3));

  return trajectory_msg;
}

}  // namespace eve_ros2_examples
