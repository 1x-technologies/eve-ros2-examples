#pragma once

#include <boost/uuid/uuid_generators.hpp>
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/task_space_command.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"

namespace eve_ros2_examples {

unique_identifier_msgs::msg::UUID create_random_uuid() {
  // Create a random uuid to track msgs
  boost::uuids::random_generator gen;
  boost::uuids::uuid u = gen();
  unique_identifier_msgs::msg::UUID uuid_msg;
  std::array<uint8_t, 16> uuid;
  std::copy(std::begin(u.data), std::end(u.data), uuid.begin());
  uuid_msg.uuid = uuid;
  return uuid_msg;
}

/*
This generates the individual single joint command
*/
halodi_msgs::msg::JointSpaceCommand generate_joint_space_command(int32_t joint_id, double q_des, double qd_des = 0.0,
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

/*
This generates an individual task space command
*/
halodi_msgs::msg::TaskSpaceCommand generate_task_space_command(int32_t body_frame_id, int32_t expressed_in_frame_id, bool use_z_up,
                                                               double px_des, double py_des, double pz_des, double roll_des = 0.0,
                                                               double pitch_des = 0.0, double yaw_des = 0.0) {
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

halodi_msgs::msg::WholeBodyTrajectoryPoint gen_default_target(int32_t t) {
  halodi_msgs::msg::WholeBodyTrajectoryPoint ret_msg;

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
  ret_msg.task_space_commands.push_back(generate_task_space_command(halodi_msgs::msg::ReferenceFrameName::PELVIS,
                                                                    halodi_msgs::msg::ReferenceFrameName::BASE, true, 0.0, 0.0, 0.91));

  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_PITCH, 0.3));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_ROLL, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_SHOULDER_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_ELBOW_PITCH, -1.3));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_ELBOW_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_WRIST_PITCH, 0.2));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::LEFT_WRIST_ROLL, 0.0));

  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_PITCH, 0.3));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_ROLL, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_SHOULDER_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_ELBOW_PITCH, -1.3));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_ELBOW_YAW, 0.0));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_WRIST_PITCH, 0.2));
  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::RIGHT_WRIST_ROLL, 0.0));

  ret_msg.joint_space_commands.push_back(generate_joint_space_command(halodi_msgs::msg::JointName::NECK_PITCH, 0.0));

  return ret_msg;
}

/*
The target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of a desired pelvis pose (Task Space) and
desired joint configurations for the arms, with no more than one desired value per joint.

The desired time at which we want to reach the target is also specified.
*/
halodi_msgs::msg::WholeBodyTrajectory gen_default_msg(unique_identifier_msgs::msg::UUID uuid_msg = create_random_uuid()) {
  // begin construction of the publsihed msg
  halodi_msgs::msg::WholeBodyTrajectory trajectory_msg;
  trajectory_msg.append_trajectory = false;
  // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint
  // velocities and accelerations between each waypoint
  trajectory_msg.interpolation_mode.value = halodi_msgs::msg::TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
  trajectory_msg.trajectory_id = uuid_msg;

  // we give the robot 3 seconds to return to its default configuration
  trajectory_msg.trajectory_points.push_back(gen_default_target(3));

  return trajectory_msg;
}

}  // namespace eve_ros2_examples