#!/usr/bin/env python3

import uuid
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from halodi_msgs.msg import ReferenceFrameName, TrajectoryInterpolation, WholeBodyTrajectory, WholeBodyTrajectoryPoint, TaskSpaceCommand, JointSpaceCommand, JointName



def generate_uuid_msg():
    """Generates a UUID msg based on the current time.

    Parameters: None
    
    Returns: UUID msg
    """
    
    return UUID(uuid=np.asarray(list(uuid.uuid1().bytes)).astype(np.uint8))
    
def generate_task_space_command_msg(body_frame_id, expressed_in_frame_id, xyzrpy, z_up=True):
    """Generates a task space command msg.

    Parameters:
    - body_frame_id (enum): body part to be moved, e.g. ReferenceFrameName.PELVIS
    - expressed_in_frame_id (enum): reference frame for body_frame_id, e.g. ReferenceFrameName.BASE
    - xyzrpy (array of 6 floats): desired pose of body_frame_id relative to expressed_in_frame_id, as a list/tuple/1D np.array of [ posX, posY, posZ, rotX, rotY, rotZ ]
    - z_up (bool): whether or not xyzrpy follows the Z-up co-ordinate convention. Default: True
    
    Returns: TaskSpaceCommand msg
    """
    
    msg_ = TaskSpaceCommand(express_in_z_up=z_up)
    msg_.body_frame.frame_id = body_frame_id
    msg_.expressed_in_frame.frame_id = expressed_in_frame_id
    
    msg_.pose.position.x = xyzrpy[0]
    msg_.pose.position.y = xyzrpy[1]
    msg_.pose.position.z = xyzrpy[2]
    quat_ = Rotation.from_euler("xyz", xyzrpy[3:]).as_quat() # Euler to quaternion
    msg_.pose.orientation.x = quat_[0]
    msg_.pose.orientation.y = quat_[1]
    msg_.pose.orientation.z = quat_[2]
    msg_.pose.orientation.w = quat_[3]
    
    return msg_

def generate_joint_space_command_msg(joint_id, q_desired, qd_desired=0.0, qdd_desired=0.0):
    """Generates a joint space command msg.
    This msg has additional gains fields. If you do not wish to set these yourself, please ensure that the use_default_gains bool is set to True.
    Msgs generated by this function have use_default_gains set to True.

    Parameters:
    - joint_id (enum): joint to be moved, e.g. JointName.NECK_PITCH
    - q_desired (float): desired final joint position
    - q_desired (float): desired final joint velocity. Default: 0.0
    - q_desired (float): desired final joint acceleration. Default: 0.0
    
    Returns: JointSpaceCommand msg
    """
    
    msg_ = JointSpaceCommand(joint=JointName(joint_id=joint_id), use_default_gains=True)
    msg_.q_desired = q_desired
    msg_.qd_desired = qd_desired
    msg_.qdd_desired = qdd_desired

    return msg_



class WholeBodyTrajectoryPublisher(Node):
    """A helper/example class to publish whole body trajectory messages.

    Constructor parameters:
    - initial_trajectory_msg (WholeBodyTrajectory): if not None, this is published first. Default: None
    - periodic_trajectory_msg (WholeBodyTrajectory): if not None, this is published on a loop upon completion of initial_trajectory_msg if it was provided. Default: None
    """
    
    def __init__(self, initial_trajectory_msg=None, periodic_trajectory_msg=None):
        super().__init__("whole_body_trajectory_example") # initialize the underlying Node with the name whole_body_trajectory_example
        self._publisher = self.create_publisher(WholeBodyTrajectory, "/eve/whole_body_trajectory", 10) # create a publisher with outbound queue size of 10
        self._subscriber = self.create_subscription(GoalStatus, "/eve/whole_body_trajectory_status", self.goal_status_cb, 10) # create a GoalStatus subscriber with inbound queue size of 10
        
        if initial_trajectory_msg is not None:
            initial_trajectory_msg.trajectory_id = generate_uuid_msg() # populate UUID
            self.get_logger().info("Publishing initial trajectory ...")
            self._publisher.publish(initial_trajectory_msg) # publish initial_trajectory_msg
        else:
            periodic_trajectory_msg.trajectory_id = generate_uuid_msg() # populate UUID
            self.get_logger().info("Publishing first periodic trajectory ...")
            self._publisher.publish(periodic_trajectory_msg) # publish periodic_trajectory_msg instead

        self._periodic_trajectory_msg = periodic_trajectory_msg # store periodic_trajectory_msg for re-publishing in goal_status_cb

    def goal_status_cb(self, msg):
        """GoalStatus callback. Logs/prints some statuses and re-pubishes periodic_trajectory_msg if it was provided to the constructor.

        Parameters:
        - msg (GoalStatus): msg from a GoalStatus subscription
        
        Returns: None
        """
    
        if msg.status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info("Goal accepted")
        elif msg.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled")
        elif msg.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted")
        elif msg.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            if self._periodic_trajectory_msg is not None:
                self.get_logger().info("Republishing periodic trajectory ...")
                self._periodic_trajectory_msg.trajectory_id = generate_uuid_msg()
                self._publisher.publish(self._periodic_trajectory_msg)
        
        
        
def run_warmup_loop(args=None):
    """An example function that moves all the joints in a repeated movement sequence.

    Parameters:
    - args (?): for rclpy.init(). Default: None

    Returns: None
    """

    NOMINAL_PELVIS_HEIGHT_ABOVE_BASE = 0.91
    cumulative_seconds_from_start_ = 0

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 3
    periodic_trajectory_pt_msg_1_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_)) # create a trajectory point msg, timestamped for 3 seconds in the future
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_PITCH, 0.5)) # append a desired joint position of 0.5 radians for the pitch of the right shoulder
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_ROLL, -0.1))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_YAW, -0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_PITCH, -0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_YAW, -0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, 0.0))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_PITCH, 0.5))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_ROLL, 0.1))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_YAW, 0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_PITCH, -0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_YAW, 0.2))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, 0.0))
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(generate_joint_space_command_msg(JointName.NECK_PITCH, 0.0))
    periodic_trajectory_pt_msg_1_.task_space_commands.append(generate_task_space_command_msg(ReferenceFrameName.PELVIS, ReferenceFrameName.BASE, [ 0.0, 0.0, NOMINAL_PELVIS_HEIGHT_ABOVE_BASE, 0.0, 0.0, np.deg2rad(0.0) ])) # append a desired task space pose for the pelvis WRT base [ posX, posY, posZ, roll, pitch, yaw ]

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_2_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_)) # create another trajectory point msg, 1 additional second in the future
    periodic_trajectory_pt_msg_2_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_ROLL, -1.5))
    periodic_trajectory_pt_msg_2_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_PITCH, -1.5))
    periodic_trajectory_pt_msg_2_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_ROLL, 1.5))
    periodic_trajectory_pt_msg_2_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_PITCH, -1.5))

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_3_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_YAW, 0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_YAW, 0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, 0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, 0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_YAW, -0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_YAW, -0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, 0.5))
    periodic_trajectory_pt_msg_3_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, -0.5))

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_4_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_4_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, -0.5))
    periodic_trajectory_pt_msg_4_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, -0.5))
    periodic_trajectory_pt_msg_4_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, -0.5))
    periodic_trajectory_pt_msg_4_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, 0.5))

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_5_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_PITCH, -1.5))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_PITCH, -1.5))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, 0.0))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_PITCH, -1.5))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_PITCH, -1.5))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_5_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, 0.0))

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 2
    periodic_trajectory_pt_msg_6_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_6_.task_space_commands.append(generate_task_space_command_msg(ReferenceFrameName.PELVIS, ReferenceFrameName.BASE, [ 0.1, -0.3, NOMINAL_PELVIS_HEIGHT_ABOVE_BASE, np.deg2rad(20.0), 0.0, np.deg2rad(30.0) ])) # move the pelvis 0.1m forward, -0.3m to the left. Roll 20 degrees and yaw 30 degrees

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 2
    periodic_trajectory_pt_msg_7_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_7_.task_space_commands.append(generate_task_space_command_msg(ReferenceFrameName.PELVIS, ReferenceFrameName.BASE, [ -0.1, 0.3, NOMINAL_PELVIS_HEIGHT_ABOVE_BASE, np.deg2rad(-20.0), 0.0, np.deg2rad(-30.0) ]))

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 3
    periodic_trajectory_pt_msg_8_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_8_.joint_space_commands.append(generate_joint_space_command_msg(JointName.NECK_PITCH, 0.3))
    periodic_trajectory_pt_msg_8_.task_space_commands.append(generate_task_space_command_msg(ReferenceFrameName.PELVIS, ReferenceFrameName.BASE, [ 0.0, 0.0, 0.65, 0.0, 0.0, np.deg2rad(0.0) ])) # do a squat

    # an extra message to make sure the trajectory ends in a safe position
    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_9_ = WholeBodyTrajectoryPoint(time_from_start=Duration(sec=cumulative_seconds_from_start_))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_PITCH, 0.5))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_ROLL, -0.1))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_YAW, -0.2))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_PITCH, -1.5))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_ELBOW_YAW, -0.2))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, 0.0))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_PITCH, 0.5))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_ROLL, 0.1))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_SHOULDER_YAW, 0.2))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_PITCH, -1.5))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_ELBOW_YAW, 0.2))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, 0.0))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, 0.0))
    periodic_trajectory_pt_msg_9_.joint_space_commands.append(generate_joint_space_command_msg(JointName.NECK_PITCH, 0.0))
    periodic_trajectory_pt_msg_9_.task_space_commands.append(generate_task_space_command_msg(ReferenceFrameName.PELVIS, ReferenceFrameName.BASE, [ 0.0, 0.0, NOMINAL_PELVIS_HEIGHT_ABOVE_BASE, 0.0, 0.0, np.deg2rad(0.0) ]))



    periodic_trajectory_msg_ = WholeBodyTrajectory(append_trajectory=False) # create a whole body trajectory msg that will override any trajectory currently being executed
    periodic_trajectory_msg_.interpolation_mode.value = TrajectoryInterpolation.MINIMUM_JERK_CONSTRAINED # choose an interpolation mode
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_1_) # pack in all the points created above
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_2_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_3_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_4_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_5_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_6_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_7_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_8_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_9_)


    
    rclpy.init(args=args) # initialize rclpy
    
    wbtp_ = WholeBodyTrajectoryPublisher(None, periodic_trajectory_msg_) # create the helper class
    rclpy.spin(wbtp_) # spin the node in the WholeBodyTrajectoryPublisher for blocking and pub/sub functionality

    wbtp_.destroy_node() # shut down the node
    rclpy.shutdown() # shut down rclpy



if __name__ == "__main__":
    run_warmup_loop()