# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import copy

# ROS2 Python API libraries
from rclpy.node import Node

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Panda kinematic model
from .scripts.models.panda import Panda, FingersAction
from .helpers.rviz_helper import RVizHelper

# For communication with the teleop node
from std_srvs.srv import Empty

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

class PandaTeleopControl(Node):
    def __init__(self):
        super().__init__('panda')

        # Declare parameters that are loaded from params.yaml to the parameter server
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_dt', None),
                ('joint_controller_name', None),
                ('joint_control_topic', None),
                ('end_effector_target_topic', None),
                ('end_effector_pose_topic', None),
                ('model_file', None),
                ('base_frame', None),
                ('end_effector_frame', None),
                ('arm_joint_tag', None),
                ('finger_joint_tag', None),
                ('initial_joint_angles', None),
                ('share_dir', None)
            ]
        )

        # Create joint commands, end effector publishers; subscribe to joint state
        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)
        self._end_effector_target_subscriber = self.create_subscription(Odometry, self.get_parameter('end_effector_target_topic').value, self.callback_end_effector_target, 10)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose_topic').value, 10)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value

        # Create the service for opening/closing the gripper
        self._actuate_gripper_service = self.create_service(Empty, 'actuate_gripper', self.callback_actuate_gripper)

        self._panda = Panda(self)
        self._num_joints = self._panda.num_joints

        self._joint_states = JointState()
        self._joint_states.velocity = [0.] * self._num_joints
        self._joint_states.effort = [0.] * self._num_joints

        # Publish initial joint states target
        msg = Float64MultiArray()
        self._joint_states.position = self._panda.reset_model()
        msg.data = self._joint_states.position
        self._joint_commands_publisher.publish(msg)

        # Set an end effector target
        self._end_effector_current = self._panda.solve_fk(self._joint_states, remap=False)
        self._joint_targets = self._panda.solve_ik(self._end_effector_current)
        self._end_effector_target = copy.deepcopy(self._end_effector_current)

        # Create the RViz helper for visualizing the waypoints and trajectories
        self._rviz_helper = RVizHelper(self)

    def callback_end_effector_target(self, end_effector_target: Odometry):

        self._end_effector_target = end_effector_target
        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

    def callback_actuate_gripper(self, request: Empty.Request, response: Empty.Response):

        if self._panda.gripper_state == FingersAction.CLOSE: # if the grippers are CLOSED, OPEN them
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.OPEN)[-2:]
        else: # if the grippers are OPEN, CLOSE them
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.CLOSE)[-2:]

        return Empty.Response()

    def callback_joint_states(self, joint_states):

        self._joint_states = joint_states

        if self._panda.gripper_state == FingersAction.OPEN: # if the grippers are OPEN, keep them OPEN
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.OPEN)[-2:]
        else: # if the grippers are CLOSED, keep them CLOSED
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.CLOSE)[-2:]

        # Calculate the end effector location relative to the base from forward kinematics
        self._end_effector_current = self._panda.solve_fk(self._joint_states)

        # if self.end_effector_reached():
        #     self.get_logger().info('END EFFECTOR TARGET REACHED!')

        # Publish the end effector target and odometry messages
        self._end_effector_pose_publisher.publish(self._end_effector_current)

        # Update the RViz helper and publish
        self._rviz_helper.publish(self._end_effector_current)

        self.joint_group_position_controller_callback()

    def joint_group_position_controller_callback(self) -> None:

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets)
        self._joint_commands_publisher.publish(msg)

    def end_effector_reached(self,
                            max_error_pos: float = 0.05,
                            max_error_rot: float = 0.05,
                            max_error_vel: float = 0.1,
                            mask: np.ndarray = np.array([1., 1., 1.])) -> bool:

        # Check the position to see if the target has been reached
        position = np.array([
            self._end_effector_current.pose.pose.position.x,
            self._end_effector_current.pose.pose.position.y,
            self._end_effector_current.pose.pose.position.z])
        velocity = np.array([
            self._end_effector_current.twist.twist.linear.x,
            self._end_effector_current.twist.twist.linear.y,
            self._end_effector_current.twist.twist.linear.z,
            self._end_effector_current.twist.twist.angular.x,
            self._end_effector_current.twist.twist.angular.y,
            self._end_effector_current.twist.twist.angular.z])
        target = np.array([
            self._end_effector_target.pose.pose.position.x,
            self._end_effector_target.pose.pose.position.y,
            self._end_effector_target.pose.pose.position.z])

        masked_target = mask * target
        masked_current = mask * position

        end_effector_reached = (np.linalg.norm(masked_current - masked_target) < max_error_pos) and \
            (np.linalg.norm(velocity[:3]) < max_error_vel)

        return end_effector_reached