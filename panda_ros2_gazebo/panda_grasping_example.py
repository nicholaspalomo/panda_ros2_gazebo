# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Example of Panda robot picking up an object and putting it in a specified location

import enum
import copy

# ROS2 Python API libraries
import rclpy
from rclpy.node import Node

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import SpawnEntity

# Panda kinematic model
from .scripts.models.panda import Panda, FingersAction

# Helper class for RViz visualization
from .rviz_helper import RVizHelper

class StateMachineAction(enum.Enum):

    GRAB = enum.auto() # activate the gripper; grab the box
    DELIVER = enum.auto() # take the box from the starting destination to the delivery destination
    RELEASE = enum.auto() # drop the box
    HOVER = enum.auto() # hover over the box
    HOME = enum.auto() # return to the home/neutal orientation

class PandaPickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

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

        # Declare service for spawning objects
        self._spawn_model = self.create_client(SpawnEntity, '/gazebo/spawn_model')

        # Create joint commands, end effector publishers; subscribe to joint state
        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value, 10)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose_topic').value, 10)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value

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
        self._panda.set_joint_states(self._joint_states)
        self._end_effector_current = copy.deepcopy(self._panda.end_effector_odom)
        self._joint_targets = self._panda.solve_ik(self._end_effector_current)
        self._end_effector_target = copy.deepcopy(self._end_effector_current)

        # Create the RViz helper for visualizing the waypoints and trajectories
        self._rviz_helper = RVizHelper(self)

    def callback_joint_states(self, joint_states):

        self._joint_states = joint_states
        self._panda.set_joint_states(self._joint_states)

        # Calculate the end effector location relative to the base from forward kinematics
        self._end_effector_current = copy.deepcopy(self._panda.end_effector_odom)

        if self.end_effector_reached():
            # sample a new end effector target
            # self.get_logger().info('END EFFECTOR TARGET REACHED!')
            self.go_to_next_target() # cycle to next action in state machine

        # Publish the end effector target and odometry messages
        self._end_effector_target_publisher.publish(self._end_effector_target)
        self._end_effector_pose_publisher.publish(self._end_effector_current)

        # Update the RViz helper and publish
        self._rviz_helper.publish(self._end_effector_current)

        self.joint_group_position_controller_callback()

        # TODO: Download the wood_cube_5cm model; Specify the state machine