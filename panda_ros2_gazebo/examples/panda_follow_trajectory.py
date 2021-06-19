# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from numpy import random
from scipy.spatial.transform import Rotation as R
import copy

# ROS2 Python API libraries
from rclpy.node import Node

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import GetParameters

# Panda kinematic model
from .scripts.models.panda import Panda, FingersAction
from .helpers.rviz_helper import RVizHelper

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

class PandaFollowTrajectory(Node):
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
        self._end_effector_current = self._panda.solve_fk(self._joint_states, remap=False)
        self._joint_targets = self._panda.solve_ik(self._end_effector_current)
        self._end_effector_target = copy.deepcopy(self._end_effector_current)

        # Create the RViz helper for visualizing the waypoints and trajectories
        self._rviz_helper = RVizHelper(self)

    def setup_joint_group_effort_controller(self):

        self._err = np.zeros((self._num_joints,))
        self._int_err = np.zeros((self._num_joints,))

        joint_controller_name = self.get_parameter('joint_controller_name')

        # create a service client to retrieve the PID gains from the joint_group_effort_controller
        self._parameter_getter_client = self.create_client(GetParameters, '/' + joint_controller_name + '/get_parameters')
        while not self._parameter_getter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._request = GetParameters.Request()
        self._response = GetParameters.Response()

        self._p_gains = np.zeros((self._num_joints,))
        self._i_gains = np.zeros((self._num_joints,))
        self._d_gains = np.zeros((self._num_joints,))
        for i, joint in enumerate(self._panda.joint_names):
            
            self._request.names = {'gains.' + joint + '.p', 'gains.' + joint + '.i', 'gains.' + joint + '.d'}

            self._response = self._parameter_getter_client.call_async(self._request)

            self._p_gains[i] = self._response.values[0]
            self._i_gains[i] = self._response.values[1]
            self._d_gains[i] = self._response.values[2]

    def callback_joint_states(self, joint_states):

        self._joint_states = joint_states

        # Calculate the end effector location relative to the base from forward kinematics
        self._end_effector_current = self._panda.solve_fk(self._joint_states)

        if self.end_effector_reached():
            # sample a new end effector target
            self.sample_end_effector_target()

        # Publish the end effector target and odometry messages
        self._end_effector_target_publisher.publish(self._end_effector_target)
        self._end_effector_pose_publisher.publish(self._end_effector_current)

        # Update the RViz helper and publish
        self._rviz_helper.publish(self._end_effector_current)

        self.joint_group_position_controller_callback()

    def joint_group_position_controller_callback(self) -> None:

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets)
        self._joint_commands_publisher.publish(msg)

    def joint_group_effort_controller_callback(self) -> None:

        # compute the effort from 
        err = self._joint_targets - self._joint_states.position
        self._int_err += self._control_dt * err
        deriv_err = (err - self._err) / self._control_dt

        self._effort = np.multiply(self._p_gains, err) + np.multiply(self._i_gains, self._int_err) + np.multiply(self._d_gains, deriv_err)

        self._err = err

        msg = Float64MultiArray()
        msg.data = self._effort.copy()
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

        return True # end_effector_reached

    def sample_end_effector_target(self) -> Odometry:

        # Sample a new target position...
        self._end_effector_target.header.stamp = self.get_clock().now().to_msg()
        self._end_effector_target.pose.pose.position.x = 0.5
        self._end_effector_target.pose.pose.position.y = 0.2 * np.cos(0.5 * self.get_clock().now().seconds_nanoseconds()[0])
        self._end_effector_target.pose.pose.position.z = 0.2 * np.sin(0.5 * self.get_clock().now().seconds_nanoseconds()[0]) + 0.6

        # ...and a new target orientation
        r = 0. # np.random.uniform(low=-np.pi/4, high=np.pi/4)
        p = 0. # np.random.uniform(low=-np.pi/4, high=np.pi/4)
        y = 0. # np.random.uniform(low=-np.pi/4, high=np.pi/4)

        quat_xyzw = R.from_euler('xyz', [r, p, y], degrees=False).as_quat()
        self._end_effector_target.pose.pose.orientation.x = quat_xyzw[0]
        self._end_effector_target.pose.pose.orientation.y = quat_xyzw[1]
        self._end_effector_target.pose.pose.orientation.z = quat_xyzw[2]
        self._end_effector_target.pose.pose.orientation.w = quat_xyzw[3]

        # self.get_logger().info("END EFFECTOR TARGET SET TO:\n[x, y z]=[{}, {}, {}]\n[r, p , y]=[{}, {}, {}]".format(
        #     self._end_effector_target.pose.pose.position.x,
        #     self._end_effector_target.pose.pose.position.y,
        #     self._end_effector_target.pose.pose.position.z,
        #     r, p, y
        # ))

        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        # Sample whether or not the fingers should be open or closed
        if random.random() > 0.5:
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.OPEN)[-2:]
        else:
            self._joint_targets[-2:] = self._panda.move_fingers(list(self._joint_targets), FingersAction.CLOSE)[-2:]