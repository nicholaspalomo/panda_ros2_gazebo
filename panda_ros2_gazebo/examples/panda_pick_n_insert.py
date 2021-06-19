# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import enum
import copy
import numpy as np
import math
from typing import List
from scipy.spatial.transform import Rotation as R

# ROS2 Python API libraries
import rclpy
from rclpy.node import Node

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Panda kinematic model
from .scripts.models.panda import Panda, FingersAction

# Helper class for RViz visualization
from .helpers.rviz_helper import RVizHelper

# For spawning entities into Gazebo
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState
from rclpy.task import Future

from .helpers.helpers import get_xml_from_file

# TODO: Create a "task library" for things like "go to location", "pick up item", "drop item", etc. See gym-ignition repository for ideas
# TODO: First make sure you can spawn the URDFs correctly and still do the pick and place (pick and insert) before trying to clean up

MODEL_DATABASE_TEMPLATE = """\
<sdf version="1.4">
    <world name="default">
        <include>
            <uri>model://{}</uri>
            <static>{}</static>
        </include>
    </world>
</sdf>"""

class StateMachineAction(enum.Enum):

    GRAB = enum.auto() # activate the gripper; grab the box
    DELIVER = enum.auto() # take the box from the starting destination to the delivery destination
    RELEASE = enum.auto() # drop the box
    HOVER = enum.auto() # hover over the box
    HOME = enum.auto() # return to the home/neutal orientation

def check_service_call_completed(node: Node, response: Future):
    rclpy.spin_until_future_complete(node, response)
    if response.result() is not None:
        print('RESPONSE: %r' % response.result())

        return response.result()
    else:
        raise RuntimeError(
            'EXCEPTION WHILE CALLING SERVICE: %r' % response.exception())

class PandaPickAndInsert(Node):
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

        # Timestep counter for how much time the robot should wait before transitioning to the next state.
        self._wait = 0
        self._max_wait = 200

        # Create joint commands, end effector publishers; subscribe to joint state
        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value, 10)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose_topic').value, 10)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value
        base_link_frame = self.get_parameter('base_frame').value

        self._panda = Panda(self)
        self._num_joints = self._panda.num_joints

        self._joint_states: JointState = JointState()
        self._joint_states.velocity = [0.] * self._num_joints
        self._joint_states.effort = [0.] * self._num_joints

        # Publish initial joint states target
        self._joint_states.position = self._panda.reset_model()

        # Set an end effector target
        self._end_effector_current = self._panda.solve_fk(self._joint_states, remap=False)

        self._end_effector_target: Odometry = copy.deepcopy(self._end_effector_current)
        quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()
        self._end_effector_target.pose.pose.orientation.x = quat_xyzw[0]
        self._end_effector_target.pose.pose.orientation.y = quat_xyzw[1]
        self._end_effector_target.pose.pose.orientation.z = quat_xyzw[2]
        self._end_effector_target.pose.pose.orientation.w = quat_xyzw[3]
        self._initial_end_effector_target = copy.deepcopy(self._end_effector_target)

        self._joint_targets: List[float] = self._panda.solve_ik(self._end_effector_target)
        self._joint_targets_finish: List[float] = self._joint_targets.copy()
        self._joint_targets_start: List[float] = self._joint_targets.copy()

        # At the start, the fingers should be OPEN
        self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.OPEN)[-2:]

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets.copy())
        self._joint_commands_publisher.publish(msg)

        # Create the RViz helper for visualizing the waypoints and trajectories
        self._rviz_helper = RVizHelper(self)

        # Save the current state of the state machine
        self._state : StateMachineAction = StateMachineAction.HOME

        # Declare service for spawning objects
        self._spawn_model_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.get_logger().info("CONNECTING TO `/spawn_entity` SERVICE...")
        if not self._spawn_model_client.service_is_ready():
            self._spawn_model_client.wait_for_service()
            self.get_logger().info("...CONNECTED!")
        self.get_logger().info("[WARNING] ANY MODEL YOU WANT TO SPAWN USING THE GAZEBO `/spawn_entity` SERVICE MUST EXIST IN THE GAZEBO_MODEL_PATH VARIABLE, UNLESS AN ABSOLUTE PATH TO THE URDF IS PROVIDED.")

        self._sparkplug_pose: Pose = Pose()
        self._sparkplug_pose.position.x = 0.4 # [m]
        self._sparkplug_pose.position.y = 0.0 # [m]
        self._sparkplug_pose.position.z = 0.01 # [m]
        self._sparkplug_pose.orientation.w = 1.0

        # Initialize the sparkplug spawn request
        self._sparkplug_counter = 0
        self._spawn_model_request: SpawnEntity.Request = SpawnEntity.Request()
        self._spawn_model_request.reference_frame = base_link_frame

        self._set_model_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self._set_model_state_request: SetEntityState.Request = SetEntityState.Request()
        # TODO: SetEntityState not working
        # if not result.success:
        #     self._set_model_state_request.state.pose = copy.deepcopy(self._sparkplug_pose)
        #     self._set_model_state_request.state.name = "sparkplug"
        #     self._set_model_state_request.state.reference_frame = base_link_frame
        #     response = self._set_model_state_client.call_async(self._set_model_state_request)
        #     result = check_service_call_completed(self, response)

    def callback_joint_states(self, joint_states):

        self._joint_states = joint_states
        self._panda.set_joint_states(self._joint_states)

        # Calculate the end effector location relative to the base from forward kinematics
        self._end_effector_current = self._panda.solve_fk(self._joint_states)

        self.get_next_target() # cycle target to next action in state machine

        # Publish the end effector target and odometry messages
        self._end_effector_target_publisher.publish(self._end_effector_target)
        self._end_effector_pose_publisher.publish(self._end_effector_current)

        # Update the RViz helper and publish
        self._rviz_helper.publish(self._end_effector_current)

        self.joint_group_position_controller_callback()

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

    def joint_group_position_controller_callback(self) -> None:

        if self._state == StateMachineAction.DELIVER or self._state == StateMachineAction.GRAB:
            self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.CLOSE)[-2:]
        else:
            self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.OPEN)[-2:]

        if self._wait > 2 * self._max_wait:
            self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.OPEN)[-2:]

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets)
        self._joint_commands_publisher.publish(msg)

    def sample_new_sparkplug_pose(self):

        # Spawn the sparkplug insertion fixture into the simulation
        if self._sparkplug_counter == 0:
            self._spawn_model_request.xml = MODEL_DATABASE_TEMPLATE.format("sparkplug_socket", str(1))
            self._spawn_model_request.name = "sparkplug_socket"
            self._spawn_model_request.initial_pose = Pose()
            self._spawn_model_request.initial_pose.position.x = 0.25
            self._spawn_model_request.initial_pose.position.z = 0.2
            _ = self._spawn_model_client.call_async(self._spawn_model_request)

        # Set the XML for the call to spawn sparkplug in simulation
        self._spawn_model_request.xml = MODEL_DATABASE_TEMPLATE.format("sparkplug", str(0))

        # Sample new pose for the sparkplug
        random_position = np.random.uniform(low=[0.5, -0.2], high=[0.6, 0.2])
        self._sparkplug_pose.position.x = random_position[0]
        self._sparkplug_pose.position.y = random_position[1]
        self._sparkplug_pose.position.z = 0.01
        quat = R.from_euler(seq="y", angles=90, degrees=True).as_quat()
        self._sparkplug_pose.orientation.x = quat[0]
        self._sparkplug_pose.orientation.y = quat[1]
        self._sparkplug_pose.orientation.z = quat[2]
        self._sparkplug_pose.orientation.w = quat[3]

        # Spawn a new sparkplug
        self._spawn_model_request.name = "sparkplug{}".format(str(self._sparkplug_counter))
        self._spawn_model_request.initial_pose = copy.deepcopy(self._sparkplug_pose)
        _ = self._spawn_model_client.call_async(self._spawn_model_request)

        self._sparkplug_counter += 1

    def interp_joint_targets(self, joint_target: List[float], joint_targets_finish: List[float], joint_targets_start: List[float], num_steps):

        return [jt + (jf - js) / (num_steps - 1) for jt, jf, js in zip(joint_target, joint_targets_finish, joint_targets_start)].copy()

    def get_next_target(self):
        hover_height = 0.20
        sparkplug_height = 0.02
        grab_height = 0.05

        # if self._state == StateMachineAction.HOVER: # and self._wait < self._max_wait and self._wait > 0:
        #     quat_xyzw = R.from_euler(seq="yxz", angles=[180, -90, 0], degrees=True).as_quat()
        # if self._state == StateMachineAction.DELIVER:
        #     quat_xyzw = R.from_euler(seq="xyz", angles=[0, 180 ,0], degrees=True).as_quat() # [180, 180, 0] ? 
        # else:
        quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()
        self._end_effector_target.pose.pose.orientation.x = quat_xyzw[0]
        self._end_effector_target.pose.pose.orientation.y = quat_xyzw[1]
        self._end_effector_target.pose.pose.orientation.z = quat_xyzw[2]
        self._end_effector_target.pose.pose.orientation.w = quat_xyzw[3]

        # If the current state is "HOME" position and the target location has been reached
        if self._state == StateMachineAction.HOME: # and self.end_effector_reached():

            if self._wait < self._max_wait:
                # Keep waiting
                self._wait += 1

            else:
                self._wait = 0

                # Go to the location where the box is and hover above it
                self._state = StateMachineAction.HOVER

                # Set the end effector target to the sparkplug pose
                self.sample_new_sparkplug_pose()
                self._end_effector_target.pose.pose.position = copy.deepcopy(self._sparkplug_pose.position)
                self._end_effector_target.pose.pose.position.x += 0.012
                self._end_effector_target.pose.pose.position.z = hover_height # hover above the sparkplug

                self._joint_targets = self._panda.solve_ik(self._end_effector_target)

            return

        if self._state == StateMachineAction.HOVER:

            if self._wait == 0 and self.end_effector_reached():
                self._end_effector_target.pose.pose.position.z = grab_height

                self._joint_targets_start = self._joint_targets.copy()
                self._joint_targets_finish = self._panda.solve_ik(self._end_effector_target)

                self._wait += 1

            if self._wait < self._max_wait and self._wait > 0:
                # Lower the gripper
                self._end_effector_target.pose.pose.position.z = grab_height + (1 - self._wait/(self._max_wait-1)) * (hover_height - grab_height)

                self._joint_targets = self.interp_joint_targets(self._joint_targets, self._joint_targets_finish, self._joint_targets_start, self._max_wait)

                # Keep waiting
                self._wait += 1
            
            if self._wait == self._max_wait:
                self._wait = 0

                # Pick up the box
                self._state = StateMachineAction.GRAB

            return

        if self._state == StateMachineAction.GRAB:

            if self._wait == self._max_wait:
                self._end_effector_target.pose.pose.position.z = 2 * hover_height

                self._joint_targets_start = self._joint_targets.copy()
                self._joint_targets_finish = self._panda.solve_ik(self._end_effector_target)

            if self._wait < 2 * self._max_wait and self._wait >= self._max_wait:
                # Raise the gripper
                self._end_effector_target.pose.pose.position.z = grab_height + ((self._wait - self._max_wait)/(self._max_wait - 1)) * (2 * hover_height - grab_height)

                self._joint_targets = self.interp_joint_targets(self._joint_targets, self._joint_targets_finish, self._joint_targets_start, self._max_wait)

            self._wait += 1

            if self._wait == 2 * self._max_wait:

                # Set the location for the delivery target
                self._end_effector_target.pose.pose.position.x = 0.25 - 52.5 / 1000. + 35. / 1000. * ((self._sparkplug_counter - 1) % 4) # The holes are (max) 52.5 mm from centerline of sparkplug tray and the centers are spaced 35.0 mm apart. 0.25 m is the distance of the center of the tray from the coordinate system of the base.
                self._end_effector_target.pose.pose.position.y = - 52.5 / 1000. + 35. / 1000. * (((self._sparkplug_counter - 1) // 4) % 4)
                self._end_effector_target.pose.pose.position.z = 0.2 + 2 * hover_height

                self._joint_targets = self._joint_targets.copy()
                self._joint_targets_start = self._joint_targets.copy()
                self._joint_targets_finish = self._panda.solve_ik(self._end_effector_target)

                # Deliver the box to its location. Maybe hover above the location before dropping the box
                self._state = StateMachineAction.DELIVER

                self._wait = -3 * self._max_wait

            return

        if self._state == StateMachineAction.DELIVER:
            goal = 0.2 + 0.07
            grab_height = goal + 0.25 * sparkplug_height
            hover_height = goal + sparkplug_height

            if self._wait < 0:
                self._joint_targets = self.interp_joint_targets(self._joint_targets, self._joint_targets_finish, self._joint_targets_start, 3 * self._max_wait)

                self._wait += 1

            if self._wait == 0 and self.end_effector_reached():
                self._end_effector_target.pose.pose.position.x += 0.01 # hack
                self._end_effector_target.pose.pose.position.z = grab_height

                self._joint_targets_start = self._joint_targets.copy()
                self._joint_targets_finish = self._panda.solve_ik(self._end_effector_target)

                self._wait += 1

            if self._wait == 2 * self._max_wait:
                self._end_effector_target.pose.pose.position.z = hover_height

                self._joint_targets_start = self._joint_targets.copy()
                self._joint_targets_finish = self._panda.solve_ik(self._end_effector_target)

            if self._wait >= self._max_wait:
                self._wait += 1

            if self._wait < self._max_wait and self._wait > 0:
                # Lower the gripper
                self._end_effector_target.pose.pose.position.z = grab_height + (1 - self._wait/(self._max_wait-1)) * (hover_height - grab_height)

                self._joint_targets = self.interp_joint_targets(self._joint_targets, self._joint_targets_finish, self._joint_targets_start, self._max_wait)

                # Keep waiting
                self._wait += 1

            else:

                if self._wait < 3 * self._max_wait and self._wait > self._max_wait * 2:
                    # Raise the gripper
                    self._end_effector_target.pose.pose.position.z = grab_height + ((self._wait - 2 * self._max_wait) / (self._max_wait - 1)) * (hover_height - grab_height)

                    self._joint_targets = self.interp_joint_targets(self._joint_targets, self._joint_targets_finish, self._joint_targets_start, self._max_wait)

            if self._wait == 3 * self._max_wait:
                self._wait = 0

                # Return to the home position and spawn the box in a new location
                self._end_effector_target = copy.deepcopy(self._initial_end_effector_target)

                self._joint_targets = self._panda.solve_ik(self._end_effector_target)

                self._state = StateMachineAction.HOME

                print("RETURNING TO HOME")

            return