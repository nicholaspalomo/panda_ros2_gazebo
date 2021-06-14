# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Example of Panda robot picking up an object and putting it in a specified location

import enum
import copy
import numpy as np
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
from .rviz_helper import RVizHelper

# Misc
from .helpers import quat_mult

# For spawning entities into Gazebo
import xml
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

MODEL_DATABASE_TEMPLATE = """\
<sdf version="1.4">
    <world name="default">
        <include>
            <uri>model://{}</uri>
        </include>
    </world>
</sdf>"""

class StateMachineAction(enum.Enum):

    GRAB = enum.auto() # activate the gripper; grab the box
    DELIVER = enum.auto() # take the box from the starting destination to the delivery destination
    RELEASE = enum.auto() # drop the box
    HOVER = enum.auto() # hover over the box
    HOME = enum.auto() # return to the home/neutal orientation

def check_service_call_completed(node: Node, response):
    rclpy.spin_until_future_complete(node, response)
    if response.result() is not None:
        print('RESPONSE: %r' % response.result())
    else:
        raise RuntimeError(
            'EXCEPTION WHILE CALLING SERVICE: %r' % response.exception())

# TODO: Visualize the robot end configuration before the robot goes there. Maybe spawn a second instance of the panda robot for that and set the transparency to something reasonable (e.g. alpha = 0.1)

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
        self._spawn_model_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.get_logger().info("CONNECTING TO `/spawn_entity` SERVICE...")
        if not self._spawn_model_client.service_is_ready():
            self._spawn_model_client.wait_for_service()
            self.get_logger().info("...CONNECTED!")
        self.get_logger().info("[WARNING] ANY MODEL YOU WANT TO SPAWN USING THE GAZEBO `/spawn_entity` SERVICE MUST EXIST IN THE GAZEBO_MODEL_PATH VARIABLE, UNLESS AN ABSOLUTE PATH TO THE URDF IS PROVIDED.")

        self._spawn_model_request = SpawnEntity.Request()

        # Initialize and spawn the box into the simulation, can repeatedly change the location of it using "set model state"
        self._spawn_model_request.name = "cube"
        self._spawn_model_request.xml = MODEL_DATABASE_TEMPLATE.format('wood_cube_5cm')
        self._spawn_model_request.reference_frame = "panda_link0"
        self._spawn_model_request.initial_pose = Pose()
        self._spawn_model_request.initial_pose.position.x = 0.4 # [m]
        self._spawn_model_request.initial_pose.position.y = 0.0 # [m]
        self._spawn_model_request.initial_pose.position.z = 5/2 * 0.01 # [m]

        response = self._spawn_model_client.call_async(self._spawn_model_request)
        check_service_call_completed(self, response)

        # TODO: Visualize the cube in RViz using the mesh resource approach. See the answer given here for how to do that: https://answers.ros.org/question/217324/visualizing-gazebo-model-in-rviz/

        # Get the pose of the cube so that we can get the target pose for the end effector
        self._get_model_state_client = self.create_client(GetEntityState, '/get_entity_state')
        self._entity_state_request: GetEntityState.Request = GetEntityState.Request()
        self._entity_state_request.name = 'cube'
        self._entity_state_request.reference_frame = self.get_parameter('base_frame').value

        self._cube_pose: EntityState = self._get_model_state_client.call_async(self._entity_state_request)

        self._set_model_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self._set_model_state_request: SetEntityState.Request = SetEntityState.Request()

        # Timestep counter for how much time the robot should wait before transitioning to the next state.
        self._wait = 0
        self._max_wait = 1000

        # Create joint commands, end effector publishers; subscribe to joint state
        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value, 10)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose_topic').value, 10)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value

        self._panda = Panda(self)
        self._num_joints = self._panda.num_joints

        self._joint_states: JointState = JointState()
        self._joint_states.velocity = [0.] * self._num_joints
        self._joint_states.effort = [0.] * self._num_joints
        self._joint_states.effort = [0.] * self._num_joints

        # Publish initial joint states target
        msg = Float64MultiArray()
        self._joint_states.position = self._panda.reset_model()
        msg.data = self._joint_states.position
        self._joint_commands_publisher.publish(msg)

        # Set an end effector target
        self._panda.set_joint_states(self._joint_states)
        self._end_effector_current: Odometry = copy.deepcopy(self._panda.end_effector_odom)
        self._joint_targets: List[float] = copy.deepcopy(self._joint_states.position)
        self._end_effector_target: Odometry = copy.deepcopy(self._end_effector_current)

        # At the start, the fingers should be OPEN
        self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.OPEN)[-2:]

        # Create the RViz helper for visualizing the waypoints and trajectories
        self._rviz_helper = RVizHelper(self)

        # Save the current state of the state machine
        self._state : StateMachineAction = StateMachineAction.HOME

    def callback_joint_states(self, joint_states):

        self._joint_states = joint_states
        self._panda.set_joint_states(self._joint_states)

        # Calculate the end effector location relative to the base from forward kinematics
        self._end_effector_current = copy.deepcopy(self._panda.end_effector_odom)

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

        if self._state == StateMachineAction.HOME:
            end_effector_reached = np.linalg.norm(np.array(self._joint_states.position) - np.array(self._joint_targets)) < max_error_pos * 2.

            end_effector_reached = end_effector_reached and np.linalg.norm(np.array(self._joint_states.velocity)) < max_error_vel * 2.

            self.get_logger().info('JOINT POSITION ERROR NORM:\n{}'.format(np.linalg.norm(np.array(self._joint_states.position) - np.array(self._joint_targets))))

        else:
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

            # Check the orientation to see if the target has been reached
            orientation = np.array([
                self._end_effector_current.pose.pose.orientation.x,
                self._end_effector_current.pose.pose.orientation.y,
                self._end_effector_current.pose.pose.orientation.z,
                self._end_effector_current.pose.pose.orientation.w])
            target_inv = np.array([
                -self._end_effector_target.pose.pose.orientation.x,
                -self._end_effector_target.pose.pose.orientation.y,
                -self._end_effector_target.pose.pose.orientation.z,
                self._end_effector_target.pose.pose.orientation.w])
            target_inv /= np.linalg.norm(target_inv)

            # find the rotation difference between the two quaternions
            orientation_diff = quat_mult(orientation, target_inv)
            rot_vec = R.from_quat(orientation_diff).as_rotvec()

            end_effector_reached = end_effector_reached and (np.pi - np.linalg.norm(rot_vec) < max_error_rot) and (np.linalg.norm(velocity[3:]) < max_error_vel)

            # self.get_logger().info('ERROR NORM:\n{}'.format(np.linalg.norm(masked_current - masked_target)))
            # self.get_logger().info('END EFFECTOR POSE:\n{}'.format(self._end_effector_current.pose.pose.position))
            # self.get_logger().info('END EFFECTOR TARGET:\n{}'.format(self._end_effector_target.pose.pose.position))

        return end_effector_reached

    def joint_group_position_controller_callback(self) -> None:

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets)
        self._joint_commands_publisher.publish(msg)

    def sample_new_cube_pose(self):

        # TODO: Sample new pose for the cube
        random_position = np.random.uniform(low=[0.2, -0.3, 0.025], high=[0.4, 0.3, 0.025])
        self._cube_pose.pose.position.x = random_position[0]
        self._cube_pose.pose.position.y = random_position[1]
        self._cube_pose.pose.position.z = random_position[2]

        self._set_model_state_request.state = self._cube_pose
        response = self._set_model_state_client.call_async(self._set_model_state_request)
        check_service_call_completed(self, response)

    def get_next_target(self):

        # TODO: Need to update the end_effector_reached function to double check that the gripper state has been reached as well
        # TODO: In each state, need to define the end effector position and the length of time (or the number of time steps the robot should remain in that state once it has been reached)
        # TODO: Maybe in addition to an end_effector_reached function, you should also right a function to check if the task has been completed

        # If the current state is "HOME" position and the target location has been reached
        if self._state == StateMachineAction.HOME and self.end_effector_reached():

            if self._wait < self._max_wait:
                # Keep waiting
                self._wait += 1
            else:
                self._wait = 0

                # Set the end effector target to the cube pose
                self._end_effector_target.pose.pose = copy.deepcopy(self._cube_pose.pose)
                self._end_effector_target.pose.pose.position.z = 8. / 100. # hover 3 cm above the cube
                quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()
                self._end_effector_target.pose.pose.orientation.x = quat_xyzw[0]
                self._end_effector_target.pose.pose.orientation.y = quat_xyzw[1]
                self._end_effector_target.pose.pose.orientation.z = quat_xyzw[2]
                self._end_effector_target.pose.pose.orientation.w = quat_xyzw[3]

                self._joint_targets = self._panda.solve_ik(self._end_effector_target)

                # Go to the location where the box is and hover above it
                self._state = StateMachineAction.HOVER

            return

        if self._state == StateMachineAction.HOVER and self.end_effector_reached():

            if self._wait < self._max_wait:
                # Keep waiting
                self._wait += 1
            else:
                self._wait = 0

                # Lower the gripper and close it around the cube
                self._end_effector_target.pose.pose.position.z = 2.5 / 100.

                self._joint_targets = self._panda.solve_ik(self._end_effector_target)

                # Pick up the box
                self._state = StateMachineAction.GRAB

            return

        if self._state == StateMachineAction.GRAB and self.end_effector_reached():

            if self._wait < self._max_wait:
                # Keep waiting (give the fingers some time to finish closing)
                self._wait += 1

                self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.CLOSE)[-2:]

            else:
                self._wait = 0

                # set the height again to 3 cm above the ground
                self._end_effector_target.pose.pose.position.z = 8. / 100.

                self._joint_targets = self._panda.solve_ik(self._end_effector_target)

                # Deliver the box to its location. Maybe hover above the location before dropping the box
                self._state = StateMachineAction.DELIVER

            return

        if self._state == StateMachineAction.DELIVER and self.end_effector_reached():

            if self._wait < self._max_wait:
                # Keep waiting
                self._wait += 1

            else:
                # Go back to the home position and wait until another box has spawned
                self._joint_targets[-2:] = self._panda.move_fingers(self._joint_targets, FingersAction.OPEN)[-2:]

                if self._wait < 2 * self._max_wait:
                    # Keep waiting
                    self._wait += 1

                else:
                    self._wait = 0

                    # Return to the home position and spawn the box in a new location
                    self._joint_targets = self._panda.reset_model()

                    self._state = StateMachineAction.HOME
