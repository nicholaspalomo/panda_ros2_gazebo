# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from panda_ros2_gazebo.panda_ros2_gazebo.python.gym_ignition.rbd.idyntree.helpers import FrameVelocityRepresentation

from panda_ros2_gazebo.panda_ros2_gazebo.python.gym_ignition.rbd.idyntree.kindyncomputations import KinDynComputations

import enum
import numpy as np
from typing import List
from scenario import core as scenario
import idyntree.bindings as idt
from gym_ignition.rbd import conversions
from gym_ignition.rbd.idyntree import inverse_kinematics_nlp, kindyncomputations, helpers
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance, Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Empty

import rclpy
class FingersAction(enum.Enum):

    OPEN = enum.auto()
    CLOSE = enum.auto()

class Panda():

    def __init__(self,
                 node_handle,
                 position: List[float] = (0.0, 0.0, 0.0),
                 orientation: List[float] = (0, 0, 0, 1.0),
                 model_file: str = None):

        self._node_handle = node_handle

        # Initial pose
        initial_pose = Transform()
        initial_pose.translation.x = position[0]
        initial_pose.translation.y = position[1]
        initial_pose.translation.z = position[2]
        initial_pose.rotation.x = orientation[0]
        initial_pose.rotation.y = orientation[1]
        initial_pose.rotation.z = orientation[2]
        initial_pose.rotation.w = orientation[3]
        self._base_position = initial_pose.translation
        self._base_orientation = initial_pose.rotation

        if model_file is None:
            self._urdf = self.get_model_file()

            if not os.path.exists(self._urdf):
                raise FileNotFoundError(self._urdf)
        else:
            self._urdf = model_file
        
        self._articulated_system = self._get_model_loader(self._urdf).model()

        # Get the joint names directly from the iDynTree model
        self._joint_names = []
        self._arm_joint_names = []
        self._finger_joint_names = [] # fingers are excluded from the IK
        self._finger_joint_limits = []

        # Get the joints to exclude from the IK optimization
        self._exclude_tag = self._node_handle.get_parameter('exclude_tag').value

        for joint_idx in range(self._articulated_system.getNrOfJoints()):
            joint_name = self._articulated_system.getJointName(joint_idx)

            # Get the joint names from the model
            self._joint_names.append(joint_name)

            # Get the names of joints to be included in optimization. Joint names containing the 'exclude tag' (e.g. the fingers) should not be considered by the IK algorithm
            if self._exclude_tag in joint_name:
                self._finger_joint_names.append(joint_name)

                # Get the joint limits for the fingers (prismatic)
                min_lim = 0
                max_lim = 0
                joint_obj = self._articulated_system.getJoint(joint_name)
                if not joint_obj.getPosLimits(joint_idx, min_lim, max_lim):
                    ValueError("Failed to fetch finger joint limits.")
                self._finger_joint_limits.append([min_lim, max_lim])
            else:
                self._arm_joint_names.append(joint_name)
        self._finger_joint_limits = dict(zip(self._finger_joint_names, self._finger_joint_limits))

        # Get the reduced articulated system with the excluded joints and linkage branches removed
        self._reduced_articulated_system = self._get_model_loader(self._urdf, self._arm_joint_names).model()

        # Initial joint targets
        self._initial_joint_position_targets = self._node_handle.get_parameter('initial_joint_angles').value

        # TODO: Get kinematic-dynamic computations to be able to compute frame poses from kinematics
        self._fk = KinDynComputations(self._urdf, considered_joints=self._arm_joint_names, velocity_representation=FrameVelocityRepresentation.INERTIAL_FIXED_REPRESENTATION)

        # create containers for the joint position, velocity, effort
        self._joint_states = JointState()
        self._end_effector_odom = Odometry() # TODO: Get the end effector odometry directly from the IK model
        self._end_effector_odom.header.seq = np.uint32(0)
        self._end_effector_odom.header.stamp = rclpy.get_rostime()
        self._end_effector_odom.header.frame_id = self.base_frame()
        self._end_effector_odom.child_frame_id = self.end_effector_frame()

        # Create the inverse kinematics model
        self._ik = self.get_panda_ik(self._arm_joint_names)

        self._srv_gazebo_pause = self._node_handle.create_client(Empty, '/gazebo/pause_physics')
        self._srv_gazebo_unpause = self._node_handle.create_client(Empty, '/gazebo/unpause_physics')
        self._srv_set_model_state = self._node_handle.create_client(SetEntityState, '/gazebo/set_entity_state')

    def end_effector_pose(self) -> Odometry:
        """ Returns an end effector odometry message """

        # get the end effector pose from the kinematic model
        end_effector_pose_in_base_frame = self._fk.get_relative_transform(self.base_frame(), self.end_effector_frame())

        # Get the end effector Jacobian
        J = self._fk.get_frame_jacobian(self.end_effector_frame())

        end_effector_twist = np.matmul(J, self.joint_states.velocity[:-2, np.newaxis])

        end_effector_pose_msg = PoseWithCovariance()
        pose = Pose()
        pose.position.x = end_effector_pose_in_base_frame[0, -1]
        pose.position.y = end_effector_pose_in_base_frame[1, -1]
        pose.position.z = end_effector_pose_in_base_frame[2, -1]

        quat = R.from_matrix(end_effector_pose_in_base_frame[:3, :3])
        quat = R.as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        end_effector_pose_msg.pose = pose

        self._end_effector_odom.pose = end_effector_pose_msg

        end_effector_twist_msg = TwistWithCovariance()
        twist = Twist()
        velocity = Vector3()
        velocity.x = end_effector_twist[0]
        velocity.y = end_effector_twist[1]
        velocity.z = end_effector_twist[2]
        twist.linear = velocity
        velocity.x = end_effector_twist[3]
        velocity.y = end_effector_twist[4]
        velocity.z = end_effector_twist[5]
        twist.angular = velocity

        end_effector_twist_msg.twist = twist

        self._end_effector_odom.twist = end_effector_twist_msg

        return self._end_effector_odom

    def _get_panda_ik(self, optimized_joints: List[str]) -> \
        inverse_kinematics_nlp.InverseKinematicsNLP:

        # Create IK
        ik = inverse_kinematics_nlp.InverseKinematicsNLP(
            urdf_filename=self.get_model_file(),
            considered_joints=optimized_joints,
            joint_serialization=self.joint_names())

        # Initialize IK
        ik.initialize(verbosity=1,
                    floating_base=False,
                    cost_tolerance=1E-8,
                    constraints_tolerance=1E-8,
                    base_frame=self.base_frame())

        # Set the current configuration
        ik.set_current_robot_configuration(
            base_position=np.array([self.base_position().x, self.base_position().y, self.base_position().z]),
            base_quaternion=np.array([
                self.base_orientation().x, 
                self.base_orientation().y, 
                self.base_orientation().z, 
                self.base_orientation().w]),
            joint_configuration=self.joint_positions())

        # Add the cartesian target of the end effector
        ik.add_target(frame_name=self.end_effector_frame(),
                    target_type=inverse_kinematics_nlp.TargetType.POSE,
                    as_constraint=False)

        return ik

    def solve_ik(self, target_pose: Odometry) -> np.ndarray:

        target_position = np.array([target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_pose.pose.pose.position.z])

        quat_xyzw = np.array([target_pose.pose.pose.orientation.x, target_pose.pose.pose.orientation.y, target_pose.pose.pose.orientation.z, target_pose.pose.pose.orientation.w])

        quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()

        self._ik.update_transform_target(
            target_name=self._ik.get_active_target_names()[0],
            position=target_position,
            quaternion=conversions.Quaternion.to_wxyz(xyzw=quat_xyzw))

        # Run the IK
        self._ik.solve()

        return self._ik.get_reduced_solution().joint_configuration

    def reset_model(self) -> np.ndarray:

        # TODO: Reset the model's joints by pausing the Gazebo physics engine, setting the model state, and starting the simulation again. Also reset the joint angle targets
        # Use forward kinematics to get the end effector position corresponding to the joint angles

        return self._initial_joint_position_targets

    def num_joints(self) -> int:

        return len(self._joint_names)

    def num_arm_joints(self) -> int:

        return len(self._arm_joint_names)

    def num_end_effector_joints(self) -> int:

        return len(self._finger_joint_names)

    def base_position(self) -> Vector3:

        return self._base_position

    def base_orientation(self) -> Quaternion:

        return self._base_orientation

    def joint_states(self) -> JointState:

        return self._joint_states

    def joint_positions(self) -> np.ndarray:

        return self._joint_states.position

    def joint_names(self) -> List[str]:

        return self._joint_names

    def finger_joint_limits(self):

        return self._finger_joint_limits

    def base_frame(self) -> str:

        return self._node_handle.get_parameter('base_frame').value

    def end_effector_frame(self) -> str:

        return self._node_handle.get_parameter('end_effector_frame').value

    def get_model_file(self) -> str:

        return self._node_handle.get_parameter('model_file').value

    def _get_model_loader(self, urdf, joint_serialization=[]):

        # Get the model loader
        model_loader = idt.ModelLoader()
        
        # If the joint serialization function parameter is not empty, pass it as an argument to the model loader
        if joint_serialization:
            ok_load = model_loader.loadModelFromFile(urdf, joint_serialization)
        else:
            ok_load = model_loader.loadModelFromFile(urdf)

        if not ok_load:
            raise RuntimeError("Failed to load model")

        return model_loader
