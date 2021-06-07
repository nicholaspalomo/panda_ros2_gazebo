# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import sys
from panda_ros2_gazebo.panda_ros2_gazebo.python.gym_ignition.rbd.idyntree.helpers import FrameVelocityRepresentation

from panda_ros2_gazebo.panda_ros2_gazebo.python.gym_ignition.rbd.idyntree.kindyncomputations import KinDynComputations
sys.path.append('.')

import enum
import numpy as np
from typing import List
from scenario import core as scenario
import idyntree.bindings as idt
from gym_ignition.rbd import conversions
from gym_ignition.rbd.idyntree import inverse_kinematics_nlp, kindyncomputations, helpers
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, SetModelState
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

        # Get the joints to exclude from the IK optimization
        self._exclude_tag = self._node_handle.get_parameter('exclude_tag').value

        for joint_idx in range(self._articulated_system.getNrOfJoints()):
            joint_name = self._articulated_system.getJointName(joint_idx)

            # Get the joint names from the model
            self._joint_names.append(joint_name)

            # Get the names of joints to be included in optimization. Joint names containing the 'exclude tag' (e.g. the fingers) should not be considered by the IK algorithm
            if self._exclude_tag in joint_name:
                self._finger_joint_names.append(joint_name)
            else:
                self._arm_joint_names.append(joint_name)

        # Get the reduced articulated system with the excluded joints and linkage branches removed
        self._reduced_articulated_system = self._get_model_loader(self._urdf, self._arm_joint_names).model()

        # Initial joint targets
        self._initial_joint_position_targets = self._node_handle.get_parameter('initial_joint_angles').value

        # TODO: Get kinematic-dynamic computations to be able to compute frame poses from kinematics
        self._kindyn = KinDynComputations(self._urdf, considered_joints=self._joint_names, velocity_representation=FrameVelocityRepresentation.INERTIAL_FIXED_REPRESENTATION)

        # create containers for the joint position, velocity, effort
        self._joint_states = JointState()
        self._end_effector_odom = Odometry() # TODO: Get the end effector odometry directly from the IK model
        self._end_effector_odom.header.seq = np.uint32(0)
        self._end_effector_odom.header.stamp = rclpy.get_rostime()
        self._end_effector_odom.header.frame_id = self.base_frame()
        self._end_effector_odom.child_frame_id = self.end_effector_frame()

        # create the inverse kinematics model
        self._ik = self.get_panda_ik(self._arm_joint_names)

        # TODO: Get the PID gains from the parameter server
        # From:
        # https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
        # pid_gains_1000hz = {
        #     'panda_joint1': scenario.PID(50,    0,  20),
        #     'panda_joint2': scenario.PID(10000, 0, 500),
        #     'panda_joint3': scenario.PID(100,   0,  10),
        #     'panda_joint4': scenario.PID(1000,  0,  50),
        #     'panda_joint5': scenario.PID(100,   0,  10),
        #     'panda_joint6': scenario.PID(100,   0,  10),
        #     'panda_joint7': scenario.PID(10,  0.5, 0.1),
        #     'panda_finger_joint1': scenario.PID(100, 0, 50),
        #     'panda_finger_joint2': scenario.PID(100, 0, 50),
        # }

        # self._srv_gazebo_pause = self._node_handle.service_client(Empty, )
        # self._srv_gazebo_pause

    def end_effector_pose(self):

        # get the end effector pose from the kinematic model
        end_effector_pose_in_base_frame = self._kindyn.get_relative_transform(self.base_frame(), self.end_effector_frame())

        # TODO: populate the end effector odometry message


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

    def solve_ik(self, target_position: np.ndarray,
            target_orientation: np.ndarray,) -> np.ndarray:

        quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()

        self._ik.update_transform_target(
            target_name=self._ik.get_active_target_names()[0],
            position=target_position,
            quaternion=conversions.Quaternion.to_wxyz(xyzw=quat_xyzw))

        # Run the IK
        self._ik.solve()

        return self._ik.get_reduced_solution().joint_configuration

    def reset_model(self) -> np.ndarray:

        # TODO: Reset the model by pausing the Gazebo physics engine, setting the model state, and starting the simulation again

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
