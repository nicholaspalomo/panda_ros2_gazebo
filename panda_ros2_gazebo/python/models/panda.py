# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os

from ..rbd.idyntree.helpers import FrameVelocityRepresentation

import enum
import numpy as np
from typing import List
import idyntree.bindings as idt
from ..rbd import conversions
from ..rbd.idyntree import inverse_kinematics_nlp
from ..rbd.idyntree import kindyncomputations
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance, Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Empty

# TODO: rather than using joint tags from the params.yaml, get the joint type directly from the iDynTree library
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
        initial_pose.translation.x = np.float_(position[0])
        initial_pose.translation.y = np.float_(position[1])
        initial_pose.translation.z = np.float_(position[2])
        initial_pose.rotation.x = np.float_(orientation[0])
        initial_pose.rotation.y = np.float_(orientation[1])
        initial_pose.rotation.z = np.float_(orientation[2])
        initial_pose.rotation.w = np.float_(orientation[3])
        self._base_position = initial_pose.translation
        self._base_orientation = initial_pose.rotation

        if model_file is None:
            self._urdf = self.model_file

            if not os.path.exists(self._urdf):
                raise FileNotFoundError(self._urdf)
        else:
            self._urdf = model_file

        # Get the model loader
        self._model_loader = idt.ModelLoader()
        
        self._articulated_system = self._get_model_loader(self._urdf).model()

        self._node_handle.get_logger().info('NUMBER OF JOINTS:\n{}'.format(self._articulated_system.getNrOfJoints()))

        # Get the joint names directly from the iDynTree model
        self._joint_names = []
        self._arm_joint_names = {}
        self._finger_joint_names = {} # fingers are excluded from the IK
        self._finger_joint_limits = []

        # Get the joints to exclude from the IK optimization
        self._finger_joint_tag = self._node_handle.get_parameter('finger_joint_tag').value
        self._arm_joint_tag = self._node_handle.get_parameter('arm_joint_tag').value

        for joint_idx in range(self._articulated_system.getNrOfJoints()):
            joint_name = self._articulated_system.getJointName(joint_idx)

            self._node_handle.get_logger().info('JOINT NAME: {} JOINT INDEX: {}'.format(joint_name, joint_idx))

            # Get the names of joints to be included in optimization. Joint names containing the 'exclude tag' (e.g. the fingers) should not be considered by the IK algorithm
            joint_obj = self._articulated_system.getJoint(joint_idx)
            if joint_obj.enablePosLimits(True): # can't enable joint limits for a fixed joint; so this method will return 'False'
                if self._finger_joint_tag in joint_name:
                    # Get the joint names from the model
                    self._joint_names.append(joint_name)

                    self._finger_joint_names[joint_idx] = joint_name

                    # Get the joint limits for the fingers (prismatic)
                    joint_obj = self._articulated_system.getJoint(joint_idx)
                    self._finger_joint_limits.append([joint_obj.getMinPosLimit(joint_idx), joint_obj.getMaxPosLimit(joint_idx)])
                elif self._arm_joint_tag in joint_name:
                    # Get the joint names from the model
                    self._joint_names.append(joint_name)

                    self._arm_joint_names[joint_idx] = joint_name

        self._finger_joint_limits = dict(zip(self._finger_joint_names, self._finger_joint_limits))

        # Initial joint targets
        self._initial_joint_position_targets = self._node_handle.get_parameter('initial_joint_angles').value

        # TODO: Get kinematic-dynamic computations to be able to compute frame poses from kinematics
        self._fk = kindyncomputations.KinDynComputations(self._urdf, considered_joints=list(self._arm_joint_names.values()), velocity_representation=FrameVelocityRepresentation.INERTIAL_FIXED_REPRESENTATION)

        # create containers for the joint position, velocity, effort
        self._joint_states = JointState()
        self._joint_states.position = self._initial_joint_position_targets
        self._joint_states.velocity = [0.] * len(self._joint_names)
        self._joint_states.effort = [0.] * len(self._joint_names)

        self._end_effector_odom = Odometry()
        self._end_effector_odom.header.stamp = self._node_handle.get_clock().now().to_msg()
        self._end_effector_odom.header.frame_id = self.base_frame
        self._end_effector_odom.child_frame_id = self.end_effector_frame
        self._end_effector_odom.pose.pose.orientation.w = 1.0 # so that orientation is a unit quaternion

        # Create the inverse kinematics model
        self._ik = self._get_panda_ik(self.joint_names)

        self._srv_gazebo_pause = self._node_handle.create_client(Empty, '/gazebo/pause_physics')
        self._srv_gazebo_unpause = self._node_handle.create_client(Empty, '/gazebo/unpause_physics')
        self._srv_set_model_state = self._node_handle.create_client(SetEntityState, '/gazebo/set_entity_state')

    def solve_fk(self, joint_positions: List[float]) -> Odometry:
        """ Returns an end effector odometry message """

        # Update the IK model
        self.update_robot_configuration(self._ik, joint_positions)

        # get the end effector pose from the kinematic model
        end_effector_pose_in_base_frame = self._fk.get_relative_transform(self.base_frame, self.end_effector_frame)

        # Get the end effector Jacobian
        J = self._fk.get_frame_jacobian(self.end_effector_frame)

        self._node_handle.get_logger().info('END EFFECTOR JACOBIAN:\n{}'.format(J))

        # compose the joint velocity vector for determining the end effector pose using the end effector Jacobian
        num_joints = self._articulated_system.getNrOfJoints() + 1 # '+1' here because this includes the fixed joint between the robot and the 'world', I guess...
        joint_velocities = np.zeros((num_joints,))
        j = 0
        for i in range(num_joints):
            if i in self._arm_joint_names.keys():
                joint_velocities[i] = self._joint_states.velocity[j]
                j += 1

            if i in self._finger_joint_names.keys():
                joint_velocities[i] = self._joint_states.velocity[j]
                j += 1

        self._node_handle.get_logger().info('JOINT VELOCITIES:\n{}'.format(joint_velocities))

        end_effector_twist = np.matmul(J, joint_velocities[:, np.newaxis]).squeeze()

        self._node_handle.get_logger().info('END EFFECTOR TWIST:\n{}'.format(end_effector_twist))
        self._node_handle.get_logger().info('END EFFECTOR POSE IN BASE FRAME:\n{}'.format(end_effector_pose_in_base_frame))

        end_effector_pose_msg = PoseWithCovariance()
        pose = Pose()
        pose.position.x = end_effector_pose_in_base_frame[0, -1]
        pose.position.y = end_effector_pose_in_base_frame[1, -1]
        pose.position.z = end_effector_pose_in_base_frame[2, -1]

        quat = R.from_matrix(end_effector_pose_in_base_frame[:3, :3]).as_quat()

        self._node_handle.get_logger().info('END EFFECTOR QUATERNION:\n{}'.format(quat))

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
        
        self._end_effector_odom.header.stamp = self._node_handle.get_clock().now().to_msg()

        return self._end_effector_odom

    def _get_panda_ik(self, optimized_joints: List[str]) -> \
        inverse_kinematics_nlp.InverseKinematicsNLP:

        # Create IK
        ik = inverse_kinematics_nlp.InverseKinematicsNLP(
            urdf_filename=self.model_file,
            considered_joints=optimized_joints,
            joint_serialization=self.joint_names)

        # Initialize IK
        ik.initialize(verbosity=1,
                    floating_base=False,
                    cost_tolerance=1E-8,
                    constraints_tolerance=1E-8,
                    base_frame=self.base_frame)

        self.update_robot_configuration(ik, self._joint_states.position)

        # Add the cartesian target of the end effector
        ik.add_target(frame_name=self.end_effector_frame,
                    target_type=inverse_kinematics_nlp.TargetType.POSE,
                    as_constraint=False)

        return ik

    def update_robot_configuration(self, ik: inverse_kinematics_nlp.InverseKinematicsNLP, joint_positions: List[float]):

        # Set the current configuration
        ik.set_current_robot_configuration(
            base_position=np.array([self.base_position.x, self.base_position.y, self.base_position.z]),
            base_quaternion=np.array([
                self.base_orientation.x, 
                self.base_orientation.y, 
                self.base_orientation.z, 
                self.base_orientation.w]),
            joint_configuration=np.array(joint_positions))

    def solve_ik(self, target_pose: Odometry) -> np.ndarray:

        target_position = np.array([target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_pose.pose.pose.position.z])

        # target_position = np.matmul(
        #     np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), 
        #     target_position[:, np.newaxis]).squeeze()

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

    @property
    def num_joints(self) -> int:

        return len(self._joint_names)

    @property
    def num_arm_joints(self) -> int:

        return len(list(self._arm_joint_names.values()))

    @property
    def num_end_effector_joints(self) -> int:

        return len(self._finger_joint_names)

    @property
    def base_position(self) -> Vector3:

        return self._base_position

    @property
    def base_orientation(self) -> Quaternion:

        return self._base_orientation

    @property
    def joint_states(self) -> JointState:

        return self._joint_states

    def set_joint_states(self, joint_states: JointState):

        self._joint_states = joint_states

    @property
    def joint_positions(self) -> List[float]:

        return self._joint_states.position

    @property
    def joint_names(self) -> List[str]:

        return self._joint_names

    @property
    def finger_joint_limits(self):

        return self._finger_joint_limits

    @property
    def base_frame(self) -> str:

        return self._node_handle.get_parameter('base_frame').value

    @property
    def end_effector_frame(self) -> str:

        return self._node_handle.get_parameter('end_effector_frame').value

    @property
    def model_file(self) -> str:

        model_file = os.path.join(os.getcwd(), self._node_handle.get_parameter('model_file').value)

        self._node_handle.get_logger().info('MODEL URDF:\n{}'.format(model_file))

        return model_file

    def _get_model_loader(self, urdf, joint_serialization=[]):
        
        # If the joint serialization function parameter is not empty, pass it as an argument to the model loader
        if joint_serialization:
            ok_load = self._model_loader.loadModelFromFile(urdf, joint_serialization)
        else:
            ok_load = self._model_loader.loadModelFromFile(urdf)

        if not ok_load:
            raise RuntimeError("Failed to load model")
        else:
            self._node_handle.get_logger().info('MODEL SUCCESSFULLY LOADED:\n{}'.format(urdf))

        return self._model_loader
