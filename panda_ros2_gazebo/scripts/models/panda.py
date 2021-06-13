# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import enum
import numpy as np
from ..rbd.idyntree import numpy
from typing import List
from scipy.spatial.transform import Rotation as R

# For using iDynTree inverse kinematics library
import idyntree.bindings as idt
from idyntree.bindings import KinDynComputations
from ..rbd import conversions
from ..rbd.idyntree import inverse_kinematics_nlp
from ..rbd.idyntree import kindyncomputations
from ..rbd.idyntree.helpers import FrameVelocityRepresentation

# ROS2 message and service data structures
import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance, Pose, Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Empty

class FingersAction(enum.Enum):

    OPEN = enum.auto()
    CLOSE = enum.auto()

class Panda():

    def __init__(self,
                 node_handle: Node,
                 position: List[float] = (0.0, 0.0, 0.0),
                 orientation: List[float] = (0, 0, 0, 1.0),
                 model_file: str = None):

        self._node_handle = node_handle
        self._base_2_end_effector_tf_buffer = Buffer(node=self._node_handle)
        self._base_2_end_effector_tf_listener = TransformListener(self._base_2_end_effector_tf_buffer, self._node_handle)

        # Try to get the transform every second.
        # If the transform is unavailable the timer callback will wait for it.
        self._output_timer = self._node_handle.create_timer(0.001, self.solve_fk)

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
        self._initial_joint_position_targets = self.move_fingers(self._initial_joint_position_targets, FingersAction.OPEN)

        self._fk = KinDynComputations()
        self._fk.setFrameVelocityRepresentation(idt.INERTIAL_FIXED_REPRESENTATION)
        self._fk.loadRobotModel(self._articulated_system)

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

        self._rot = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])

    async def solve_fk(self):
        """ Returns an end effector odometry message """

        when = rclpy.time.Time()
        try:
            # Suspends callback until transform becomes available
            end_effector_pose_in_base_frame: TransformStamped = await self._base_2_end_effector_tf_buffer.lookup_transform_async(self.base_frame, self.end_effector_frame, when)

            joint_positions = self._joint_states.position
            joint_velocities = self._joint_states.velocity

            # Update the robot state
            self._fk.setJointPos(numpy.FromNumPy.to_idyntree_dyn_vector(array=np.array(joint_positions)))
            
            self._end_effector_odom.header.stamp = self._node_handle.get_clock().now().to_msg()

            # compose the joint velocity vector for determining the end effector pose using the end effector Jacobian
            dofs = 6 + self._fk.model().getNrOfDOFs()
            J = idt.MatrixDynSize(6, dofs)
            self._fk.getFrameFreeFloatingJacobian(self.end_effector_frame, J)
            velocities = np.zeros((dofs,))
            for i, joint_idx in enumerate(self._arm_joint_names.keys()):
                velocities[i+6] = joint_velocities[joint_idx]

            end_effector_twist = np.matmul(J.toNumPy(), velocities[:, np.newaxis]).squeeze() # Double-check - is this in the right coordinate system?

            end_effector_pose_msg = PoseWithCovariance()
            pose = Pose()
            pose.position.x = end_effector_pose_in_base_frame.transform.translation.x
            pose.position.y = end_effector_pose_in_base_frame.transform.translation.y
            pose.position.z = end_effector_pose_in_base_frame.transform.translation.z

            pose.orientation.x = end_effector_pose_in_base_frame.transform.rotation.x
            pose.orientation.y = end_effector_pose_in_base_frame.transform.rotation.y
            pose.orientation.z = end_effector_pose_in_base_frame.transform.rotation.z
            pose.orientation.w = end_effector_pose_in_base_frame.transform.rotation.w

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

        except LookupException as e:

            self._node_handle.get_logger().info('FAILED TO GET TRANSFORM FROM {} TO {}'.format(self.base_frame, self.end_effector_frame))

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
        target_position = np.matmul(
            self._rot, 
            target_position[:, np.newaxis]).squeeze()

        target_orientation = R.from_quat([target_pose.pose.pose.orientation.x, target_pose.pose.pose.orientation.y, target_pose.pose.pose.orientation.z, target_pose.pose.pose.orientation.w]).as_matrix()
        target_orientation = np.matmul(
            self._rot, 
            target_orientation)
        quat_xyzw = R.from_matrix(target_orientation).as_quat()

        # quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()

        self._ik.update_transform_target(
            target_name=self._ik.get_active_target_names()[0],
            position=target_position,
            quaternion=conversions.Quaternion.to_wxyz(xyzw=quat_xyzw))

        # Run the IK
        self._ik.solve()

        return self._ik.get_reduced_solution().joint_configuration

    def reset_model(self) -> np.ndarray:

        self._node_handle.get_logger().info('RESETTING THE JOINT ANGLES...')
        self._node_handle.get_logger().info('INITIAL JOIN ANGLES:\n{}'.format(self._initial_joint_position_targets))

        return self._initial_joint_position_targets

    def move_fingers(self, joint_positions: List[float], action: FingersAction = FingersAction.OPEN):
        # Returns a vector of joint positions in which the fingers are 'OPEN' or 'CLOSED', depending on the value of 'action'

        for finger_idx, finger_limits in self.finger_joint_limits.items():
            if action is FingersAction.OPEN:
                joint_positions[finger_idx] = finger_limits[1]
                self._node_handle.get_logger().info('OPENING THE GRIPPER...')

            if action is FingersAction.CLOSE:
                joint_positions[finger_idx] = finger_limits[0]
                self._node_handle.get_logger().info('CLOSING THE GRIPPER...')

        return joint_positions.copy()

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
    def finger_joint_idxs(self):

        return self._finger_joint_names

    @property
    def base_frame(self) -> str:

        return self._node_handle.get_parameter('base_frame').value

    @property
    def end_effector_frame(self) -> str:

        return self._node_handle.get_parameter('end_effector_frame').value

    @property
    def end_effector_odom(self) -> Odometry:
        
        return self._end_effector_odom

    @property
    def model_file(self) -> str:

        model_file = os.path.join(self._node_handle.get_parameter('share_dir').value, self._node_handle.get_parameter('model_file').value)

        self._node_handle.get_logger().info('MODEL FILE NAME:\n{}'.format(model_file))

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
            self._node_handle.get_logger().info('MODEL SUCCESSFULLY LOADED. FILE NAME:\n{}'.format(urdf))

        return self._model_loader
