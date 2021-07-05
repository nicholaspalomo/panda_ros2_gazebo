# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# TODO: Interpolate the joint positions while also clamping
# TODO: Switch to using a joint position interface
# TODO: Make sure to load the params yaml when this node is launched
# TODO: Define a custom service that takes end effector target (odometry) as request and returns response containing joint position trajectory
# TODO: Show the trajectory in RViz repeating in a loop

# TODO: This node should publish a tf2_msgs/TFMessage to the /tf topic for RViz to subscribe to

# ROS2 Python API libraries
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

# Service files for the planned trajectory
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from panda_msgs.srv import GetJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Header
from tf2_msgs.msg import TFMessage

# Panda kinematic model
from .scripts.models.panda import Panda, FingersAction
from .helpers.helpers import rpy2quat

# Misc
import copy
from typing import List
import numpy as np

class PandaTeleopControl2(Node):
    def __init__(self):
        super().__init__('panda')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_dt', None),
                ('max_joint_speed', None),
                ('joint_controller_name', None),
                ('joint_control_topic', None),
                ('joint_trajectory_topic', None),
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
        self._control_dt: np.float64 = 1. / self.get_parameter('control_dt').value
        self._max_joint_speed: np.float64 = self.get_parameter('max_joint_speed').value

        print("THE MAXIMUM JOINT SPEED IS: {}".format(self.get_parameter('max_joint_speed').value))

        self._set_ee_target_srv = self.create_service(GetJointTrajectory, 'set_ee_target', self.get_joint_targets_plan) # Example call: `ros2 service call /set_ee_target panda_msgs/srv/GetJointTrajectory "{x: 0.5, y: 0.5, z: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0}"`
        self._go_to_target_real_world_srv = self.create_service(Empty, '/go2target/robot', self.go_to_target_real_world)
        self._go_to_target_sim_srv = self.create_service(Empty, '/go2target/sim', self.go_to_target_sim)
        self._actuate_gripper_srv = self.create_service(Empty, 'actuate_gripper', self.actuate_gripper)

        # Create a Panda model object (for getting the IK)
        self._panda = Panda(self)
        self._current_joint_positions: np.ndarray = np.array(self._panda.reset_model())
        self._joint_trajectory_point: JointTrajectoryPoint = JointTrajectoryPoint()
        self._current_target_joint_trajectory: JointTrajectory = JointTrajectory()
        self._current_target_joint_setpoint: Float64MultiArray = Float64MultiArray()
        self._seq: int = 0

        #  Subscribe to the joint states
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)

        # Also create a joint states publisher so that we can update the RViz visualization
        self._joint_states_publisher = self.create_publisher(JointState, '/joint_states', 10)
        # Publish the initial joint states

        # DEBUG:
        print("INITIAL JOINT ANGLES: {}".format(list(self._current_joint_positions)))

        initial_joint_states_msg: JointState = JointState()
        initial_joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        initial_joint_states_msg.header.frame_id = self._panda.base_frame
        initial_joint_states_msg.name = self._panda.joint_names
        initial_joint_states_msg.position = list(self._current_joint_positions)
        initial_joint_states_msg.velocity = [0.] * self._panda.num_joints
        initial_joint_states_msg.effort = [0.] * self._panda.num_joints
        self._joint_states_publisher.publish(initial_joint_states_msg)

        self._idyn_joint_trajectory_pub = self.create_publisher(JointTrajectory, 'idyn/' + self.get_parameter('joint_trajectory_topic').value, 10)
        self._idyn_joint_group_position_controller_pub = self.create_publisher(Float64MultiArray, 'idyn/' + self.get_parameter('joint_control_topic').value, 10)
        self._rviz_trajectory_pub = self.create_publisher(Path, '/rviz/end_effector_target_trajectory', 10)
        self._joint_control_pub = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)

        self._robot_state_callback_timer: Timer = self.create_timer(self._control_dt, self.robot_state_timer_callback)

    def robot_state_timer_callback(self):

        joint_states_msg: JointState = JointState()
        joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        joint_states_msg.header.frame_id = self._panda.base_frame
        joint_states_msg.name = self._panda.joint_names
        joint_states_msg.position = list(self._current_joint_positions)
        joint_states_msg.velocity = [0.] * self._panda.num_joints
        joint_states_msg.effort = [0.] * self._panda.num_joints
        self._joint_states_publisher.publish(joint_states_msg)

    def go_to_target_real_world(self, request: Empty.Request, response: Empty.Response):

        # Publish the final joint target and the joint trajectory
        self._idyn_joint_trajectory_pub.publish(self._current_target_joint_trajectory)
        self._idyn_joint_group_position_controller_pub.publish(self._current_target_joint_setpoint)

        # publish to the joint position topic
        self._joint_control_pub.publish(self._current_target_joint_setpoint)

        return response

    def go_to_target_sim(self, request: Empty.Request, response: Empty.Response):

        # Publish the final joint target and the joint trajectory
        self._idyn_joint_trajectory_pub.publish(self._current_target_joint_trajectory)
        self._idyn_joint_group_position_controller_pub.publish(self._current_target_joint_setpoint)

        # Visualize the trajectory
        for point in self._current_target_joint_trajectory.points:
            self._current_joint_positions = point.positions

            joint_states_msg: JointState = JointState()
            joint_states_msg.header.stamp = self.get_clock().now().to_msg()
            joint_states_msg.header.frame_id = self._panda.base_frame
            joint_states_msg.name = self._panda.joint_names
            joint_states_msg.position = list(point.positions)
            joint_states_msg.velocity = [0.] * self._panda.num_joints
            joint_states_msg.effort = [0.] * self._panda.num_joints
            
            self._joint_states_publisher.publish(joint_states_msg)

        # self.get_logger().info("[SIMULATION] RETURNING TO HOME...")

        # # Return to home
        # joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        # joint_states_msg.header.frame_id = self._panda.base_frame
        # joint_states_msg.name = self._panda.joint_names
        # joint_states_msg.position = list(self._panda.reset_model())
        # joint_states_msg.velocity = [0.] * self._panda.num_joints
        # joint_states_msg.effort = [0.] * self._panda.num_joints
        
        # self._joint_states_publisher.publish(joint_states_msg)

        return response

    def callback_joint_states(self, joint_states: JointState):

        self._current_joint_positions = np.array(joint_states.position)
        self._panda.set_joint_states(joint_states)

    def actuate_gripper(self, request: Empty.Request, response: Empty.Response):

        # actuate the fingers - if OPEN, CLOSE them; otherwise OPEN them
        if self._panda.gripper_state == FingersAction.OPEN:
            self._current_target_joint_setpoint.data = self._panda.move_fingers(self._current_target_joint_setpoint.data, FingersAction.CLOSE)
        else:
            self._current_target_joint_setpoint.data = self._panda.move_fingers(self._current_target_joint_setpoint.data, FingersAction.OPEN)

    def _interp_joint_targets(self, current_joint_targets: List[float], joint_targets_goal: List[float], joint_targets_start: List[float], num_steps: int):

        return [jt + (jf - js) / (num_steps - 1) for jt, jf, js in zip(current_joint_targets, joint_targets_goal, joint_targets_start)].copy()

    def get_joint_targets_plan(self, request: GetJointTrajectory.Request, response: GetJointTrajectory.Response):

        # Use IK to get the desired joint positions at the target end effector pose
        end_effector_target: Odometry = Odometry()
        end_effector_target.pose.pose.position.x = request.x
        end_effector_target.pose.pose.position.y = request.y
        end_effector_target.pose.pose.position.z = request.z
        end_effector_target_quat_xyzw = rpy2quat([request.roll, request.pitch, request.yaw], input_in_degrees=True)
        end_effector_target.pose.pose.orientation.w = end_effector_target_quat_xyzw.w
        end_effector_target.pose.pose.orientation.x = end_effector_target_quat_xyzw.x
        end_effector_target.pose.pose.orientation.y = end_effector_target_quat_xyzw.y
        end_effector_target.pose.pose.orientation.z = end_effector_target_quat_xyzw.z
        
        target_joint_positions: np.ndarray = self._panda.solve_ik(end_effector_target)

        # Maximum joint position error should determine how long it takes to reach the target orientation
        max_joint_position_err = np.max(np.abs(self._current_joint_positions[:-2] - target_joint_positions[:-2]))
        actuation_time = max_joint_position_err / self._max_joint_speed

        self.get_logger().info("ACTUATION TIME: {}".format(actuation_time))

        # Calculate the number of points in the joint trajectory (separated by control_dt)
        num_trajectory_steps = np.floor(actuation_time / self._control_dt).astype(np.int) + 1

        self.get_logger().info("NUMBER OF TRAJECTORY STEPS: {}".format(num_trajectory_steps))

        # RViz marker looks for a nav_msgs/path. So we need to evaluate the forward kinematics at each point along the trajectory
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._panda.base_frame

        # Form the trajectory message
        self._current_target_joint_trajectory = JointTrajectory()
        targets = list(self._current_joint_positions).copy()
        for i in range(num_trajectory_steps):
            self._joint_trajectory_point.positions = targets
            self._joint_trajectory_point.time_from_start = rclpy.time.Duration(seconds=(np.float_(i+1) * self._control_dt), nanoseconds=0.0).to_msg()

            # Linearly interpolate the joint position targets along the trajectory
            targets = self._interp_joint_targets(
                targets,
                target_joint_positions,
                list(self._current_joint_positions),
                num_trajectory_steps)

            self._current_target_joint_trajectory.points.append(copy.deepcopy(self._joint_trajectory_point))

            # For visualization of EE path in RViz
            pose_stamped: PoseStamped = PoseStamped()
            end_effector_pose: Odometry = self._panda.solve_fk(JointState(position=list(self._current_target_joint_trajectory.points[-1].positions), velocity=([0.] * self._panda.num_joints), effort=([0.] * self._panda.num_joints)), remap=False)
            pose_stamped.header = end_effector_pose.header
            pose_stamped.pose = end_effector_pose.pose.pose
            path.poses.append(pose_stamped)

        # Populate the remaining fields of the response
        self._current_target_joint_trajectory.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._panda.base_frame)
        self._current_target_joint_trajectory.joint_names = self._panda.joint_names

        self.get_logger().info("LAST TRAJECTORY POINT: {}".format(self._current_target_joint_trajectory.points[-1]))

        self._current_target_joint_setpoint.data = self._current_target_joint_trajectory.points[-1].positions

        self._rviz_trajectory_pub.publish(path)

        self._seq += 1

        return response