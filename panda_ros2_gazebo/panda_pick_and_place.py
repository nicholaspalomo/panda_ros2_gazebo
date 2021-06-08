from ntpath import join
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

import time
import numpy as np
import gym_ignition
from typing import List
from functools import partial
from models.panda import Panda, FingersAction
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from rcl_interfaces.srv import GetParameters

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

class PandaPickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose').value)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value
        self._control_callback_timer = self.create_timer(self._control_dt, self.joint_group_position_controller_callback)
        self._run_callback = self.create_timer(self._control_dt, self.run)

        self._panda = Panda(self.handle)

        self._num_joints = self._panda.num_joints
        self._joint_targets = self._panda.reset_model()
        self._joint_states = JointState()

        self._end_effector_target = Odometry()
        self._end_effector_target.header.seq = np.uint32(0)
        self._end_effector_target.header.stamp = rclpy.get_rostime()
        self._end_effector_target.header.frame_id = self._panda.base_frame
        self._end_effector_target.child_frame_id = self._panda.end_effector_frame
        self._end_effector_target.pose.pose.orientation.w = 1.0

        # publish the initial end effector target odometry message
        self._end_effector_target_publisher

    def setup_joint_group_effort_controller(self):

        self._err = np.zeros((self._num_joints,))
        self._int_err = np.zeros((self._num_joints,))

        joint_controller_name = self.get_parameter('joint_controller_name')

        # create a service client to retrieve the PID gains from the joint_group_effort_controller (until ROS2 has a joint_group_position_controller).
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
        
        self._panda.set_joint_states(joint_states)
        self._joint_states = joint_states

    def joint_group_position_controller_callback(self) -> None:

        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        msg = Float64MultiArray()
        msg.data = self._joint_targets.copy()
        self._joint_commands_publisher.publish(msg)

    def joint_group_effort_controller_callback(self) -> None:

        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        # compute the effort from 
        err = self._joint_targets - self._joint_states.position # TODO: Populate this in the joint_states callback
        self._int_err += self._control_dt * err
        deriv_err = (err - self._err) / self._control_dt

        self._effort = np.multiply(self._p_gains, err) + np.multiply(self._i_gains, self._int_err) + np.multiply(self._d_gains, deriv_err)

        self._err = err

        msg = Float64MultiArray()
        msg.data = self._effort.copy()
        self._joint_commands_publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def end_effector_reached(self,
                            max_error_pos: float = 0.01,
                            max_error_vel: float = 0.5,
                            mask: np.ndarray = np.array([1., 1., 1.])) -> bool:
        
        current_end_effector_pose = self._panda.end_effector_pose()
        position = np.array([
            current_end_effector_pose.pose.pose.position.x,
            current_end_effector_pose.pose.pose.position.y,
            current_end_effector_pose.pose.pose.position.z])
        velocity = np.array([
            current_end_effector_pose.twist.twist.linear.x,
            current_end_effector_pose.twist.twist.linear.y,
            current_end_effector_pose.twist.twist.linear.z,
            current_end_effector_pose.twist.twist.angular.x,
            current_end_effector_pose.twist.twist.angular.y,
            current_end_effector_pose.twist.twist.angular.z])
        target = np.array([
            self._end_effector_target.pose.pose.position.x,
            self._end_effector_target.pose.pose.position.y,
            self._end_effector_target.pose.pose.position.z])

        masked_target = mask * target
        masked_current = mask * position

        return np.linalg.norm(masked_current - masked_target) < max_error_pos and \
            np.linalg.norm(velocity[:3]) < max_error_vel

    def move_fingers(self,
                    action: FingersAction) -> None:
        # TODO: Include the fingers in the control loop
        for i, joint_name in enumerate(self._panda.joint_names):
            if self._panda.finger_joint_limits().get(joint_name) is not None:
                if action is FingersAction.OPEN:
                    self._joint_targets[i] = self._panda.finger_joint_limits[joint_name][1]

                if action is FingersAction.CLOSE:
                    self._joint_targets[i] = self._panda.finger_joint_limits[joint_name][0]

    # def get_unload_position(bucket: scenario_core.Model) -> np.ndarray:

    #     return bucket.base_position() + np.array([0, 0, 0.3])

    def run(self):

        if self.end_effector_reached():
            # sample a new end effector target
            self._end_effector_target.header.seq += 1
            self._end_effector_target.header.stamp = rclpy.get_rostime()
            self._end_effector_target.pose.pose.position.x = np.random.uniform(low=0.25, high=0.75)
            self._end_effector_target.pose.pose.position.y = np.random.uniform(low=0.25, high=0.75)
            self._end_effector_target.pose.pose.position.z = np.random.uniform(low=0.25, high=0.75)

            r = np.random.uniform(low=-np.pi/4, high=np.pi/4)
            p = np.random.uniform(low=-np.pi/4, high=np.pi/4)
            y = np.random.uniform(low=-np.pi/4, high=np.pi/4)

            quat_xyzw = R.from_euler('xyz', [r, p, y], from_degrees=False)
            self._end_effector_target.pose.pose.orientation.x = quat_xyzw[0]
            self._end_effector_target.pose.pose.orientation.x = quat_xyzw[1]
            self._end_effector_target.pose.pose.orientation.x = quat_xyzw[2]
            self._end_effector_target.pose.pose.orientation.x = quat_xyzw[3]

            print("End effector target set to [x, y z]=[{}, {}, {}], [r, p , y]=[{}, {}, {}]".format(
                self._end_effector_target.pose.pose.position.x,
                self._end_effector_target.pose.pose.position.y,
                self._end_effector_target.pose.pose.position.z,
                r, p, y
            ))

            self._end_effector_target_publisher.publish(self._end_effector_target)
            self._end_effector_pose_publisher.publish(self._panda.end_effector_pose())

def main(args=None):
    rclpy.init(args=args)

    pick_and_place = PandaPickAndPlace()

    rclpy.spin(pick_and_place)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pick_and_place.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()