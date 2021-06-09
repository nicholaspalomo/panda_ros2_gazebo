import rclpy
from rclpy.node import Node

import numpy as np

from .python.models.panda import Panda, FingersAction
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import GetParameters

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

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
                ('initial_joint_angles', None)
            ]
        )

        # Create joint commands, end effector publishers; subscribe to joint state
        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value, 10)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value, 10)
        self._end_effector_pose_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_pose_topic').value, 10)
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self._control_dt = self.get_parameter('control_dt').value
        self._control_callback_timer = self.create_timer(self._control_dt, self.joint_group_position_controller_callback)

        # TODO: Implement logic to wait until first joint states measurement is received
        self._panda = Panda(self)

        # Sample an end effector target
        self._end_effector_target = Odometry()
        self.sample_end_effector_target()
        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        self._num_joints = self._panda.num_joints
        self._joint_states = JointState()
        self._joint_states.position = [0.] * self._num_joints
        self._joint_states.velocity = [0.] * self._num_joints
        self._joint_states.effort = [0.] * self._num_joints

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

        self.get_logger().info('JOINT POSITIONS: {}'.format(self._joint_states.position))

    def joint_group_position_controller_callback(self) -> None:

        if self.end_effector_reached():
            # sample a new end effector target
            self.sample_end_effector_target()
            self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        self.get_logger().info('JOINT TARGETS: {}'.format(self._joint_targets))

        msg = Float64MultiArray()
        msg.data = list(self._joint_targets)
        self._joint_commands_publisher.publish(msg)

    def joint_group_effort_controller_callback(self) -> None:
        
        if self.end_effector_reached():
            # sample a new end effector target
            self.sample_end_effector_target()
            self._joint_targets = self._panda.solve_ik(self._end_effector_target)

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
                            max_error_pos: float = 0.1,
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

        self.get_logger().info('END EFFECTOR POSITION: {}'.format(masked_current))
        self.get_logger().info('END EFFECTOR POSITION TARGET: {}'.format(masked_target))
        self.get_logger().info('TRANSLATIONAL POSITION ERR: {}'.format(masked_current - masked_target))
        self.get_logger().info('END EFFECTOR TRANSLATIONAL VELOCITY NORM: {}'.format(np.linalg.norm(velocity[:3])))
        self.get_logger().info('JOINT POSITION ERROR: {}'.format(self._joint_targets - np.array(self._joint_states.position)))

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

    def sample_end_effector_target(self) -> Odometry:

        self._end_effector_target.header.stamp = self.get_clock().now().to_msg()
        self._end_effector_target.pose.pose.position.x = np.random.uniform(low=0.25, high=0.6)
        self._end_effector_target.pose.pose.position.y = np.random.uniform(low=0.25, high=0.6)
        self._end_effector_target.pose.pose.position.z = np.random.uniform(low=0.25, high=0.6)

        r = np.random.uniform(low=-np.pi/4, high=np.pi/4)
        p = np.random.uniform(low=-np.pi/4, high=np.pi/4)
        y = np.random.uniform(low=-np.pi/4, high=np.pi/4)

        quat_xyzw = R.from_euler('xyz', [r, p, y], degrees=False).as_quat()
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