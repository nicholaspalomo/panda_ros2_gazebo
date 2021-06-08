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
from scenario import core as scenario_core

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from rcl_interfaces.srv import GetParameters

# TODO: Publish the end effector odometry message. Make sure to update the end effector odometry on every call to the control callback

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

class PandaPickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        self._joint_commands_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('joint_control_topic').value)
        self._end_effector_target_publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value)
        self._end_effector_pose_subscriber = self.create_subscription()
        self._joint_states_subscriber = self.create_subscription(JointState, '/joint_states')
        self._control_dt = self.get_parameter('control_dt').value
        self._control_callback_timer = self.create_timer(self._control_dt, self.callback_pid)

        self._panda = Panda(self.handle)

        self._num_joints = self._panda.num_joints()
        self._err = np.zeros((self._num_joints,))
        self._int_err = np.zeros((self._num_joints,))
        self._joint_targets = self._panda.reset_model()

        # TODO: Get the PID gains from the parameter server
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
        for i, joint in enumerate(self._panda.joint_names()):
            
            self._request.names = {'gains.' + joint + '.p', 'gains.' + joint + '.i', 'gains.' + joint + '.d'}

            self._response = self._parameter_getter_client.call_async(self._request)

            self._p_gains[i] = self._response.values[0]
            self._i_gains[i] = self._response.values[1]
            self._d_gains[i] = self._response.values[2]

        self._end_effector_target = Odometry()

    def callback_pid(self) -> None:

        self._joint_targets = self._panda.solve_ik(self._end_effector_target)

        # compute the effort from 
        err = self._joint_targets - self._joint_positions
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
            current_end_effector_pose.twist.twist.angular.z,
        ])
        target = np.array([
            self._end_effector_target.pose.pose.position.x,
            self._end_effector_target.pose.pose.position.y,
            self._end_effector_target.pose.pose.position.z])

        masked_target = mask * target
        masked_current = mask * position

        return np.linalg.norm(masked_current - masked_target) < max_error_pos and \
            np.linalg.norm(velocity[:3]) < max_error_vel


    # def get_unload_position(bucket: scenario_core.Model) -> np.ndarray:

    #     return bucket.base_position() + np.array([0, 0, 0.3])


    def move_fingers(self,
                    panda: models.panda.Panda,
                    action: FingersAction) -> None:

        # Get the joints of the fingers
        finger1 = panda.get_joint(joint_name="panda_finger_joint1")
        finger2 = panda.get_joint(joint_name="panda_finger_joint2")

        if action is FingersAction.OPEN:
            finger1.set_position_target(position=finger1.position_limit().max)
            finger2.set_position_target(position=finger2.position_limit().max)

        if action is FingersAction.CLOSE:
            finger1.set_position_target(position=finger1.position_limit().min)
            finger2.set_position_target(position=finger2.position_limit().min)


    # def insert_bucket(world: scenario_gazebo.World) -> scenario_gazebo.Model:

    #     # Insert objects from Fuel
    #     uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    #     # Download the cube SDF file
    #     bucket_sdf = scenario_gazebo.get_model_file_from_fuel(
    #         uri=uri(org="GoogleResearch",
    #                 name="Threshold_Basket_Natural_Finish_Fabric_Liner_Small"),
    #         use_cache=False)

    #     # Assign a custom name to the model
    #     model_name = "bucket"

    #     # Insert the model
    #     assert world.insert_model(bucket_sdf,
    #                               scenario_core.Pose([0.68, 0, 1.02], [1., 0, 0, 1]),
    #                               model_name)

    #     # Return the model
    #     return world.get_model(model_name=model_name)


    # def insert_table(world: scenario_gazebo.World) -> scenario_gazebo.Model:

    #     # Insert objects from Fuel
    #     uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    #     # Download the cube SDF file
    #     bucket_sdf = scenario_gazebo.get_model_file_from_fuel(
    #         uri=uri(org="OpenRobotics",
    #                 name="Table"),
    #         use_cache=False)

    #     # Assign a custom name to the model
    #     model_name = "table"

    #     # Insert the model
    #     assert world.insert_model(bucket_sdf,
    #                               scenario_core.Pose_identity(),
    #                               model_name)

    #     # Return the model
    #     return world.get_model(model_name=model_name)


    # def insert_cube_in_operating_area(world: scenario_gazebo.World) -> scenario_gazebo.Model:

    #     # Insert objects from Fuel
    #     uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    #     # Download the cube SDF file
    #     cube_sdf = scenario_gazebo.get_model_file_from_fuel(
    #         uri=uri(org="openrobotics", name="wood cube 5cm"), use_cache=False)

    #     # Sample a random position
    #     random_position = np.random.uniform(low=[0.2, -0.3, 1.01], high=[0.4, 0.3, 1.01])

    #     # Get a unique name
    #     model_name = gym_ignition.utils.scenario.get_unique_model_name(
    #         world=world, model_name="cube")

    #     # Insert the model
    #     assert world.insert_model(
    #         cube_sdf, scenario_core.Pose(random_position, [1., 0, 0, 0]), model_name)

    #     # Return the model
    #     return world.get_model(model_name=model_name)

    def run(self):
        # ====================
        # INITIALIZE THE WORLD
        # ====================

        # Get the simulator and the world
        gazebo, world = gym_ignition.utils.scenario.init_gazebo_sim(
            step_size=0.001, real_time_factor=2.0, steps_per_run=1)

        # Open the GUI
        gazebo.gui()
        time.sleep(3)
        gazebo.run(paused=True)

        # Insert the Panda manipulator
        panda = gym_ignition_environments.models.panda.Panda(
            world=world, position=[-0.1, 0, 1.0])

        # Enable contacts only for the finger links
        panda.get_link("panda_leftfinger").enable_contact_detection(True)
        panda.get_link("panda_rightfinger").enable_contact_detection(True)

        # Process model insertion in the simulation
        gazebo.run(paused=True)

        # Monkey patch the class with finger helpers
        panda.open_fingers = partial(move_fingers, panda=panda, action=FingersAction.OPEN)
        panda.close_fingers = partial(move_fingers, panda=panda, action=FingersAction.CLOSE)

        # Add a custom joint controller to the panda
        add_panda_controller(panda=panda, controller_period=gazebo.step_size())

        # Populate the world
        table = insert_table(world=world)
        bucket = insert_bucket(world=world)
        gazebo.run(paused=True)

        # Create and configure IK for the panda
        ik_joints = [j.name() for j in panda.joints() if j.type is not scenario_core.JointType_fixed ]
        ik = get_panda_ik(panda=panda, optimized_joints=ik_joints)

        # Get some manipulator links
        finger_left = panda.get_link(link_name="panda_leftfinger")
        finger_right = panda.get_link(link_name="panda_rightfinger")
        end_effector_frame = panda.get_link(link_name="end_effector_frame")

        while True:

            # Insert a new cube
            cube = insert_cube_in_operating_area(world=world)
            gazebo.run(paused=True)

            # =========================
            # PHASE 1: Go over the cube
            # =========================

            print("Hovering")

            # Position over the cube
            position_over_cube = np.array(cube.base_position()) + np.array([0, 0, 0.4])

            # Get the joint configuration that brings the EE over the cube
            over_joint_configuration = solve_ik(
                target_position=position_over_cube,
                target_orientation=np.array(cube.base_orientation()),
                ik=ik)

            # Set the joint references
            assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)

            # Open the fingers
            panda.open_fingers()

            # Run the simulation until the EE reached the desired position
            while not end_effector_reached(position=position_over_cube,
                                        end_effector_link=end_effector_frame,
                                        max_error_pos=0.05,
                                        max_error_vel=0.5):
                gazebo.run()

            # Wait a bit more
            [gazebo.run() for _ in range(500)]

            # =======================
            # PHASE 2: Reach the cube
            # =======================

            print("Reaching")

            # Get the joint configuration that brings the EE to the cube
            over_joint_configuration = solve_ik(
                target_position=np.array(cube.base_position()) + np.array([0, 0, 0.04]),
                target_orientation=np.array(cube.base_orientation()),
                ik=ik)

            # Set the joint references
            assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)
            panda.open_fingers()

            # Run the simulation until the EE reached the desired position
            while not end_effector_reached(
                    position=np.array(cube.base_position()) + np.array([0, 0, 0.04]),
                    end_effector_link=end_effector_frame):

                gazebo.run()

            # Wait a bit more
            [gazebo.run() for _ in range(500)]

            # =======================
            # PHASE 3: Grasp the cube
            # =======================

            print("Grasping")

            # Close the fingers
            panda.close_fingers()

            # Detect a graps reading the contact wrenches of the finger links
            while not (np.linalg.norm(finger_left.contact_wrench()) >= 50.0 and
                    np.linalg.norm(finger_right.contact_wrench()) >= 50.0):
                gazebo.run()

            # =============
            # PHASE 4: Lift
            # =============

            print("Lifting")

            # Position over the cube
            position_over_cube = np.array(cube.base_position()) + np.array([0, 0, 0.4])

            # Get the joint configuration that brings the EE over the cube
            over_joint_configuration = solve_ik(
                target_position=position_over_cube,
                target_orientation=np.array(cube.base_orientation()),
                ik=ik)

            # Set the joint references
            assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)

            # Run the simulation until the EE reached the desired position
            while not end_effector_reached(position=position_over_cube,
                                        end_effector_link=end_effector_frame,
                                        max_error_pos=0.1,
                                        max_error_vel=0.5):
                gazebo.run()

            # Wait a bit more
            [gazebo.run() for _ in range(500)]

            # =====================================
            # PHASE 5: Place the cube in the bucket
            # =====================================

            print("Dropping")

            # Get the joint configuration that brings the EE over the bucket
            unload_joint_configuration = solve_ik(
                target_position=get_unload_position(bucket=bucket),
                target_orientation=np.array([0, 1.0, 0, 0]),
                ik=ik)

            # Set the joint references
            assert panda.set_joint_position_targets(unload_joint_configuration,
                                                    ik_joints)

            # Run the simulation until the EE reached the desired position
            while not end_effector_reached(
                    position=get_unload_position(bucket=bucket) +
                            np.random.uniform(low=-0.05, high=0.05, size=3),
                    end_effector_link=end_effector_frame,
                    max_error_pos=0.01,
                    max_error_vel=0.1,
                    mask=np.array([1, 1, 0])):

                gazebo.run()

            # Open the fingers
            panda.open_fingers()

            # Wait that both fingers are in not contact (with the cube)
            while finger_left.in_contact() or finger_right.in_contact():
                gazebo.run()

            # Wait a bit more
            [gazebo.run() for _ in range(500)]

            # Remove the cube
            world.remove_model(model_name=cube.name())

# It is always a good practice to close the simulator.
# In this case it is not required since above there is an infinite loop.
# gazebo.close()

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