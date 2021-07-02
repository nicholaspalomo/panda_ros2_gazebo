# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import copy
from typing import List

# ROS2 message data structures
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class RVizHelper():
    def __init__(self,
                 node_handle):

        self._max_msgs: int = 100
        self._end_effector_target_pose_msg: List[PoseStamped] = []
        self._end_effector_trajectory_msg: Path = Path()

        # Create the publishers
        self._end_effector_target_pose_vis_pub = node_handle.create_publisher(PoseStamped, 'end_effector_target_vis', 10)
        self._end_effector_trajectory_vis_pub = node_handle.create_publisher(Path, 'end_effector_trajectory_vis', 10)

    def set_pose_msg(self, nav_msg: Odometry):

        if len(self._end_effector_target_pose_msg) > 1:
            self._end_effector_target_pose_msg[:-1] = copy.deepcopy(self._end_effector_target_pose_msg[1:])

        if len(self._end_effector_target_pose_msg) < self._max_msgs:
            self._end_effector_target_pose_msg.append(PoseStamped())

        self._end_effector_target_pose_msg[-1].header = copy.deepcopy(nav_msg.header)
        self._end_effector_target_pose_msg[-1].pose = copy.deepcopy(nav_msg.pose.pose)

        self._end_effector_trajectory_msg.header = copy.deepcopy(nav_msg.header)
        self._end_effector_trajectory_msg.poses = self._end_effector_target_pose_msg

    def publish(self, nav_msg: Odometry):

        self.set_pose_msg(nav_msg)

        self._end_effector_target_pose_vis_pub.publish(self._end_effector_target_pose_msg[-1])
        self._end_effector_trajectory_vis_pub.publish(self._end_effector_trajectory_msg)