# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import enum
import numpy as np
from typing import List
from scenario import core as scenario
from gym_ignition.scenario import model_wrapper, model_with_file

from geometry_msgs.msg import Transform, Vector3, Quaternion

class FingersAction(enum.Enum):

    OPEN = enum.auto()
    CLOSE = enum.auto()

class Panda():

    def __init__(self,
                 position: List[float] = (0.0, 0.0, 0.0),
                 orientation: List[float] = (0, 0, 0, 1.0),
                 model_file: str = None):

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

        self._joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        ]
        self._joint_positions = np.zeros((len(self._joint_names),))

        # Get the default model description (URDF or SDF) allowing to pass a custom model
        if model_file is None:
            model_file = self.get_model_file()

        # Initial joint configuration - # TODO: Set model state here
        # model.to_gazebo().reset_joint_positions(
        #     [0, -0.785,0, -2.356, 0, 1.571, 0.785],
        #     [name for name in model.joint_names() if "panda_joint" in name])

        # TODO: Get the PID gains from the parameter server
        # From:
        # https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
        pid_gains_1000hz = {
            'panda_joint1': scenario.PID(50,    0,  20),
            'panda_joint2': scenario.PID(10000, 0, 500),
            'panda_joint3': scenario.PID(100,   0,  10),
            'panda_joint4': scenario.PID(1000,  0,  50),
            'panda_joint5': scenario.PID(100,   0,  10),
            'panda_joint6': scenario.PID(100,   0,  10),
            'panda_joint7': scenario.PID(10,  0.5, 0.1),
            'panda_finger_joint1': scenario.PID(100, 0, 50),
            'panda_finger_joint2': scenario.PID(100, 0, 50),
        }

    def base_position(self) -> Vector3:

        return self._base_position

    def base_orientation(self) -> Quaternion:

        return self._base_orientation

    def joint_positions(self) -> np.array:

        return self._joint_positions

    def joint_names(self) -> List[str]:

        return self._joint_names

    def base_frame(self) -> str:

        return "panda_link0"

    def get_model_file(self) -> str:
        
        # TODO: Get the model file from the server

        return 'filename'
