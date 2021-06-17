# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import sys

# ROS2 Python API libraries
import rclpy

# Panda example imports
from .examples.panda_teleop_control import PandaTeleopControl
from .examples.panda_follow_trajectory import PandaFollowTrajectory
from .examples.panda_pick_n_place import PandaPickAndPlace

def main(args=None):
    rclpy.init(args=args)

    if "follow" in sys.argv[2]:
        node = PandaFollowTrajectory()
    elif "picknplace" in sys.argv[2]:
        node = PandaPickAndPlace()
    else:
        node = PandaTeleopControl()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()