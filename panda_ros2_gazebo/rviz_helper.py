# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class RVizHelper():
    def __init__(self,
                 node_handle):

        self._node_handle = node_handle
        self._waypoints: List[int] = []