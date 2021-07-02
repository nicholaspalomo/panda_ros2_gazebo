from typing import List
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion

def quat_mult(q0, q1):
    # Function to multiply two quaternions of the form (i, j, k, re)

    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def rpy2quat(rpy: List[float], input_in_degrees=False) -> Quaternion:

    quat = R.from_euler('xyz', rpy, degrees=input_in_degrees).as_quat()

    out = Quaternion()
    out.x = quat[0]
    out.y = quat[1]
    out.z = quat[2]
    out.w = quat[3]

    return out # return roll-pith-yaw angles as quaternion

def get_xml_from_file(file: str) -> str:

    f = open(file, 'r')
    return f.read()