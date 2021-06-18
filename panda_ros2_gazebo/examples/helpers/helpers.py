import numpy as np

def quat_mult(q0, q1):
    # Function to multiply two quaternions of the form (i, j, k, re)

    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def get_xml_from_file(file: str) -> str:

    f = open(file, 'r')
    return f.read()