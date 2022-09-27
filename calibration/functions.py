import numpy as np
import math

# Inverse the matrix
def matrix_inverse(M):
    M[0:3, 0:3] = np.transpose(M[0:3, 0:3])
    M_neg = np.negative(M[0:3, 0:3])
    M[0:3, 3] = np.dot(M_neg, M[0:3, 3])
    return M

# Rotate around the z axis
def rotZ(angle):
    R = np.zeros([4,4])

    R[0, 0] = math.cos(angle); R[0, 1] = -math.sin(angle);
    R[1, 0] = math.sin(angle); R[1, 1] = math.cos(angle);
    R[2, 2] = 1;
    R[3, 3] = 1;

    return R

# Converts rotation matrix to Quaternion
def rot2q(R):
    q = np.zeros(4)
    q[0] = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
    q[1] = math.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2])
    q[2] = math.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2])
    q[3] = math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
    q = q * 0.5

    max = np.argmax(q)

    if max == 0:
        q[1] = (0.25 / q[0]) * (R[0, 1] + R[1, 0])
        q[2] = (0.25 / q[0]) * (R[0, 2] + R[2, 0])
        q[3] = (0.25 / q[0]) * (R[2, 1] - R[1, 2])
    elif max == 1:
        q[0] = (0.25 / q[1]) * (R[0, 1] + R[1, 0])
        q[2] = (0.25 / q[1]) * (R[1, 2] + R[2, 1])
        q[3] = (0.25 / q[1]) * (R[0, 2] - R[2, 0])
    elif max == 2:
        q[0] = (0.25 / q[2]) * (R[0, 2] + R[2, 0])
        q[1] = (0.25 / q[2]) * (R[1, 2] + R[2, 1])
        q[3] = (0.25 / q[2]) * (R[1, 0] - R[0, 1])
    else:
        q[0] = (0.25 / q[3]) * (R[2, 1] - R[1, 2])
        q[1] = (0.25 / q[3]) * (R[0, 2] - R[2, 0])
        q[2] = (0.25 / q[3]) * (R[1, 0] - R[0, 1])

    return q

# Conversion of quaternion to rotation
def q2rot(q):
    R = np.zeros(shape=(3,3))
    R[0, 0] = q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2;
    R[1, 1] = q[3]**2 - q[0]**2 + q[1]**2 - q[2]**2;
    R[2, 2] = q[3]**2 - q[0]**2 - q[1]**2 + q[2]**2;

    R[0, 1] = 2 * (-q[3] * q[2] + q[0] * q[1]);
    R[1, 0] = 2 * (q[3] * q[2] + q[0] * q[1]);

    R[0, 2] = 2 * (q[3] * q[1] + q[0] * q[2]);
    R[2, 0] = 2 * (-q[3] * q[1] + q[0] * q[2]);

    R[1, 2] = 2 * (-q[3] * q[0] + q[1] * q[2]);
    R[2, 1] = 2 * (q[3] * q[0] + q[1] * q[2]);

    return R

# Skew matrix
def skewqA(q):
    Q = np.zeros(shape=(4,4))

    Q[0] = [q[3], -1 * q[0], -1 * q[1], -1 * q[2]]
    Q[1] = [q[0], q[3], -1 * q[2], q[1]]
    Q[2] = [q[1], q[2], q[3], -1 * q[0]]
    Q[3] = [q[2], -1 * q[1], q[0], q[3]]

    return Q

# Create skew symmetric matrix from vector QB
def skewqB(q):
    Q = np.zeros(shape=(4,4))

    Q[0] = [q[3], -1 * q[0], -1 * q[1], -1 * q[2]]
    Q[1] = [q[0], q[3], q[2], -1 * q[1]]
    Q[2] = [q[1], -1 * q[2], q[3], q[0]]
    Q[3] = [q[2], q[1], -1 * q[0], q[3]]

    return Q
