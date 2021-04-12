import numpy as np
from math import pi


def calc_kinematics(x_in, y_in, z_in, alpha_in, beta_in, gamma_in):
    # defining input variables
    x = x_in
    y = y_in
    z = 0.16107 + z_in

    alpha = alpha_in
    beta = beta_in
    gamma = gamma_in


    # calculating sin and cos values for matrices
    a_sin = np.sin(alpha)
    a_cos = np.cos(alpha)

    b_sin = np.sin(beta)
    b_cos = np.cos(beta)

    g_sin = np.sin(gamma)
    g_cos = np.cos(gamma)


    # defining rotation matrices
    r_a = np.array([
        [1., 0., 0.],
        [0., a_cos, -a_sin],
        [0., a_sin, a_cos]
    ])

    r_b = np.array([
        [b_cos, 0., b_sin],
        [0., 1., 0.],
        [-b_sin, 0., b_cos]
    ])

    r_g = np.array([
        [g_cos, -g_sin, 0.],
        [g_sin, g_cos, 0.],
        [0., 0., 1.]
    ])


    # defining total rotation matrix
    r = r_g @ r_b @ r_a


    # defining position vector
    p = np.array([
        [x],
        [y],
        [z]
    ])


    # defining base plate position vectors
    a1 = np.array([
        [-0.14228], [-0.0475], [0.]
    ])

    a2 = np.array([
        [-0.11228], [-0.09947], [0.]
    ])

    a3 = np.array([
        [0.11228], [-0.09947], [0.]
    ])

    a4 = np.array([
        [0.14228], [-0.0475], [0.]
    ])

    a5 = np.array([
        [0.030], [0.14697], [0.]
    ])

    a6 = np.array([
        [-0.030], [0.14697], [0.]
    ])


    # defining tool plate position vectors
    b1 = np.array([
        [-0.09761], [0.02172], [0.]
    ])

    b2 = np.array([
        [-0.030], [-0.09539], [0.]
    ])

    b3 = np.array([
        [0.030], [-0.09539], [0.]
    ])

    b4 = np.array([
        [0.09761], [0.02172], [0.]
    ])

    b5 = np.array([
        [0.06761], [0.07368], [0.]
    ])

    b6 = np.array([
        [-0.06761], [0.07368], [0.]
    ])


    # calculating leg vectors
    s1 = p + (r @ b1) - a1
    s2 = p + (r @ b2) - a2
    s3 = p + (r @ b3) - a3
    s4 = p + (r @ b4) - a4
    s5 = p + (r @ b5) - a5
    s6 = p + (r @ b6) - a6


    # calculating leg lengths
    l1 = np.sqrt(np.float_power(s1[0, 0], 2) + np.float_power(s1[1, 0], 2) + np.float_power(s1[2, 0], 2))
    l2 = np.sqrt(np.float_power(s2[0, 0], 2) + np.float_power(s2[1, 0], 2) + np.float_power(s2[2, 0], 2))
    l3 = np.sqrt(np.float_power(s3[0, 0], 2) + np.float_power(s3[1, 0], 2) + np.float_power(s3[2, 0], 2))
    l4 = np.sqrt(np.float_power(s4[0, 0], 2) + np.float_power(s4[1, 0], 2) + np.float_power(s4[2, 0], 2))
    l5 = np.sqrt(np.float_power(s5[0, 0], 2) + np.float_power(s5[1, 0], 2) + np.float_power(s5[2, 0], 2))
    l6 = np.sqrt(np.float_power(s6[0, 0], 2) + np.float_power(s6[1, 0], 2) + np.float_power(s6[2, 0], 2))

    # actuator stroke position
    d1 = l1 - 0.181
    d2 = l2 - 0.181
    d3 = l3 - 0.181
    d4 = l4 - 0.181
    d5 = l5 - 0.181
    d6 = l6 - 0.181

    return np.array([d1, d2, d3, d4, d5, d6])