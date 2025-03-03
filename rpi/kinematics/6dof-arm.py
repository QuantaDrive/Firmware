import numpy as np
import sympy as sp

def dh_transformation_matrix(theta, alpha, d, a, radians=False, sympy=False):
    if sympy:
        if not radians:
            theta = sp.rad(theta)
            alpha = sp.rad(alpha)
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0,              sp.sin(alpha),                sp.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
    else:
        if not radians:
            theta = np.radians(theta)
            alpha = np.radians(alpha)
        return np.matrix([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

def transformation_matrix(x, y, z, yaw, pitch, roll, radians=False):
    if not radians:
        yaw = np.radians(yaw)
        pitch = np.radians(pitch)
        roll = np.radians(roll)
    return np.matrix([
        [np.cos(yaw)*np.cos(pitch), np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(roll), np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll), x],
        [np.sin(yaw)*np.cos(pitch), np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll), y],
        [           -np.sin(pitch),             np.cos(pitch)*np.sin(roll)                           , np.cos(pitch)*np.cos(roll)                                       , z],
        [                        0,                                                                 0,                                                                 0, 1]
    ])

def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, dh_params, toolframe_matrix=sp.eye(4)):
    j1_dh_matrix = dh_transformation_matrix(theta1 + dh_params[0, 0], dh_params[0, 1], dh_params[0, 2], dh_params[0, 3], sympy=True)
    j2_dh_matrix = dh_transformation_matrix(theta2 + dh_params[1, 0], dh_params[1, 1], dh_params[1, 2], dh_params[1, 3], sympy=True)
    j3_dh_matrix = dh_transformation_matrix(theta3 + dh_params[2, 0], dh_params[2, 1], dh_params[2, 2], dh_params[2, 3], sympy=True)
    j4_dh_matrix = dh_transformation_matrix(theta4 + dh_params[3, 0], dh_params[3, 1], dh_params[3, 2], dh_params[3, 3], sympy=True)
    j5_dh_matrix = dh_transformation_matrix(theta5 + dh_params[4, 0], dh_params[4, 1], dh_params[4, 2], dh_params[4, 3], sympy=True)
    j6_dh_matrix = dh_transformation_matrix(theta6 + dh_params[5, 0], dh_params[5, 1], dh_params[5, 2], dh_params[5, 3], sympy=True)
    final_matrix = j1_dh_matrix * j2_dh_matrix * j3_dh_matrix * j4_dh_matrix * j5_dh_matrix * j6_dh_matrix * toolframe_matrix
    coordinates = [
        final_matrix[0, 3],
        final_matrix[1, 3],
        final_matrix[2, 3]
    ]
    pitch = sp.atan2(-final_matrix[2, 0], sp.sqrt(final_matrix[2, 2] ** 2 + final_matrix[2, 1] ** 2)).evalf()
    orientation = [
        sp.atan2(final_matrix[1, 0] / sp.cos(pitch), final_matrix[0, 0] / sp.cos(pitch)),
        pitch,
        sp.atan2(final_matrix[2, 1] / sp.cos(pitch), final_matrix[2, 2] / sp.cos(pitch))
    ]
    return coordinates, orientation

def inverse_kinematics(x, y, z, yaw, pitch, roll, dh_params, toolframe_matrix=np.identity(4)):
    toolframe_matrix_inverse = np.linalg.inv(toolframe_matrix) # this can be done faster maybe
    j0_6_reverse_kin_matrix = transformation_matrix(x, y, z, yaw, pitch, roll) @ toolframe_matrix_inverse
    j0_6_negate_matrix = transformation_matrix(0, 0, -dh_params[5, 2], 0, 0, 0)
    spherical_wrist_matrix = j0_6_reverse_kin_matrix @ j0_6_negate_matrix
    thetaA = np.arctan2(spherical_wrist_matrix[1, 3], spherical_wrist_matrix[0, 3])
    # x and y transformed so that j1 angle is 0 for the following calculations
    pos_j1_zero = np.array([
        spherical_wrist_matrix[0, 3] * np.cos(thetaA) - spherical_wrist_matrix[1, 3] * np.sin(thetaA),
        spherical_wrist_matrix[1, 3] * np.cos(thetaA) + spherical_wrist_matrix[0, 3] * np.sin(thetaA),
        spherical_wrist_matrix[2, 3]])
    # l1 = x' - dh_a1
    l1 = pos_j1_zero[0] - dh_params[0, 3]
    # l4 = z' - dh_d1
    l4 = pos_j1_zero[2] - dh_params[0, 2]
    # l2 = pythagorean theorem on l1 and l4
    l2 = np.sqrt(l1 ** 2 + l4 ** 2)
    # l3 = pythagorean theorem on dh_a3 and dh_d4
    l3 = np.sqrt(dh_params[2, 3] ** 2 + dh_params[3, 2] ** 2)
    # θB = arctan2(l1, l4)
    thetaB = np.arctan2(l1, l4)
    # θC = arccos((dh_a2^2 + l2^2 - l3^2) / (2*dh_a2*l2))
    thetaC = np.arccos((dh_params[1, 3] ** 2 + l2 ** 2 - l3 ** 2) / (2 * dh_params[1, 3] * l2))
    # θD = arccos((dh_a2^2 + l3^2 - l2^2) / (2*dh_a2*l3))
    thetaD = np.arccos((dh_params[1, 3] ** 2 + l3 ** 2 - l2 ** 2) / (2 * dh_params[1, 3] * l3))
    # θD = arctan(dh_a3 / dh_d4)
    thetaE = np.arctan(dh_params[2, 3] / dh_params[3, 2])

    j1_angle = thetaA
    j2_angle = thetaB - thetaC
    j3_angle = thetaD + thetaE

    j1_dh_matrix = dh_transformation_matrix(np.radians(dh_params[0, 0]) + j1_angle, np.radians(dh_params[0, 1]), dh_params[0, 2], dh_params[0, 3], radians=True)
    j2_dh_matrix = dh_transformation_matrix(np.radians(dh_params[1, 0]) + j2_angle, np.radians(dh_params[1, 1]), dh_params[1, 2], dh_params[1, 3], radians=True)
    j3_dh_matrix = dh_transformation_matrix(np.radians(dh_params[2, 0]) + j3_angle, np.radians(dh_params[2, 1]), dh_params[2, 2], dh_params[2, 3], radians=True)
    j0_3_dh_matrix = j1_dh_matrix @ j2_dh_matrix @ j3_dh_matrix
    j0_3_orientation_matrix_transposed = np.matrix_transpose(j0_3_dh_matrix[0:3:1,0:3:1]) @ j0_6_reverse_kin_matrix[0:3:1,0:3:1]

    j4_angle = np.atan2(j0_3_orientation_matrix_transposed[1, 2], j0_3_orientation_matrix_transposed[0, 2])
    # j5 = atan2(sqrt(1 - orientation_33^2), orientation_33)
    j5_angle = np.atan2(np.sqrt(1 - j0_3_orientation_matrix_transposed[2, 2] ** 2), j0_3_orientation_matrix_transposed[2, 2])
    j6_angle = np.atan2(j0_3_orientation_matrix_transposed[2, 1], - j0_3_orientation_matrix_transposed[2, 0])
    # if j5_angle > 0:
    #     # j4 = atan2(orientation_23, orientation_13)
    #     j4_angle = np.atan2(j0_3_orientation_matrix_transposed[1, 2], j0_3_orientation_matrix_transposed[0, 2])
    #     # j6 = atan2(orientation_32, - orientation_31)
    #     j6_angle = np.atan2(j0_3_orientation_matrix_transposed[2, 1], - j0_3_orientation_matrix_transposed[2, 0])
    # else:
    #     # j4 = atan2( - orientation_23,  - orientation_13)
    #     j4_angle = np.atan2( - j0_3_orientation_matrix_transposed[1, 2], - j0_3_orientation_matrix_transposed[0, 2])
    #     # j6 = atan2( - orientation_32, orientation_31)
    #     j6_angle = np.atan2( - j0_3_orientation_matrix_transposed[2, 1], j0_3_orientation_matrix_transposed[2, 0])
    return j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle

dh_params = np.matrix([
    [  0.0,  90.0, 231.5,   0.0],
    [ 90.0,   0.0,   0.0, 221.0],
    [-90.0, -90.0,   0.0,   0.0],
    [  0.0,  90.0, 224.5,   0.0],
    [  0.0, -90.0,   0.0,   0.0],
    [  0.0,   0.0,  77.0,   0.0]
])

toolframe_matrix = transformation_matrix(0, 0, 0, 0, 0, 0)

coordinates, orientation = forward_kinematics(0, 30, 0, 0, 0, 0, dh_params, toolframe_matrix)
print("x: " + str(coordinates[0].evalf()))
print("y: " + str(coordinates[1].evalf()))
print("z: " + str(coordinates[2].evalf()))
print("yaw: " + str(sp.deg(orientation[0]).evalf()))
print("pitch: " + str(sp.deg(orientation[1]).evalf()))
print("roll: " + str(sp.deg(orientation[2]).evalf()))
print()

# j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(coordinates[0], coordinates[1], coordinates[2], sp.deg(orientation[0]), sp.deg(orientation[1]), sp.deg(orientation[2]), dh_params, toolframe_matrix)
j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(457.7706, 0, 387.7706, -90, -90, -90, dh_params, toolframe_matrix)
print("j1_angle: " + str(np.degrees(j1_angle)))
print("j2_angle: " + str(np.degrees(j2_angle)))
print("j3_angle: " + str(np.degrees(j3_angle)))
print("j4_angle: " + str(np.degrees(j4_angle)))
print("j5_angle: " + str(np.degrees(j5_angle)))
print("j6_angle: " + str(np.degrees(j6_angle)))