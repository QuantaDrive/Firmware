import numpy as np

def dh_transformation_matrix(theta, alpha, d, a, radians=False):
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

def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, dh_params, toolframe_matrix=np.identity(4)):
    j1_dh_matrix = dh_transformation_matrix(theta1 + dh_params[0, 0], dh_params[0, 1], dh_params[0, 2], dh_params[0, 3])
    j2_dh_matrix = dh_transformation_matrix(theta2 + dh_params[1, 0], dh_params[1, 1], dh_params[1, 2], dh_params[1, 3])
    j3_dh_matrix = dh_transformation_matrix(theta3 + dh_params[2, 0], dh_params[2, 1], dh_params[2, 2], dh_params[2, 3])
    j4_dh_matrix = dh_transformation_matrix(theta4 + dh_params[3, 0], dh_params[3, 1], dh_params[3, 2], dh_params[3, 3])
    j5_dh_matrix = dh_transformation_matrix(theta5 + dh_params[4, 0], dh_params[4, 1], dh_params[4, 2], dh_params[4, 3])
    j6_dh_matrix = dh_transformation_matrix(theta6 + dh_params[5, 0], dh_params[5, 1], dh_params[5, 2], dh_params[5, 3])
    final_matrix = j1_dh_matrix @ j2_dh_matrix @ j3_dh_matrix @ j4_dh_matrix @ j5_dh_matrix @ j6_dh_matrix @ toolframe_matrix
    coordinates = np.squeeze(np.asarray(final_matrix[0:3, 3]))
    pitch = np.atan2(-final_matrix[2, 0], np.sqrt(final_matrix[0, 0] ** 2 + final_matrix[1, 0] ** 2))
    orientation = np.array([
        np.arctan2(final_matrix[1, 0] / np.cos(pitch), final_matrix[0, 0] / np.cos(pitch)),
        pitch,
        np.arctan2(final_matrix[2, 1] / np.cos(pitch), final_matrix[2, 2] / np.cos(pitch))
    ])
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
    [ 180.0,  90.0, 231.5,   0.0],
    [ 90.0,   0.0,   0.0, 221.0],
    [-90.0, -90.0,   0.0,   0.0],
    [  0.0,  90.0, 224.5,   0.0],
    [  0.0, -90.0,   0.0,   0.0],
    [  0.0,   0.0,  77.0,   0.0]
])

toolframe_matrix = transformation_matrix(0, 0, 0, 0, 0, 0)

coordinates, orientation = forward_kinematics(0, 45, 45, 0, 0, 0, dh_params, toolframe_matrix)
print("x: " + str(coordinates[0]))
print("y: " + str(coordinates[1]))
print("z: " + str(coordinates[2]))
print("yaw: " + str(np.degrees(orientation[0])))
print("pitch: " + str(np.degrees(orientation[1])))
print("roll: " + str(np.degrees(orientation[2])))
print()

#j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(coordinates[0], coordinates[1], coordinates[2], np.degrees(orientation[0]), np.degrees(orientation[1]), np.degrees(orientation[2]), dh_params, toolframe_matrix)
j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(457.7706, 0, 387.7706, -90, -90, -90, dh_params, toolframe_matrix)
print("j1_angle: " + str(np.around(np.degrees(j1_angle), decimals=4)))
print("j2_angle: " + str(np.around(np.degrees(j2_angle), decimals=4)))
print("j3_angle: " + str(np.around(np.degrees(j3_angle), decimals=4)))
print("j4_angle: " + str(np.around(np.degrees(j4_angle), decimals=4)))
print("j5_angle: " + str(np.around(np.degrees(j5_angle), decimals=4)))
print("j6_angle: " + str(np.around(np.degrees(j6_angle), decimals=4)))