import sympy as sp

def dh_transformation_matrix(theta, alpha, d, a, radians=False):
    if not radians:
        theta = sp.rad(theta)
        alpha = sp.rad(alpha)
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def transformation_matrix(x, y, z, yaw, pitch, roll, radians=False):
    if not radians:
        yaw = sp.rad(yaw)
        pitch = sp.rad(pitch)
        roll = sp.rad(roll)
    return sp.Matrix([
        [sp.cos(yaw)*sp.cos(pitch), sp.cos(yaw)*sp.sin(pitch)*sp.sin(roll) - sp.sin(yaw)*sp.cos(roll), sp.cos(yaw)*sp.sin(pitch)*sp.cos(roll) + sp.sin(yaw)*sp.sin(roll), x],
        [sp.sin(yaw)*sp.cos(pitch), sp.sin(yaw)*sp.sin(pitch)*sp.sin(roll) + sp.cos(yaw)*sp.cos(roll), sp.sin(yaw)*sp.sin(pitch)*sp.cos(roll) - sp.cos(yaw)*sp.sin(roll), y],
        [           -sp.sin(pitch),             sp.cos(pitch)*sp.sin(roll)                           ,             sp.cos(pitch)*sp.cos(roll)                           , z],
        [                        0,                                                                 0,                                                                 0, 1]
    ])

def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, dh_params, toolframe_matrix=sp.eye(4)):
    j1_dh_matrix = dh_transformation_matrix(theta1 + dh_params[0, 0], dh_params[0, 1], dh_params[0, 2], dh_params[0, 3])
    j2_dh_matrix = dh_transformation_matrix(theta2 + dh_params[1, 0], dh_params[1, 1], dh_params[1, 2], dh_params[1, 3])
    j3_dh_matrix = dh_transformation_matrix(theta3 + dh_params[2, 0], dh_params[2, 1], dh_params[2, 2], dh_params[2, 3])
    j4_dh_matrix = dh_transformation_matrix(theta4 + dh_params[3, 0], dh_params[3, 1], dh_params[3, 2], dh_params[3, 3])
    j5_dh_matrix = dh_transformation_matrix(theta5 + dh_params[4, 0], dh_params[4, 1], dh_params[4, 2], dh_params[4, 3])
    j6_dh_matrix = dh_transformation_matrix(theta6 + dh_params[5, 0], dh_params[5, 1], dh_params[5, 2], dh_params[5, 3])
    final_matrix = j1_dh_matrix * j2_dh_matrix * j3_dh_matrix * j4_dh_matrix * j5_dh_matrix * j6_dh_matrix * toolframe_matrix
    coordinates = [
        final_matrix[0, 3],
        final_matrix[1, 3],
        final_matrix[2, 3]
    ]
    pitch = sp.atan2(sp.sqrt(final_matrix[0, 0] ** 2 + final_matrix[1, 0] ** 2), -final_matrix[2, 0])
    orientation = [
        sp.atan2(final_matrix[0, 0] / sp.cos(pitch), final_matrix[1, 0] / sp.cos(pitch)),
        pitch,
        sp.atan2(final_matrix[2, 2] / sp.cos(pitch), final_matrix[2, 1] / sp.cos(pitch))
    ]
    return coordinates, orientation

def inverse_kinematics(x, y, z, yaw, pitch, roll, dh_params, toolframe_matrix=sp.eye(4)):
    toolframe_matrix_inverse = toolframe_matrix**-1     # this can be done faster maybe
    j0_6_reverse_kin_matrix = transformation_matrix(x, y, z, yaw, pitch, roll) * toolframe_matrix_inverse
    j0_6_negate_matrix = transformation_matrix(0, 0, -dh_params[5, 2], 0, 0, 0)
    spherical_wrist_matrix = j0_6_reverse_kin_matrix * j0_6_negate_matrix
    thetaA = sp.atan2(spherical_wrist_matrix[1, 3], spherical_wrist_matrix[0, 3])
    # x and y transformed so that j1 angle is 0 for the following calculations
    pos_j1_zero = [
        spherical_wrist_matrix[0, 3] * sp.cos(thetaA) - spherical_wrist_matrix[1, 3] * sp.sin(thetaA),
        spherical_wrist_matrix[1, 3] * sp.cos(thetaA) + spherical_wrist_matrix[0, 3] * sp.sin(thetaA),
        spherical_wrist_matrix[2, 3]
    ]
    # l1 = x' - dh_a1
    l1 = pos_j1_zero[0] - dh_params[0, 3]
    # l4 = z' - dh_d1
    l4 = pos_j1_zero[2] - dh_params[0, 2]
    # l2 = pythagorean theorem on l1 and l4
    l2 = sp.sqrt(l1 ** 2 + l4 ** 2)
    # l3 = pythagorean theorem on dh_a3 and dh_d4
    l3 = sp.sqrt(dh_params[2, 3] ** 2 + dh_params[3, 2] ** 2)
    # θB = arctan2(l1, l4)
    thetaB = sp.atan2(l1, l4)
    # θC = arccos((dh_a2^2 + l2^2 - l3^2) / (2*dh_a2*l2))
    thetaC = sp.acos((dh_params[1, 3] ** 2 + l2 ** 2 - l3 ** 2) / (2 * dh_params[1, 3] * l2))
    # θD = arccos((dh_a2^2 + l3^2 - l2^2) / (2*dh_a2*l3))
    thetaD = sp.acos((dh_params[1, 3] ** 2 + l3 ** 2 - l2 ** 2) / (2 * dh_params[1, 3] * l3))
    # θD = arctan(dh_a3 / dh_d4)
    thetaE = sp.atan(dh_params[2, 3] / dh_params[3, 2])

    j1_angle = thetaA
    j2_angle = thetaB - thetaC
    j3_angle = thetaD + thetaE

    j1_dh_matrix = dh_transformation_matrix(sp.rad(dh_params[0, 0]) + j1_angle, sp.rad(dh_params[0, 1]), dh_params[0, 2], dh_params[0, 3], radians=True)
    j2_dh_matrix = dh_transformation_matrix(sp.rad(dh_params[1, 0]) + j2_angle, sp.rad(dh_params[1, 1]), dh_params[1, 2], dh_params[1, 3], radians=True)
    j3_dh_matrix = dh_transformation_matrix(sp.rad(dh_params[2, 0]) + j3_angle, sp.rad(dh_params[2, 1]), dh_params[2, 2], dh_params[2, 3], radians=True)
    j0_3_dh_matrix = j1_dh_matrix * j2_dh_matrix * j3_dh_matrix
    j0_3_orientation_matrix_transposed = j0_3_dh_matrix[0:3:1,0:3:1].T * j0_6_reverse_kin_matrix[0:3:1,0:3:1]

    j4_angle = sp.atan2(j0_3_orientation_matrix_transposed[1, 2], j0_3_orientation_matrix_transposed[0, 2])
    # j5 = atan2(sqrt(1 - orientation_33^2), orientation_33)
    j5_angle = sp.atan2(sp.sqrt(1 - j0_3_orientation_matrix_transposed[2, 2] ** 2), j0_3_orientation_matrix_transposed[2, 2])
    j6_angle = sp.atan2(j0_3_orientation_matrix_transposed[2, 1], - j0_3_orientation_matrix_transposed[2, 0])
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

dh_params = sp.Matrix([
    [ 180.0,  90.0, 231.5,   0.0],
    [ 90.0,   0.0,   0.0, 221.0],
    [-90.0, -90.0,   0.0,   0.0],
    [  0.0,  90.0, 224.5,   0.0],
    [  0.0, -90.0,   0.0,   0.0],
    [  0.0,   0.0,  77.0,   0.0]
])

toolframe_matrix = transformation_matrix(0, 0, 0, 0, 0, 0)

coordinates, orientation = forward_kinematics(0, 45, 45, 0, 0, 0, dh_params, toolframe_matrix)
print("x: " + str(coordinates[0].evalf(10)))
print("y: " + str(coordinates[1].evalf(10)))
print("z: " + str(coordinates[2].evalf(10)))
print("yaw: " + str(sp.deg(orientation[0]).evalf(10)))
print("pitch: " + str(sp.deg(orientation[1]).evalf(10)))
print("roll: " + str(sp.deg(orientation[2]).evalf(10)))
# print()

# j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(coordinates[0], coordinates[1], coordinates[2], sp.deg(orientation[0]), sp.deg(orientation[1]), sp.deg(orientation[2]), dh_params, toolframe_matrix)
j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics(457.7706, 0, 387.7706, -90, -90, -90, dh_params, toolframe_matrix)
print("j1_angle: " + str(sp.deg(j1_angle).evalf(6)))
print("j2_angle: " + str(sp.deg(j2_angle).evalf(6)))
print("j3_angle: " + str(sp.deg(j3_angle).evalf(6)))
print("j4_angle: " + str(sp.deg(j4_angle).evalf(6)))
print("j5_angle: " + str(sp.deg(j5_angle).evalf(6)))
print("j6_angle: " + str(sp.deg(j6_angle).evalf(6)))