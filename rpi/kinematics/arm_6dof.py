from typing import Tuple, Literal

import numpy as np
import sympy as sp
from pydantic import BaseModel

from kinematics.base_kinematics import BaseKinematics


class Arm6DoF(BaseKinematics):
    forward_kinematics_joint_names = ("theta1", "theta2", "theta3", "theta4", "theta5", "theta6")
    inverse_kinematics_coordinate_names = ("x", "y", "z", "yaw", "pitch", "roll")

    def __init__(self, dh_params):
        self.dh_params = np.matrix([
            [np.radians(dh_params[0][0]), np.radians(dh_params[0][1]), dh_params[0][2], dh_params[0][3]],
            [np.radians(dh_params[1][0]), np.radians(dh_params[1][1]), dh_params[1][2], dh_params[1][3]],
            [np.radians(dh_params[2][0]), np.radians(dh_params[2][1]), dh_params[2][2], dh_params[2][3]],
            [np.radians(dh_params[3][0]), np.radians(dh_params[3][1]), dh_params[3][2], dh_params[3][3]],
            [np.radians(dh_params[4][0]), np.radians(dh_params[4][1]), dh_params[4][2], dh_params[4][3]],
            [np.radians(dh_params[5][0]), np.radians(dh_params[5][1]), dh_params[5][2], dh_params[5][3]]
        ])
        self.dh_params_sympy = sp.Matrix([
            [sp.rad(dh_params[0][0]), sp.rad(dh_params[0][1]), dh_params[0][2], dh_params[0][3]],
            [sp.rad(dh_params[1][0]), sp.rad(dh_params[1][1]), dh_params[1][2], dh_params[1][3]],
            [sp.rad(dh_params[2][0]), sp.rad(dh_params[2][1]), dh_params[2][2], dh_params[2][3]],
            [sp.rad(dh_params[3][0]), sp.rad(dh_params[3][1]), dh_params[3][2], dh_params[3][3]],
            [sp.rad(dh_params[4][0]), sp.rad(dh_params[4][1]), dh_params[4][2], dh_params[4][3]],
            [sp.rad(dh_params[5][0]), sp.rad(dh_params[5][1]), dh_params[5][2], dh_params[5][3]]
        ])

    @staticmethod
    def get_length(start_coordinates: Tuple[float | int], end_coordinates: Tuple[float | int]):
        translation_length = np.sqrt((end_coordinates[0] - start_coordinates[0])**2 + (end_coordinates[1] - start_coordinates[1])**2 + (end_coordinates[2] - start_coordinates[2])**2)
        # if there is a greater rotation move the speed becomes deg per second
        rotation_z_length = np.abs(end_coordinates[3] - start_coordinates[3])
        rotation_y_length = np.abs(end_coordinates[4] - start_coordinates[4])
        rotation_x_length = np.abs(end_coordinates[5] - start_coordinates[5])
        return max(translation_length, np.degrees(rotation_z_length), np.degrees(rotation_y_length), np.degrees(rotation_x_length))

    @staticmethod
    def dh_transformation_matrix(theta, alpha, d, a, sympy=False):
        if sympy:
            return sp.Matrix([
                [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                [0,              sp.sin(alpha),                sp.cos(alpha),               d],
                [0,              0,                            0,                           1]
            ])
        else:
            return np.matrix([
                [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0,              np.sin(alpha),                np.cos(alpha),               d],
                [0,              0,                            0,                           1]
            ])

    @staticmethod
    def transformation_matrix(x, y, z, yaw, pitch, roll):
        return np.matrix([
            [np.cos(yaw)*np.cos(pitch), np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(roll), np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll), x],
            [np.sin(yaw)*np.cos(pitch), np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll), y],
            [           -np.sin(pitch),             np.cos(pitch)*np.sin(roll)                           , np.cos(pitch)*np.cos(roll)                                       , z],
            [                        0,                                                                 0,                                                                 0, 1]
        ])

    def forward_kinematics(self, joints: Tuple[float | int], toolframe_matrix=sp.eye(4)):
        theta1, theta2, theta3, theta4, theta5, theta6 = joints
        j1_dh_matrix = Arm6DoF.dh_transformation_matrix(theta1 + self.dh_params_sympy[0, 0], self.dh_params_sympy[0, 1], self.dh_params_sympy[0, 2], self.dh_params_sympy[0, 3], sympy=True)
        j2_dh_matrix = Arm6DoF.dh_transformation_matrix(theta2 + self.dh_params_sympy[1, 0], self.dh_params_sympy[1, 1], self.dh_params_sympy[1, 2], self.dh_params_sympy[1, 3], sympy=True)
        j3_dh_matrix = Arm6DoF.dh_transformation_matrix(theta3 + self.dh_params_sympy[2, 0], self.dh_params_sympy[2, 1], self.dh_params_sympy[2, 2], self.dh_params_sympy[2, 3], sympy=True)
        j4_dh_matrix = Arm6DoF.dh_transformation_matrix(theta4 + self.dh_params_sympy[3, 0], self.dh_params_sympy[3, 1], self.dh_params_sympy[3, 2], self.dh_params_sympy[3, 3], sympy=True)
        j5_dh_matrix = Arm6DoF.dh_transformation_matrix(theta5 + self.dh_params_sympy[4, 0], self.dh_params_sympy[4, 1], self.dh_params_sympy[4, 2], self.dh_params_sympy[4, 3], sympy=True)
        j6_dh_matrix = Arm6DoF.dh_transformation_matrix(theta6 + self.dh_params_sympy[5, 0], self.dh_params_sympy[5, 1], self.dh_params_sympy[5, 2], self.dh_params_sympy[5, 3], sympy=True)
        final_matrix = j1_dh_matrix * j2_dh_matrix * j3_dh_matrix * j4_dh_matrix * j5_dh_matrix * j6_dh_matrix * toolframe_matrix
        coordinates = [
            float(final_matrix[0, 3]),
            float(final_matrix[1, 3]),
            float(final_matrix[2, 3])
        ]
        pitch = sp.atan2(-final_matrix[2, 0], sp.sqrt(final_matrix[2, 2] ** 2 + final_matrix[2, 1] ** 2)).evalf()
        orientation = [
            float(sp.atan2(final_matrix[1, 0] / sp.cos(pitch), final_matrix[0, 0] / sp.cos(pitch))),
            float(pitch),
            float(sp.atan2(final_matrix[2, 1] / sp.cos(pitch), final_matrix[2, 2] / sp.cos(pitch)))
        ]
        return tuple(coordinates + orientation)

    def inverse_kinematics(self, coordinates: Tuple[float | int], toolframe_matrix=np.identity(4)):
        x, y, z, yaw, pitch, roll = coordinates
        toolframe_matrix_inverse = np.linalg.inv(toolframe_matrix) # this can be done faster maybe
        j0_6_reverse_kin_matrix = Arm6DoF.transformation_matrix(x, y, z, yaw, pitch, roll) @ toolframe_matrix_inverse
        j0_6_negate_matrix = Arm6DoF.transformation_matrix(0, 0, -self.dh_params[5, 2], 0, 0, 0)
        spherical_wrist_matrix = j0_6_reverse_kin_matrix @ j0_6_negate_matrix
        thetaA = np.arctan2(spherical_wrist_matrix[1, 3], spherical_wrist_matrix[0, 3])
        # x and y transformed so that j1 angle is 0 for the following calculations
        pos_j1_zero = np.array([
            spherical_wrist_matrix[0, 3] * np.cos(thetaA) - spherical_wrist_matrix[1, 3] * np.sin(thetaA),
            spherical_wrist_matrix[1, 3] * np.cos(thetaA) + spherical_wrist_matrix[0, 3] * np.sin(thetaA),
            spherical_wrist_matrix[2, 3]])
        # l1 = x' - dh_a1
        l1 = pos_j1_zero[0] - self.dh_params[0, 3]
        # l4 = z' - dh_d1
        l4 = pos_j1_zero[2] - self.dh_params[0, 2]
        # l2 = pythagorean theorem on l1 and l4
        l2 = np.sqrt(l1 ** 2 + l4 ** 2)
        # l3 = pythagorean theorem on dh_a3 and dh_d4
        l3 = np.sqrt(self.dh_params[2, 3] ** 2 + self.dh_params[3, 2] ** 2)
        # θB = arctan2(l1, l4)
        thetaB = np.arctan2(l1, l4)
        # θC = arccos((dh_a2^2 + l2^2 - l3^2) / (2*dh_a2*l2))
        thetaC = np.arccos((self.dh_params[1, 3] ** 2 + l2 ** 2 - l3 ** 2) / (2 * self.dh_params[1, 3] * l2))
        # θD = arccos((dh_a2^2 + l3^2 - l2^2) / (2*dh_a2*l3))
        thetaD = np.arccos((self.dh_params[1, 3] ** 2 + l3 ** 2 - l2 ** 2) / (2 * self.dh_params[1, 3] * l3))
        # θD = arctan(dh_a3 / dh_d4)
        thetaE = np.arctan(self.dh_params[2, 3] / self.dh_params[3, 2])

        j1_angle = thetaA
        j2_angle = thetaB - thetaC
        #TODO check that pi can be gotten from the dh matrix instead of being hardcoded
        j3_angle = np.pi - thetaD + thetaE

        j1_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[0, 0] + j1_angle, self.dh_params[0, 1], self.dh_params[0, 2], self.dh_params[0, 3])
        j2_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[1, 0] + j2_angle, self.dh_params[1, 1], self.dh_params[1, 2], self.dh_params[1, 3])
        j3_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[2, 0] + j3_angle, self.dh_params[2, 1], self.dh_params[2, 2], self.dh_params[2, 3])
        j0_3_dh_matrix = j1_dh_matrix @ j2_dh_matrix @ j3_dh_matrix
        j0_3_orientation_matrix_transposed = np.matrix_transpose(j0_3_dh_matrix[0:3:1,0:3:1]) @ j0_6_reverse_kin_matrix[0:3:1,0:3:1]

        # j5 = atan2(sqrt(1 - orientation_33^2), orientation_33)
        j5_angle = np.atan2(np.sqrt(1 - j0_3_orientation_matrix_transposed[2, 2] ** 2), j0_3_orientation_matrix_transposed[2, 2])
        j4_angle = np.atan2(j0_3_orientation_matrix_transposed[1, 2], j0_3_orientation_matrix_transposed[0, 2])
        j6_angle = np.atan2(j0_3_orientation_matrix_transposed[2, 1], - j0_3_orientation_matrix_transposed[2, 0])
        if np.isclose(j5_angle, 0, atol=1e-5):
            j6_angle -= j4_angle
            j4_angle = 0
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
        return j1_angle, j2_angle, j3_angle, j4_angle * 0, j5_angle, j6_angle * 0

class Arm6DoFModel(BaseModel):
    type: Literal["6DOF arm"]
    dh_params: list[list[float]]

    def get_kinematics(self):
        return Arm6DoF(self.dh_params)

if __name__ == "__main__":
    dh_params = np.matrix([
        [  180,  90.0, 231.5,   0.0],
        [ 90.0,   0.0,   0.0, 221.0],
        [-90.0, -90.0,   0.0,   0.0],
        [  0.0,  90.0, 224.5,   0.0],
        [  0.0, -90.0,   0.0,   0.0],
        [  180.0,   0.0,  77.0,   0.0]
    ])

    toolframe_matrix = Arm6DoF.transformation_matrix(0, 0, 0, 0, 0, 0)

    arm = Arm6DoF(dh_params)

    x, y, z, yaw, pitch, roll = arm.forward_kinematics((sp.rad(0), sp.rad(0), sp.rad(0), sp.rad(0), sp.rad(0), sp.rad(0)), toolframe_matrix)
    print("x: " + str(x))
    print("y: " + str(y))
    print("z: " + str(z))
    print("yaw: " + str(np.degrees(yaw)))
    print("pitch: " + str(np.degrees(pitch)))
    print("roll: " + str(np.degrees(roll)))
    print()

    j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = arm.inverse_kinematics((x, y, z, yaw, pitch, roll), toolframe_matrix)
    #j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = inverse_kinematics((457.7706, 0, 387.7706, -90, -90, -90), toolframe_matrix)
    print("j1_angle: " + str(np.degrees(j1_angle)))
    print("j2_angle: " + str(np.degrees(j2_angle)))
    print("j3_angle: " + str(180 - np.degrees(j3_angle)))
    print("j4_angle: " + str(np.degrees(j4_angle)))
    print("j5_angle: " + str(np.degrees(j5_angle) - 90))
    print("j6_angle: " + str(180 - np.degrees(j6_angle)))