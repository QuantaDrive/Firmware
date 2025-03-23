from __future__ import annotations

from typing import Literal

import numpy as np
import numpy.typing as npt
import sympy as sp

from kinematics import Move
from kinematics.base_kinematics import BaseKinematics, BaseKinematicsModel


class Arm6DoF(BaseKinematics):
    forward_kinematics_joint_names = ("theta1", "theta2", "theta3", "theta4", "theta5", "theta6")
    inverse_kinematics_coordinate_names = ("x", "y", "z", "yaw", "pitch", "roll")
    parse_coordinate_names = ("x", "y", "z", "i", "j", "k")

    def __init__(self, settings: type[Arm6DoFModel]):
        super().__init__(settings)
        self._cur_direction = np.array([0, 0, 0, 0, 0, 0])
        self._cur_speed = np.array([0, 0, 0, 0])

        self.dh_params = np.array([
            [np.radians(self.settings.dh_params[0][0]), np.radians(self.settings.dh_params[0][1]), self.settings.dh_params[0][2], self.settings.dh_params[0][3]],
            [np.radians(self.settings.dh_params[1][0]), np.radians(self.settings.dh_params[1][1]), self.settings.dh_params[1][2], self.settings.dh_params[1][3]],
            [np.radians(self.settings.dh_params[2][0]), np.radians(self.settings.dh_params[2][1]), self.settings.dh_params[2][2], self.settings.dh_params[2][3]],
            [np.radians(self.settings.dh_params[3][0]), np.radians(self.settings.dh_params[3][1]), self.settings.dh_params[3][2], self.settings.dh_params[3][3]],
            [np.radians(self.settings.dh_params[4][0]), np.radians(self.settings.dh_params[4][1]), self.settings.dh_params[4][2], self.settings.dh_params[4][3]],
            [np.radians(self.settings.dh_params[5][0]), np.radians(self.settings.dh_params[5][1]), self.settings.dh_params[5][2], self.settings.dh_params[5][3]]
        ])
        self.dh_params_sympy = sp.Matrix([
            [sp.rad(self.settings.dh_params[0][0]), sp.rad(self.settings.dh_params[0][1]), self.settings.dh_params[0][2], self.settings.dh_params[0][3]],
            [sp.rad(self.settings.dh_params[1][0]), sp.rad(self.settings.dh_params[1][1]), self.settings.dh_params[1][2], self.settings.dh_params[1][3]],
            [sp.rad(self.settings.dh_params[2][0]), sp.rad(self.settings.dh_params[2][1]), self.settings.dh_params[2][2], self.settings.dh_params[2][3]],
            [sp.rad(self.settings.dh_params[3][0]), sp.rad(self.settings.dh_params[3][1]), self.settings.dh_params[3][2], self.settings.dh_params[3][3]],
            [sp.rad(self.settings.dh_params[4][0]), sp.rad(self.settings.dh_params[4][1]), self.settings.dh_params[4][2], self.settings.dh_params[4][3]],
            [sp.rad(self.settings.dh_params[5][0]), sp.rad(self.settings.dh_params[5][1]), self.settings.dh_params[5][2], self.settings.dh_params[5][3]]
        ])

    @property
    def cur_direction(self) -> npt.NDArray[float | int]:
        return self._cur_direction

    @cur_direction.setter
    def cur_direction(self, direction: list[float | int]):
        self._cur_direction = self.normalize_direction(direction)

    def convert_coordinates(self, coordinates: list[float | int | None] | npt.NDArray[float | int | None]) -> npt.NDArray[float | int]:
        return np.array([
            coordinates[0] if coordinates[0] is not None else self.coordinates[0],
            coordinates[1] if coordinates[1] is not None else self.coordinates[1],
            coordinates[2] if coordinates[2] is not None else self.coordinates[2],
            np.radians(coordinates[3]) if coordinates[3] is not None else self.coordinates[3],
            np.radians(coordinates[4]) if coordinates[4] is not None else self.coordinates[4],
            np.radians(coordinates[5]) if coordinates[5] is not None else self.coordinates[5]
        ])

    @staticmethod
    def normalize_direction(direction: list[float | int] | npt.NDArray[float | int]) -> npt.NDArray[float | int]:
        direction = np.array(direction)
        direction_xyz = direction[:3]
        direction_xyz_length = np.sqrt(direction_xyz[0] ** 2 + direction_xyz[1] ** 2 + direction_xyz[2] ** 2)
        direction_xyz_normalized = direction_xyz / direction_xyz_length

        direction_ypr = np.sign(direction[3:])

        return np.concatenate((direction_xyz_normalized, direction_ypr))

    @staticmethod
    def angle_between_directions(direction1: npt.NDArray[float | int], direction2: npt.NDArray[float | int]) -> npt.NDArray[float | int]:
        cos_alpha_xyz = np.dot(Arm6DoF.normalize_direction(direction1)[:3], Arm6DoF.normalize_direction(direction2)[:3])
        cos_alpha_z = np.dot(Arm6DoF.normalize_direction(direction1)[3], Arm6DoF.normalize_direction(direction2)[3])
        cos_alpha_y = np.dot(Arm6DoF.normalize_direction(direction1)[4], Arm6DoF.normalize_direction(direction2)[4])
        cos_alpha_x = np.dot(Arm6DoF.normalize_direction(direction1)[5], Arm6DoF.normalize_direction(direction2)[5])
        cos_alpha = np.array([cos_alpha_xyz, cos_alpha_z, cos_alpha_y, cos_alpha_x])
        return np.clip(cos_alpha, 0, 1)

    @staticmethod
    def get_length(start_coordinates: npt.NDArray[float | int], end_coordinates: npt.NDArray[float | int]) -> tuple[npt.NDArray[float | int], int]:
        translation_length = np.sqrt((end_coordinates[0] - start_coordinates[0])**2 + (end_coordinates[1] - start_coordinates[1])**2 + (end_coordinates[2] - start_coordinates[2])**2)
        # if rotation move is greater than the translation the speed becomes deg per second
        rotation_z = np.abs(end_coordinates[3] - start_coordinates[3])
        rotation_y = np.abs(end_coordinates[4] - start_coordinates[4])
        rotation_x = np.abs(end_coordinates[5] - start_coordinates[5])
        lengths = np.array([translation_length, np.degrees(rotation_z), np.degrees(rotation_y), np.degrees(rotation_x)])
        length_index = np.argmax(lengths)
        return lengths, length_index

    def get_speed_dir_size(self, speed: list[float | int] = None) -> npt.NDArray[float | int]:
        if speed is None:
            speed = self._cur_speed
        return np.array([speed[0], speed[0], speed[0], speed[1], speed[2], speed[3]])

    def calc_new_jog_velocity(self, new_direction: list[float | int], time_to_move: float | int):
        cur_speed_per_axis = self.cur_direction * self.get_speed_dir_size()
        cur_max_accel = self.cur_direction * self.settings.max_accel
        cur_max_decel = self.cur_direction * self.settings.jog_decel

        new_direction_normalized = self.normalize_direction(new_direction)
        # if you just multiply the new direction with the max speed the max speed will be sqrt(3) times higher than it should be
        max_velocity_new_direction = new_direction_normalized * np.abs(new_direction) * self.settings.jog_velocity

        # the new speed per axis (absolute value)
        new_speed_per_axis = np.array([0, 0, 0, 0, 0, 0])
        # the new direction per axis
        move_direction = np.array([0, 0, 0, 0, 0, 0])

        for i in range(len(self._cur_direction)):
            # if the axis is not moving and does not need to move
            if np.sign(new_direction_normalized[i]) == 0 and np.sign(self.cur_direction[i]) == 0:
                new_speed_per_axis[i] = 0
            # if the axis must stop moving
            elif np.sign(new_direction_normalized[i]) == 0:
                new_speed_per_axis[i] = np.max([
                    cur_speed_per_axis[i] - cur_max_decel[i] * time_to_move,
                    0
                ])
                if new_speed_per_axis[i] != 0:
                    move_direction[i] = np.sign(self.cur_direction[i])
            # if the axis must move to the opposite direction (decelerate to 0 and accelerate)
            elif np.sign(new_direction_normalized[i]) != np.sign(self.cur_direction[i]) and cur_speed_per_axis[i] != 0:
                time_to_stall = np.abs(cur_speed_per_axis[i]) / cur_max_accel[i]
                # the axis is not going to get to 0 this time
                if time_to_stall >= time_to_move:
                    new_speed_per_axis[i] = np.max([
                        cur_speed_per_axis[i] - cur_max_accel[i] * time_to_move,
                        0
                    ])
                    move_direction[i] = np.sign(self.cur_direction[i])
                # the axis can get to 0 in time to accelerate in the opposite direction
                else:
                    time_to_accel = time_to_move - time_to_stall
                    new_speed_per_axis[i] = np.min([
                        cur_max_accel[i] * time_to_accel,
                        max_velocity_new_direction[i]
                    ])
                    move_direction[i] = np.sign(new_direction_normalized[i])
            # if the axis starts moving or continues moving in the same direction
            else:
                if max_velocity_new_direction[i] < cur_speed_per_axis[i]:
                    new_speed_per_axis[i] = np.max([
                        cur_speed_per_axis[i] - cur_max_accel[i] * time_to_move,
                        max_velocity_new_direction[i]
                    ])
                else:
                    new_speed_per_axis[i] = np.min([
                        cur_speed_per_axis[i] + cur_max_accel[i] * time_to_move,
                        max_velocity_new_direction[i]
                    ])
                move_direction[i] = np.sign(new_direction_normalized[i])

        new_speed_xyz = np.sqrt(new_speed_per_axis[0] ** 2 + new_speed_per_axis[1] ** 2 + new_speed_per_axis[2] ** 2)
        self._cur_speed = np.concatenate((new_speed_xyz, new_speed_per_axis[3:]))
        self.cur_direction = move_direction * new_speed_per_axis

        return move_direction * new_speed_per_axis

    @staticmethod
    def dh_transformation_matrix(theta, alpha, d, a, sympy=False):
        if sympy:
            return sp.Matrix([
                [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                [0,              sp.sin(alpha),                sp.cos(alpha),               d],
                [0,              0,                            0,                           1]
            ])
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])

    @staticmethod
    def transformation_matrix(x, y, z, yaw, pitch, roll):
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y],
            [  -sp,    cp*sr        ,    cp*cr        , z],
            [    0,        0        ,        0        , 1]
        ])

    def forward_kinematics(self, joints: tuple[float | int, ...], toolframe_matrix=sp.eye(4)) -> tuple[float | int, ...]:
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

    def inverse_kinematics(self, coordinates: tuple[float | int, ...], toolframe_matrix=np.identity(4)) -> tuple[float | int, ...]:
        x, y, z, yaw, pitch, roll = coordinates
        toolframe_matrix_inverse = np.linalg.inv(toolframe_matrix) # this can be done faster maybe
        j0_6_reverse_kin_matrix = Arm6DoF.transformation_matrix(x, y, z, yaw, pitch, roll) @ toolframe_matrix_inverse
        j0_6_negate_matrix = Arm6DoF.transformation_matrix(0, 0, -self.dh_params[5, 2], 0, 0, 0)
        spherical_wrist_matrix = j0_6_reverse_kin_matrix @ j0_6_negate_matrix
        thetaA = np.arctan2(spherical_wrist_matrix[1, 3], spherical_wrist_matrix[0, 3])
        # x and y transformed so that j1 angle is 0 for the following calculations
        cThetaA, sThetaA = np.cos(thetaA), np.sin(thetaA)
        pos_j1_zero = np.array([
            spherical_wrist_matrix[0, 3] * cThetaA - spherical_wrist_matrix[1, 3] * sThetaA,
            spherical_wrist_matrix[1, 3] * cThetaA + spherical_wrist_matrix[0, 3] * sThetaA,
            spherical_wrist_matrix[2, 3]])
        # l1 = x' - dh_a1
        l1 = np.sqrt(spherical_wrist_matrix[0, 3] ** 2 + spherical_wrist_matrix[1, 3] ** 2) - self.dh_params[0, 3]
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
        j3_angle = np.pi - thetaD - thetaE

        j1_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[0, 0] + j1_angle, self.dh_params[0, 1], self.dh_params[0, 2], self.dh_params[0, 3])
        j2_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[1, 0] + j2_angle, self.dh_params[1, 1], self.dh_params[1, 2], self.dh_params[1, 3])
        j3_dh_matrix = Arm6DoF.dh_transformation_matrix(self.dh_params[2, 0] + j3_angle, self.dh_params[2, 1], self.dh_params[2, 2], self.dh_params[2, 3])
        j0_3_dh_matrix = j1_dh_matrix @ j2_dh_matrix @ j3_dh_matrix
        j0_3_orientation_matrix_transposed = j0_3_dh_matrix[0:3:1,0:3:1].T @ j0_6_reverse_kin_matrix[0:3:1,0:3:1]

        # j5 = atan2(sqrt(1 - orientation_33^2), orientation_33)
        j5_angle_sign = np.atan2(-j0_3_orientation_matrix_transposed[2, 0], np.sqrt(
            j0_3_orientation_matrix_transposed[0, 0] ** 2 + j0_3_orientation_matrix_transposed[1, 0] ** 2))
        j5_angle = np.atan2(np.sqrt(1-j0_3_orientation_matrix_transposed[2, 2]**2), j0_3_orientation_matrix_transposed[2, 2]) * np.sign(j5_angle_sign)
        if j5_angle > 0:
            # j4 = atan2(orientation_23, orientation_13)
            j4_angle = - np.atan2(j0_3_orientation_matrix_transposed[1, 2], j0_3_orientation_matrix_transposed[0, 2])
            # j6 = atan2(orientation_32, - orientation_31)
            j6_angle = np.atan2(j0_3_orientation_matrix_transposed[2, 1], - j0_3_orientation_matrix_transposed[2, 0])
        else:
            # j4 = atan2( - orientation_23,  - orientation_13)
            j4_angle = - np.atan2( - j0_3_orientation_matrix_transposed[1, 2], - j0_3_orientation_matrix_transposed[0, 2])
            # j6 = atan2( - orientation_32, orientation_31)
            j6_angle = np.atan2( - j0_3_orientation_matrix_transposed[2, 1], j0_3_orientation_matrix_transposed[2, 0])
        if j4_angle > 0:
            j4_angle -= np.pi
        else:
            j4_angle += np.pi
        if np.isclose(j5_angle, 0, atol=1e-5):
            j6_angle -= j4_angle
            j4_angle = 0
        return j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle

class Arm6DoFModel(BaseKinematicsModel):
    type: Literal["6DOF arm"]
    dh_params: list[list[float]]

    def get_kinematics(self) -> Arm6DoF:
        return Arm6DoF(self)

if __name__ == "__main__":
    dh_params = np.array([
        [  180,  90.0, 231.5,   0.0],
        [ 90.0,   0.0,   0.0, 221.0],
        [-90.0, -90.0,   0.0,   0.0],
        [  0.0,  90.0, 224.5,   0.0],
        [  0.0, -90.0,   0.0,   0.0],
        [  180.0,   0.0,  77.5,   0.0]
    ])

    toolframe_matrix = Arm6DoF.transformation_matrix(0, 0, 0, 0, 0, 0)
    arm = Arm6DoF(dh_params)

    x, y, z, yaw, pitch, roll = arm.forward_kinematics((sp.rad(0), sp.rad(0), sp.rad(90), sp.rad(0), sp.rad(0), sp.rad(0)), toolframe_matrix)
    print("x: " + str(x))
    print("y: " + str(y))
    print("z: " + str(z))
    print("yaw: " + str(np.degrees(yaw)))
    print("pitch: " + str(np.degrees(pitch)))
    print("roll: " + str(np.degrees(roll)))
    print()

    # j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = arm.inverse_kinematics((x, y, z, yaw, pitch, roll), toolframe_matrix)
    j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle = arm.inverse_kinematics((308.557, 0, 521.354, np.radians(0), np.radians(0),  np.radians(0)), toolframe_matrix)
    print("j1_angle: " + str(np.degrees(j1_angle)))
    print("j2_angle: " + str(np.degrees(j2_angle)))
    print("j3_angle: " + str(np.degrees(j3_angle)))
    print("j4_angle: " + str(np.degrees(j4_angle)))
    print("j5_angle: " + str(np.degrees(j5_angle)))
    print("j6_angle: " + str(np.degrees(j6_angle)))