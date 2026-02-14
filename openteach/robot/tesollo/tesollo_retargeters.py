import numpy as np
from abc import ABC
from copy import deepcopy as copy
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from openteach.utils.files import *
from openteach.utils.vectorops import *


class TesolloKinematicControl(ABC):
    def __init__(self, bounded_angles = True):
        np.set_printoptions(suppress = True)

        # Loading the Tesollo Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_link_info.yaml"))
        self.bound_info = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_bounds.yaml"))

        self.time_steps = self.bound_info['time_steps']

        self.bounded_angles = bounded_angles
        self.bounds = {}
        for finger in self.hand_configs['fingers'].keys():
            self.bounds[finger] = np.array(self.bound_info['jointwise_angle_bounds'][
                self.finger_configs['links_info'][finger]['offset'] : self.finger_configs['links_info'][finger]['offset'] + 4
            ])

    def _get_curr_finger_angles(self, curr_angles, finger_type):
        return np.array(curr_angles[
            self.finger_configs['links_info'][finger_type]['offset'] : self.finger_configs['links_info'][finger_type]['offset'] + 4
        ])


class TesolloJointControl(TesolloKinematicControl):
    def __init__(self, bounded_angles = False):
        super().__init__(bounded_angles)
        np.set_printoptions(suppress = True)

        self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        self.rotatory_scaling_factors = self.bound_info['rotatory_scaling_factors']

    def _get_filtered_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)
        desired_angles = np.array(copy(curr_angles))

        # Applying angular bounds
        if self.bounded_angles is True:
            del_finger_angles = avg_finger_angles - curr_finger_angles
            clipped_del_finger_angles = np.clip(del_finger_angles, - self.bounds[finger_type], self.bounds[finger_type])

            # Keep first two joints fixed, only update last two joints
            for idx in range(2, self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] += clipped_del_finger_angles[idx]

            # Set first two joints to fixed values
            desired_angles[self.finger_configs['links_info'][finger_type]['offset']] = calc_finger_angles[0]  # First joint
            desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + 1] = calc_finger_angles[1]  # Second joint
        else:
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx]

        return desired_angles 

    def calculate_finger_angles(self, finger_type, finger_joint_coords, curr_angles, moving_avg_arr, spread, grasp):
        translatory_angles = []
        translatory_angles.append(0)
        for idx in range(self.hand_configs['joints_per_finger'] - 2): # Ignoring the rotatory joint
            angle = calculate_angle(
                finger_joint_coords[idx + 1],
                finger_joint_coords[idx + 2],
                finger_joint_coords[idx + 3]
            )
            translatory_angles.append(angle * self.linear_scaling_factors[idx])

        # Handle rotatory scaling based on finger type
        if finger_type in self.rotatory_scaling_factors:
            rotatory_factor = self.rotatory_scaling_factors[finger_type]
        else:
            # Default scaling for unmapped fingers
            rotatory_factor = 0.02

        if finger_type == 'thumb':
            initial_angle = 0.0
        elif finger_type == 'index':
            initial_angle = -spread
        elif finger_type == 'middle':
            initial_angle = spread

        rotatory_angle = [self.calculate_finger_rotation(finger_joint_coords) * rotatory_factor + initial_angle]
        calc_finger_angles = [initial_angle] + translatory_angles

        #####
        angles = []
        angles.append(initial_angle)
        angles.append(0)
        angle_joint3 = calculate_angle(
          finger_joint_coords[1],
          finger_joint_coords[2],
          finger_joint_coords[3]
        )
        if finger_type == 'thumb':
            angle_joint3 = 1.8 * angle_joint3
        
        if grasp == 'precision':
            angle_joint3 = 1.5 * angle_joint3

        angles.append(angle_joint3 * self.linear_scaling_factors[0])

        angle_joint4 = calculate_angle(
          finger_joint_coords[2],
          finger_joint_coords[3],
          finger_joint_coords[4]
        )

        if grasp == 'precision':
            angle_joint4 = 0

        angles.append(angle_joint4 * self.linear_scaling_factors[1])
        filtered_angles = self._get_filtered_angles(finger_type, angles, curr_angles, moving_avg_arr)
        #####

        # filtered_angles = self._get_filtered_angles(finger_type, calc_finger_angles, curr_angles, moving_avg_arr)
        return filtered_angles

    def calculate_finger_rotation(self, finger_joint_coords):
        angle = calculate_angle(finger_joint_coords[0][:1], finger_joint_coords[1][:1], finger_joint_coords[-1][:1])
        
        # Checking if the finger tip is on the left side or the right side of the knuckle
        knuckle_vector = finger_joint_coords[1] - finger_joint_coords[0]
        tip_vector = finger_joint_coords[-1] - finger_joint_coords[0]
        knuckle_vector_slope = knuckle_vector[1] / knuckle_vector[0]
        tip_vector_slope = tip_vector[1] / tip_vector[0]

        if knuckle_vector_slope > tip_vector_slope:
            return angle
        else:
            return -1 * angle