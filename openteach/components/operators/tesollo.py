from copy import deepcopy as copy
import numpy as np
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from .operator import Operator

from openteach.robot.tesollo.tesollo_retargeters import TesolloJointControl
from openteach.robot.tesollo.tesollo import TesolloHand
from openteach.utils.files import *
from openteach.utils.timer import FrequencyTimer
from openteach.constants import *

class TesolloHandOperator(Operator):
    def __init__(self, host, transformed_keypoints_port, finger_configs):
        self.notify_component_start('tesollo hand operator')
        self._host, self._port = host, transformed_keypoints_port
        # Subscriber for the transformed hand keypoints
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host,
            port = self._port,
            topic = 'transformed_hand_coords'
        )
        # Subscriber for the transformed arm frame
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host,
            port = self._port,
            topic = 'transformed_hand_frame'
        )
        # Initializing the  finger configs
        self.finger_configs = finger_configs
        
        #Initializing the solvers for tesollo hand
        self.finger_joint_solver = TesolloJointControl()

        
        self._robot = TesolloHand()

        # Initialzing the moving average queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
        }



        self._timer = FrequencyTimer(VR_FREQ)

        # Using 3 dimensional thumb motion or two dimensional thumb motion
        if self.finger_configs.get('three_dim', False):
            self.thumb_angle_calculator = self._get_3d_thumb_angles
        else:
            self.thumb_angle_calculator = self._get_2d_thumb_angles

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber
    
    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    # This function differentiates between the real robot and simulation
    def return_real(self):
        return True

    # Get the transformed finger coordinates
    def _get_finger_coords(self):
        raw_keypoints = self.transformed_hand_keypoint_subscriber.recv_keypoints()
        return dict(
            index = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['index']]]),
            middle = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['middle']]]),
            thumb =  np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['thumb']]])
        )
        
    # Generate frozen angles for the fingers
    def _generate_frozen_angles(self, joint_angles, finger_type):
        for idx in range(TESOLLO_JOINTS_PER_FINGER):
            if idx > 0:
                joint_angles[idx + TESOLLO_JOINT_OFFSETS[finger_type]] = 0.05
            else:
                joint_angles[idx + TESOLLO_JOINT_OFFSETS[finger_type]] = 0

        return joint_angles

    # Get robot thumb angles when moving only in 2D motion
    def _get_2d_thumb_angles(self, thumb_keypoints, curr_angles):
        # For tesollo, use joint position control like other fingers
        return self.finger_joint_solver.calculate_finger_angles(
            finger_type='thumb',
            finger_joint_coords=thumb_keypoints,
            curr_angles=curr_angles,
            moving_avg_arr=self.moving_average_queues['thumb']
        )

    # Get robot thumb angles when moving in 3D motion
    def _get_3d_thumb_angles(self, thumb_keypoints, curr_angles):
        # For tesollo, use joint position control like other fingers
        return self.finger_joint_solver.calculate_finger_angles(
            finger_type='thumb',
            finger_joint_coords=thumb_keypoints,
            curr_angles=curr_angles,
            moving_avg_arr=self.moving_average_queues['thumb']
        )
    
    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        hand_keypoints = self._get_finger_coords()
        desired_joint_angles = copy(self.robot.get_joint_position())

        # Movement for the index finger with option to freeze the finger
        if not self.finger_configs['freeze_index'] and not self.finger_configs['no_index']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'index',
                finger_joint_coords = hand_keypoints['index'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['index']
            )
        elif self.finger_configs['freeze_index']:
            self._generate_frozen_angles(desired_joint_angles, 'index')
        else:
            print("No index")
            pass

        # Movement for the middle finger option to freeze the finger
        if not self.finger_configs['freeze_middle'] and not self.finger_configs['no_middle']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'middle',
                finger_joint_coords = hand_keypoints['middle'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['middle']
            )
        elif self.finger_configs['freeze_middle']:
            self._generate_frozen_angles(desired_joint_angles, 'middle')
        else :
            print("No Middle")
            pass

        # Movement for the thumb finger with option to freeze the finger
        if not self.finger_configs['freeze_thumb'] and not self.finger_configs['no_thumb']:
            desired_joint_angles = self.thumb_angle_calculator(hand_keypoints['thumb'], desired_joint_angles) # Passing all thumb coordinates
        elif self.finger_configs['freeze_thumb']:
            self._generate_frozen_angles(desired_joint_angles, 'thumb')
        else:
            print("No thumb")
            pass
        
        # Move the robot
        self.robot.move(desired_joint_angles)