from copy import deepcopy as copy
import numpy as np
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from .operator import Operator

from openteach.robot.tesollo.tesollo_retargeters import TesolloJointControl
from openteach.robot.tesollo.tesollo_right import TesolloRightHand
from openteach.utils.files import *
from openteach.utils.timer import FrequencyTimer
from openteach.utils.vectorops import calculate_vector_angle
from openteach.constants import *

# Coordination mode labels
COORD_NO_ACTION = 0
COORD_LOOSELY_COUPLED = 1
COORD_UNIMANUAL_LEFT = 2
COORD_UNIMANUAL_RIGHT = 3
COORD_TIGHTLY_ASYM_LDOM = 4
COORD_TIGHTLY_ASYM_RDOM = 5
COORD_TIGHTLY_SYMMETRIC = 6


class TesolloRightHandOperator(Operator):
    def __init__(self, host, transformed_keypoints_port, finger_configs, hand_command_port=None):
        self.notify_component_start('tesollo right hand operator')
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


        self._robot = TesolloRightHand()

        # Initialzing the moving average queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
        }

        self._timer = FrequencyTimer(60)

        # Pinch toggle state for grasp locking
        self._grasp_locked = False  # False = normal mode, True = locked mode (3rd joint = 0)
        self._prev_pinch_detected = False  # For edge detection
        self._pinch_threshold = 0.03  # Distance threshold for ring-thumb pinch (meters)

        # Coordination mode state
        self._locked_angles = None  # Cached angles when gripper is locked
        self._slowdown_alpha = 0.15  # Interpolation factor for speed reduction (smaller = slower)

        # ZMQ publisher for hand command intent (used by coordination predictor)
        self._hand_cmd_pub = None
        if hand_command_port is not None:
            from openteach.utils.network import ZMQKeypointPublisher
            self._hand_cmd_pub = ZMQKeypointPublisher(host, hand_command_port)
            print(f"[TesolloLeft] Hand command ZMQ publisher on port {hand_command_port}")

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
            thumb =  np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['thumb']]]),
            ring_tip = raw_keypoints[OCULUS_JOINTS['ring'][-1]],  # Ring fingertip
            thumb_tip = raw_keypoints[OCULUS_JOINTS['thumb'][-1]]  # Thumb fingertip
        )

    def _detect_pinch_and_toggle(self, hand_keypoints):
        """
        Detect ring finger and thumb pinch gesture and toggle grasp lock state.
        Uses edge detection to only toggle on pinch start (not during hold).
        """
        # Calculate distance between ring fingertip and thumb fingertip
        ring_tip = hand_keypoints['ring_tip']
        thumb_tip = hand_keypoints['thumb_tip']
        distance = np.linalg.norm(ring_tip - thumb_tip)

        # Check if currently pinching
        is_pinching = distance < self._pinch_threshold

        # Edge detection: toggle only on rising edge (non-pinch -> pinch)
        if is_pinching and not self._prev_pinch_detected:
            self._grasp_locked = not self._grasp_locked
            print(f"[TesolloRight] Pinch detected! Grasp lock: {'ON' if self._grasp_locked else 'OFF'}")

        # Update previous state for next iteration
        self._prev_pinch_detected = is_pinching

    # Generate frozen angles for the fingers
    def _generate_frozen_angles(self, joint_angles, finger_type):
        for idx in range(TESOLLO_JOINTS_PER_FINGER):
            if idx > 0:
                joint_angles[idx + TESOLLO_JOINT_OFFSETS[finger_type]] = 0.05
            else:
                joint_angles[idx + TESOLLO_JOINT_OFFSETS[finger_type]] = 0

        return joint_angles

    def _should_lock_gripper(self, coord_mode):
        """Check if this (right) hand should lock its gripper based on coordination mode."""
        # Right hand locks when:
        #   - Unimanual Left (label 2): right hand not grasping but left is, lock right
        #     Actually per the decision tree: Unimanual Left means LEFT is grasping,
        #     so RIGHT should continue normally. Let me re-read the user's intent:
        #     "如果检测到某只手存在抓取,则在operator代码中锁定关节值"
        #     This means: if a hand IS grasping, lock THAT hand's joints to stabilize.
        #   - Unimanual Right (label 3): right hand is grasping -> lock right
        #   - Tightly Asym L-Dom (label 4): both hands grasping -> lock both
        #   - Tightly Asym R-Dom (label 5): both hands grasping -> lock both
        #   - Tightly Symmetric (label 6): both hands grasping -> lock both
        return coord_mode in (COORD_UNIMANUAL_RIGHT,
                              COORD_TIGHTLY_ASYM_LDOM,
                              COORD_TIGHTLY_ASYM_RDOM,
                              COORD_TIGHTLY_SYMMETRIC)

    def _should_slow_down(self, coord_mode):
        """Check if this (right) hand should slow down (non-dominant in tightly asymmetric)."""
        # Right hand slows down when left is dominant
        return coord_mode == COORD_TIGHTLY_ASYM_LDOM

    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        hand_keypoints = self._get_finger_coords()

        # Detect ring-thumb pinch and toggle grasp lock state
        self._detect_pinch_and_toggle(hand_keypoints)

        # Get coordination mode from CoordinationPredictor
        coord_mode = self.robot.get_coordination_mode()
        gripper_locked = self._should_lock_gripper(coord_mode)

        # Always compute desired angles from VR hand data (even when locked),
        # so the predictor can observe the operator's intent via vr_sent_command.
        desired_joint_angles = copy(self.robot.get_joint_position())
        index_vec = hand_keypoints['index'][2] - hand_keypoints['index'][1]
        middle_vec = hand_keypoints['middle'][2] - hand_keypoints['middle'][1]
        spread = calculate_vector_angle(index_vec, middle_vec)

        grasp_type = 'precision' if self._grasp_locked else 'power'

        # Movement for the index finger with option to freeze the finger
        if not self.finger_configs['freeze_index'] and not self.finger_configs['no_index']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'index',
                finger_joint_coords = hand_keypoints['index'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['index'],
                spread = spread,
                grasp = grasp_type
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
                moving_avg_arr = self.moving_average_queues['middle'],
                spread = spread,
                grasp = grasp_type
            )
        elif self.finger_configs['freeze_middle']:
            self._generate_frozen_angles(desired_joint_angles, 'middle')
        else :
            print("No Middle")
            pass

        # Movement for the thumb finger with option to freeze the finger
        if not self.finger_configs['freeze_thumb'] and not self.finger_configs['no_thumb']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'thumb',
                finger_joint_coords = hand_keypoints['thumb'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['thumb'],
                spread = spread,
                grasp = grasp_type
            )
        elif self.finger_configs['freeze_thumb']:
            self._generate_frozen_angles(desired_joint_angles, 'thumb')
        else:
            print("No thumb")
            pass

        # Publish VR intent via ZMQ (always, so predictor sees operator's hand state)
        if self._hand_cmd_pub is not None:
            self._hand_cmd_pub.pub_keypoints(desired_joint_angles, 'hand_command')

        if not gripper_locked:

            # Slow down if non-dominant in tightly asymmetric
            if self._should_slow_down(coord_mode):
                current_angles = self.robot.get_joint_position()
                if current_angles is not None:
                    alpha = self._slowdown_alpha
                    desired_joint_angles = (
                        current_angles * (1 - alpha) + desired_joint_angles * alpha
                    )

            # Move the robot
            self.robot.move(desired_joint_angles)
