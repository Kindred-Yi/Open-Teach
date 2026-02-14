import numpy as np
from copy import deepcopy as copy
from openteach.ros_links.tesollo_control2 import DexArmControl 
from openteach.constants import *
from openteach.utils.files import get_yaml_data, get_path_in_package
from openteach.robot.robot import RobotWrapper

class TesolloRightHand(RobotWrapper):
    def __init__(self, ip=None, port=None, dummy=False, **kwargs):
        # Initialize Tesollo controller with Modbus TCP
        self._controller = DexArmControl(
            hand_type='right',
        )

        # For robot configurations
        self._joint_limit_config = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_link_info.yaml"))['links_info']

        self._data_frequency = 60
        self._last_wrist_pose = None  # 用于缓存最新的 wrist pose

    @property
    def name(self):
        return 'right tesollo'

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'commanded_joint_states': self.get_commanded_joint_state,
            'controller_state': self.get_full_controller_state
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_hand_state()

    def get_commanded_joint_state(self):
        return self._controller.get_commanded_hand_state()

    def get_joint_position(self):
        return self._controller.get_hand_position()

    def get_joint_velocity(self):
        return self._controller.get_hand_velocity()

    def get_joint_torque(self):
        return self._controller.get_hand_torque()

    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()

    def get_vr_commanded_position(self):
        """Get VR/OpenTeach sent command (bypasses hardware-level modifications)."""
        return self._controller.get_vr_sent_command()

    def get_full_controller_state(self):
        """Get controller state with VR command instead of hardware-modified command."""
        hw_state = self._controller.get_full_controller_state()
        if hw_state is None:
            return None

        # Replace hardware commanded with VR commanded
        vr_cmd = self._controller.get_vr_sent_command()
        if vr_cmd is not None:
            hw_state['commanded_position'] = vr_cmd
            # Recalculate error based on VR command
            hw_state['error_position'] = vr_cmd - hw_state['actual_position']

        # Add wrist pose from VR
        wrist_pose = self._controller.get_wrist_pose()
        #if wrist_pose is not None:
        #    hw_state['wrist_position'] = wrist_pose[0]  # origin_coord
        #    hw_state['wrist_orientation'] = wrist_pose[1:]  # [cross, normal, direction]
        
        if wrist_pose is not None:
            self._last_wrist_pose = wrist_pose    # 缓存最新有效值
        if self._last_wrist_pose is not None:
            hw_state['wrist_position'] = self._last_wrist_pose[0]
            hw_state['wrist_orientation'] = self._last_wrist_pose[1:]
        else:
            hw_state['wrist_position'] = np.zeros(3, dtype=np.float32)
            hw_state['wrist_orientation'] = np.zeros((3, 3), dtype=np.float32)

        return hw_state

    def get_coordination_mode(self):
        """Get current coordination mode from CoordinationPredictor."""
        return self._controller.get_coordination_mode()

    def set_wrist_pose(self, hand_frame):
        """Set wrist pose (hand frame) to be published via ROS2."""
        self._controller.set_wrist_pose(hand_frame)

    # Getting random position initializations for the fingers
    def _get_finger_limits(self, finger_type):
        finger_min = np.array(self._joint_limit_config[finger_type]['joint_min'])
        finger_max = np.array(self._joint_limit_config[finger_type]['joint_max'])
        return finger_min, finger_max

    def _get_finger_random_angles(self, finger_type):
        finger_low_limit, finger_high_limit = self._get_finger_limits(finger_type)

        random_angles = np.zeros((TESOLLO_JOINTS_PER_FINGER))
        for idx in range(TESOLLO_JOINTS_PER_FINGER - 1): # ignoring the base
            random_angles[idx + 1] = 0.8 * (finger_low_limit[idx + 1] + (np.random.rand() * (finger_high_limit[idx + 1] - finger_low_limit[idx + 1])))

        random_angles[0] = -0.1 + (np.random.rand() * 0.2) # Base angle
        return random_angles

    def set_random_position(self):
        random_angles = []
        for finger_type in ['index', 'middle', 'thumb']:
            random_angles.append(self._get_finger_random_angles(finger_type))

        target_angles = np.hstack(random_angles)
        self.move(target_angles)

    # Movement functions
    def home(self):
        self._controller.home_hand()

    def move(self, angles):
        self._controller.move_hand(angles)

    def move_coords(self, input_coords):
        # For hand robots like Tesollo, coordinate-based movement might not be directly applicable
        # This method is required by the abstract base class but may not be used for hand control
        raise NotImplementedError("Coordinate-based movement not implemented for Tesollo hand")