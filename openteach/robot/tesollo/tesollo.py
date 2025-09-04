import numpy as np
from copy import deepcopy as copy
from openteach.ros_links.tesollo_control import DexArmControl 
from openteach.constants import *
from openteach.utils.files import get_yaml_data, get_path_in_package
from openteach.robot.robot import RobotWrapper

class TesolloHand(RobotWrapper):
    def __init__(self, ip=None, port=None, dummy=False, **kwargs):
        # Initialize Tesollo controller with Modbus TCP
        self._controller = DexArmControl(
            robot_type='tesollo',
        )

        # For robot configurations
        self._joint_limit_config = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_link_info.yaml"))['links_info']

        self._data_frequency = 300

    @property
    def name(self):
        return 'tesollo'

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_state
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