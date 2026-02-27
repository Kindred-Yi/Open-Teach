import numpy as np
from copy import deepcopy as copy
from openteach.ros_links.tesollo_control2 import DexArmControl
from openteach.constants import *
from openteach.utils.files import get_yaml_data, get_path_in_package
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.robot.robot import RobotWrapper

class TesolloRightHand(RobotWrapper):
    def __init__(self, ip=None, port=None, dummy=False,
                 host='0.0.0.0',
                 wrist_frame_port=None,
                 hand_command_port=None,
                 **kwargs):
        # Initialize Tesollo controller with Modbus TCP
        self._controller = DexArmControl(
            hand_type='right',
        )

        # For robot configurations
        self._joint_limit_config = get_yaml_data(get_path_in_package("robot/tesollo/configs/tesollo_link_info.yaml"))['links_info']

        self._data_frequency = 60
        self._last_wrist_pose = None  # 缓存最新的 wrist pose
        self._cached_vr_cmd = None   # 缓存最新的 VR command

        # ZMQ subscribers for wrist frame and VR hand command (for data collection)
        self._wrist_frame_sub = None
        self._hand_cmd_sub = None

        if wrist_frame_port is not None:
            self._wrist_frame_sub = ZMQKeypointSubscriber(
                host=host, port=wrist_frame_port, topic='transformed_hand_frame')
            print(f"[TesolloRightHand] ZMQ wrist frame subscriber on {host}:{wrist_frame_port}")

        if hand_command_port is not None:
            self._hand_cmd_sub = ZMQKeypointSubscriber(
                host=host, port=hand_command_port, topic='hand_command')
            print(f"[TesolloRightHand] ZMQ hand command subscriber on {host}:{hand_command_port}")

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

    def _poll_zmq(self):
        """Poll ZMQ subscribers non-blocking and cache latest values."""
        if self._wrist_frame_sub is not None:
            frame = self._wrist_frame_sub.recv_keypoints(flags=1)
            if frame is not None:
                self._last_wrist_pose = np.asanyarray(frame).reshape(4, 3)

        if self._hand_cmd_sub is not None:
            cmd = self._hand_cmd_sub.recv_keypoints(flags=1)
            if cmd is not None:
                self._cached_vr_cmd = np.asanyarray(cmd, dtype=np.float32)

    def get_full_controller_state(self):
        """Get controller state with VR command and wrist pose from ZMQ."""
        hw_state = self._controller.get_full_controller_state()
        if hw_state is None:
            return None

        # Poll ZMQ for latest wrist and VR command data
        self._poll_zmq()

        # Replace hardware commanded with VR commanded
        if self._cached_vr_cmd is not None:
            hw_state['commanded_position'] = self._cached_vr_cmd
            hw_state['error_position'] = self._cached_vr_cmd - hw_state['actual_position']

        # Add wrist pose
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