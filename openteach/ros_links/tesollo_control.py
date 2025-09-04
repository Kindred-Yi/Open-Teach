import time
import threading
import numpy as np
from copy import deepcopy as copy

# Try to import ROS 2 Python API with a helpful error if missing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

except ImportError as e:
    raise ImportError(
        "Failed to import rclpy. For Ubuntu 22.04 + ROS 2 Humble inside Conda, "
        "install RoboStack packages (e.g., `mamba install -c robostack -c conda-forge "
        "ros-humble-ros-base ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs`)."
    ) from e

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


TESOLLO_JOINT_STATE_TOPIC = '/gripper/joint_states'
TESOLLO_TARGET_JOINT_STATE_TOPIC = '/gripper/target_joint'

TESOLLO_HOME_VALUES = [
    0, -0.17453293, 0.78539816, 0.78539816,           # Index
    0, -0.17453293, 0.78539816, 0.78539816,           # Middle
    1.04719755, 0.43633231, 0.26179939, 0.78539816    # Thumb
]


class DexArmControl(Node):
    def __init__(self, record_type=None, robot_type='both'):
        # Initialize rclpy and node
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__('dex_arm')

        # Use real-time QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.tesollo_joint_state = None
        self.create_subscription(
            JointState,
            TESOLLO_JOINT_STATE_TOPIC,
            self._callback_tesollo_joint_state,
            qos_profile=qos
        )

        self.target_pub = self.create_publisher(
            Float32MultiArray,
            TESOLLO_TARGET_JOINT_STATE_TOPIC,
            1000
        )

        # Track last commanded joint values (no dedicated commanded topic)
        self._last_commanded = None
        self._last_commanded_time = 0.0

        # Spin the node in a background executor thread
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ----------------- Callbacks -----------------
    def _callback_tesollo_joint_state(self, joint_state):
        self.tesollo_joint_state = joint_state

    # ----------------- State getters -----------------
    def get_hand_state(self):
        if self.tesollo_joint_state is None:
            return None
        raw = copy(self.tesollo_joint_state)
        return dict(
            position=np.array(raw.position, dtype=np.float32),
            velocity=np.array(raw.velocity, dtype=np.float32) if raw.velocity else np.array([], dtype=np.float32),
            effort=np.array(raw.effort, dtype=np.float32) if raw.effort else np.array([], dtype=np.float32),
            timestamp=raw.header.stamp.sec + raw.header.stamp.nanosec * 1e-9
        )

    def get_hand_position(self):
        if self.tesollo_joint_state is None:
            return None
        return np.array(self.tesollo_joint_state.position, dtype=np.float32)

    def get_hand_velocity(self):
        if self.tesollo_joint_state is None:
            return None
        vel = getattr(self.tesollo_joint_state, 'velocity', [])
        if vel:
            return np.array(vel, dtype=np.float32)
        # If velocity is not populated, return zeros with correct length
        pos = getattr(self.tesollo_joint_state, 'position', [])
        return np.zeros(len(pos), dtype=np.float32) if pos else np.array([], dtype=np.float32)

    def get_hand_torque(self):
        if self.tesollo_joint_state is None:
            return None
        eff = getattr(self.tesollo_joint_state, 'effort', [])
        if eff:
            return np.array(eff, dtype=np.float32)
        pos = getattr(self.tesollo_joint_state, 'position', [])
        return np.zeros(len(pos), dtype=np.float32) if pos else np.array([], dtype=np.float32)

    def get_commanded_hand_state(self):
        if self._last_commanded is None:
            return None
        return dict(
            position=np.array(self._last_commanded, dtype=np.float32),
            velocity=np.array([], dtype=np.float32),
            effort=np.array([], dtype=np.float32),
            timestamp=self._last_commanded_time,
        )

    def get_commanded_hand_joint_position(self):
        if self._last_commanded is None:
            return None
        return np.array(self._last_commanded, dtype=np.float32)

    def move_hand(self, tesollo_angles):
        """
        Pubilish Float32MultiArray to /gripper/target_joint
        """
        msg = Float32MultiArray()
        msg.data = list(map(float, tesollo_angles))
        self.target_pub.publish(msg)
        self._last_commanded = list(map(float, tesollo_angles))
        self._last_commanded_time = time.time()

    def home_hand(self):
        self.move_hand(TESOLLO_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    def move_robot(self, tesollo_angles, arm_angles=None):
        self.move_hand(tesollo_angles)

    def home_robot(self):
        self.home_hand()

    # Graceful shutdown helpers
    def shutdown(self):
        try:
            if hasattr(self, '_executor'):
                self._executor.shutdown()
        finally:
            try:
                self.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
