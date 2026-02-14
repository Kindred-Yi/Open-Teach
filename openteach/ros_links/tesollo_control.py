#!/usr/bin/env python3


import time
import threading
import numpy as np
from copy import deepcopy as copy
from typing import Iterable, List, Optional

# Try to import ROS 2 Python API with a helpful error if missing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError as e:
    raise ImportError(
        "Failed to import rclpy. For Ubuntu 22.04 + ROS 2 Humble inside Conda, "
        "install RoboStack packages (e.g., `mamba install -c robostack -c conda-forge "
        "ros-humble-ros-base ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs`)."
    ) from e

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -------------------- Topics & Defaults -------------------- #
JOINT_STATES_TOPIC = '/joint_states'
JTC_CMD_TOPIC = '/delto_3f_controller/joint_trajectory'

# Must match your controller YAML order (joint_trajectory_controller.joints)
JOINT_NAMES = [
    'j_dg_1_1','j_dg_1_2','j_dg_1_3','j_dg_1_4',
    'j_dg_2_1','j_dg_2_2','j_dg_2_3','j_dg_2_4',
    'j_dg_3_1','j_dg_3_2','j_dg_3_3','j_dg_3_4',
]

TESOLLO_HOME_VALUES = [
    0.0, -0.17453293, 0.78539816, 0.78539816,           # Index
    0.0, -0.17453293, 0.78539816, 0.78539816,           # Middle
    1.04719755, 0.43633231, 0.26179939, 0.78539816      # Thumb
]


class DexArmControl(Node):
    def __init__(self, *, rate_hz: float = 60.0, joint_names: Optional[List[str]] = None, hand_type: str = 'right'):
        # Initialize rclpy and node
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__('dex_arm')

        # Teleoperation period (used for time_from_start)
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz

        # QoS setup
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to aggregated joint states from ros2_control
        self.tesollo_joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._callback_joint_state,
            qos_profile=state_qos,
        )

        # Publisher to JointTrajectoryController command topic
        self.traj_pub = self.create_publisher(JointTrajectory, JTC_CMD_TOPIC, cmd_qos)

        # Joint name order for commands
        self.joint_names: List[str] = list(joint_names) if joint_names else list(JOINT_NAMES)
        self.ndof = len(self.joint_names)

        # Track last commanded values for debugging
        self._last_commanded: Optional[List[float]] = None
        self._last_commanded_time: float = 0.0

        # Spin the node in a background executor thread
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(
            f"DexArmControl ready: rate={self.rate_hz} Hz, cmd_topic={JTC_CMD_TOPIC}, state_topic={JOINT_STATES_TOPIC}\n"
            f"Joint order: {self.joint_names}"
        )

    # ----------------- Callbacks -----------------
    def _callback_joint_state(self, msg: JointState):
        self.tesollo_joint_state = msg

    # ----------------- State getters -----------------
    def get_hand_state(self):
        if self.tesollo_joint_state is None:
            return None
        raw = copy(self.tesollo_joint_state)
        return dict(
            position=np.array(raw.position, dtype=np.float32),
            velocity=np.array(raw.velocity, dtype=np.float32) if raw.velocity else np.array([], dtype=np.float32),
            effort=np.array(raw.effort, dtype=np.float32) if raw.effort else np.array([], dtype=np.float32),
            timestamp=raw.header.stamp.sec + raw.header.stamp.nanosec * 1e-9,
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

    # ----------------- Command API -----------------
    def move_hand(self, tesollo_angles: Iterable[float], dt: Optional[float] = None):
        """
        Publish a single-point JointTrajectory to the controller.

        Args:
            tesollo_angles: Iterable of joint positions matching `self.joint_names` order.
            dt: Optional time_from_start (seconds). Defaults to 1/rate_hz.
        """
        q = list(map(float, tesollo_angles))
        if len(q) != self.ndof:
            raise ValueError(f"Expected {self.ndof} positions, got {len(q)}")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = q

        use_dt = float(self.dt if dt is None else dt)
        pt.time_from_start.sec = int(use_dt)
        pt.time_from_start.nanosec = int((use_dt - int(use_dt)) * 1e9)

        msg.points = [pt]
        self.traj_pub.publish(msg)

        self._last_commanded = q
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
