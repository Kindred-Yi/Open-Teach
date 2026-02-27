#!/usr/bin/env python3


import time
import threading
import numpy as np
import csv
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
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray, Int32

# -------------------- Topics & Defaults -------------------- #
#JOINT_STATES_TOPIC = '/joint_states'
#JTC_CMD_TOPIC = '/delto_3f_controller/joint_trajectory'

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

        if hand_type == 'right':
            JOINT_STATES_TOPIC = '/right/joint_states'
            JTC_CMD_TOPIC = '/right/dg3f_b_controller/joint_trajectory'
            JTC_STATE_TOPIC = '/right/dg3f_b_controller/controller_state'
        else:
            JOINT_STATES_TOPIC = '/left/joint_states'
            JTC_CMD_TOPIC = '/left/dg3f_b_controller/joint_trajectory'
            JTC_STATE_TOPIC = '/left/dg3f_b_controller/controller_state'

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

        # Subscribe to controller state (includes reference/desired and feedback/actual)
        self.controller_state: Optional[JointTrajectoryControllerState] = None
        self.create_subscription(
            JointTrajectoryControllerState,
            JTC_STATE_TOPIC,
            self._callback_controller_state,
            qos_profile=state_qos,
        )

        # Publisher to JointTrajectoryController command topic
        self.traj_pub = self.create_publisher(JointTrajectory, JTC_CMD_TOPIC, cmd_qos)

        # Subscriber for coordination mode (from CoordinationPredictor)
        # Use BEST_EFFORT to match the predictor's publisher QoS
        coord_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._coordination_mode: int = 0
        self.create_subscription(
            Int32,
            '/coordination_mode',
            self._callback_coordination_mode,
            qos_profile=coord_qos,
        )

        # Joint name order for commands
        self.joint_names: List[str] = list(joint_names) if joint_names else list(JOINT_NAMES)
        self.ndof = len(self.joint_names)

        # Track last commanded values for debugging
        self._last_commanded: Optional[List[float]] = None
        self._last_commanded_time: float = 0.0

        # CSV logging for sent commands
        self._cmd_log_file = open(f'/tmp/openteach_{hand_type}_sent_commands.csv', 'w', newline='')
        self._cmd_writer = csv.writer(self._cmd_log_file)
        self._cmd_writer.writerow(['timestamp'] + self.joint_names)

        # Spin the node in a background executor thread
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(
            f"DexArmControl ready: rate={self.rate_hz} Hz\n"
            f"  cmd_topic={JTC_CMD_TOPIC}\n"
            f"  state_topic={JOINT_STATES_TOPIC}\n"
            f"  controller_state_topic={JTC_STATE_TOPIC}\n"
            f"  Joint order: {self.joint_names}"
        )

    # ----------------- Callbacks -----------------
    def _callback_joint_state(self, msg: JointState):
        self.tesollo_joint_state = msg

    def _callback_controller_state(self, msg: JointTrajectoryControllerState):
        self.controller_state = msg

    def _callback_coordination_mode(self, msg: Int32):
        self._coordination_mode = msg.data

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
        """Get commanded (reference/desired) hand state from controller_state topic."""
        if self.controller_state is None:
            return None

        # Use 'reference' field which contains the desired trajectory point
        ref = self.controller_state.reference

        # Extract timestamp from header
        timestamp = ref.time_from_start.sec + ref.time_from_start.nanosec * 1e-9

        return dict(
            position=np.array(ref.positions, dtype=np.float32),
            velocity=np.array(ref.velocities, dtype=np.float32) if ref.velocities else np.array([], dtype=np.float32),
            effort=np.array(ref.effort, dtype=np.float32) if ref.effort else np.array([], dtype=np.float32),
            timestamp=timestamp,
        )

    def get_commanded_hand_joint_position(self):
        """Get commanded (reference) joint positions from controller_state topic."""
        if self.controller_state is None:
            return None
        return np.array(self.controller_state.reference.positions, dtype=np.float32)

    def get_coordination_mode(self):
        """Get current coordination mode from CoordinationPredictor.

        Returns:
            int: 0=NoAction, 1=LooselyCoupled, 2=UnimanualLeft,
                 3=UnimanualRight, 4=TightlyAsymLDom, 5=TightlyAsymRDom,
                 6=TightlySymmetric
        """
        return self._coordination_mode

    def get_full_controller_state(self):
        """
        Get complete controller state including both actual and commanded states.
        This combines data from controller_state topic for a single unified recording.

        Returns:
            dict with keys:
                - actual_position: actual joint positions
                - actual_velocity: actual joint velocities
                - actual_effort: actual joint efforts
                - commanded_position: desired/reference joint positions
                - commanded_velocity: desired/reference joint velocities
                - commanded_effort: desired/reference joint efforts
                - timestamp: current timestamp
        """
        if self.controller_state is None:
            return None

        ref = self.controller_state.reference  # Desired/commanded state
        fb = self.controller_state.feedback    # Actual/measured state
        er = self.controller_state.error

        # Use header timestamp if available, otherwise use time_from_start
        if hasattr(self.controller_state, 'header'):
            timestamp = self.controller_state.header.stamp.sec + self.controller_state.header.stamp.nanosec * 1e-9
        else:
            timestamp = time.time()

        return dict(
            # Actual (feedback) state
            actual_position=np.array(fb.positions, dtype=np.float32),
            actual_velocity=np.array(fb.velocities, dtype=np.float32) if fb.velocities else np.array([], dtype=np.float32),
            actual_effort=np.array(fb.effort, dtype=np.float32) if fb.effort else np.array([], dtype=np.float32),

            # Commanded (reference) state
            commanded_position=np.array(ref.positions, dtype=np.float32),
            commanded_velocity=np.array(ref.velocities, dtype=np.float32) if ref.velocities else np.array([], dtype=np.float32),
            commanded_effort=np.array(ref.effort, dtype=np.float32) if ref.effort else np.array([], dtype=np.float32),

            error_position=np.array(er.positions, dtype=np.float32),
            error_velocity=np.array(er.velocities, dtype=np.float32) if er.velocities else np.array([], dtype=np.float32),
            error_effort=np.array(er.effort, dtype=np.float32) if er.effort else np.array([], dtype=np.float32),

            # Timestamp
            timestamp=timestamp,
        )

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

        #use_dt = float(self.dt if dt is None else dt)
        use_dt = 0.05
        pt.time_from_start.sec = int(use_dt)
        pt.time_from_start.nanosec = int((use_dt - int(use_dt)) * 1e9)

        msg.points = [pt]
        self.traj_pub.publish(msg)

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
            if hasattr(self, '_cmd_log_file'):
                self._cmd_log_file.close()
            if hasattr(self, '_executor'):
                self._executor.shutdown()
        finally:
            try:
                self.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
