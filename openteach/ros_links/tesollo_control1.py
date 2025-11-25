#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
from control_msgs.msg import JointTrajectoryControllerState

# -------------------- Topics & Defaults -------------------- #
LEFT_JOINT_STATES_TOPIC = '/left/joint_states'
RIGHT_JOINT_STATES_TOPIC = '/right/joint_states'

LEFT_JTC_CMD_TOPIC   = '/left/dg3f_b_controller/joint_trajectory'
RIGHT_JTC_CMD_TOPIC  = '/right/dg3f_b_controller/joint_trajectory'

LEFT_JTC_STATE_TOPIC  = '/left/dg3f_b_controller/controller_state'
RIGHT_JTC_STATE_TOPIC = '/right/dg3f_b_controller/controller_state'

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
    def __init__(self, *, rate_hz: float = 60.0,
                 joint_names: Optional[List[str]] = None,
                 hand_type: str = 'right',
                 auto_reorder_to_controller: bool = True):
        """
        hand_type: 'right' or 'left'
        auto_reorder_to_controller: 若 True，按控制器 state 的 joint_names 顺序重排后再发送
        """
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__('dex_arm')

        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.auto_reorder_to_controller = bool(auto_reorder_to_controller)

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

        # 选择左右手的topic
        if hand_type == 'right':
            state_topic = RIGHT_JOINT_STATES_TOPIC
            cmd_topic = RIGHT_JTC_CMD_TOPIC
            ctrl_state_topic = RIGHT_JTC_STATE_TOPIC
        else:
            state_topic = LEFT_JOINT_STATES_TOPIC
            cmd_topic = LEFT_JTC_CMD_TOPIC
            ctrl_state_topic = LEFT_JTC_STATE_TOPIC

        # Subscriptions / Publishers
        self.tesollo_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, state_topic, self._callback_joint_state, qos_profile=state_qos)

        self.ctrl_joint_names: Optional[List[str]] = None
        self.create_subscription(
            JointTrajectoryControllerState,
            ctrl_state_topic,
            self._callback_ctrl_state,
            qos_profile=QoSProfile(depth=10)
        )

        self.traj_pub = self.create_publisher(JointTrajectory, cmd_topic, cmd_qos)

        # Command joint name order (user-specified or default)
        self.joint_names = list(joint_names) if joint_names else list(JOINT_NAMES)
        self.ndof = len(self.joint_names)

        # Internal state
        self._last_commanded: Optional[List[float]] = None
        self._last_commanded_time: float = 0.0

        # Spin the node in a background executor thread
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(
            f"DexArmControl ready: rate={self.rate_hz} Hz, cmd_topic={cmd_topic}, "
            f"state_topic={state_topic}, ctrl_state_topic={ctrl_state_topic}\n"
            f"Cmd Joint order: {self.joint_names}"
        )

    # ----------------- Callbacks -----------------
    def _callback_joint_state(self, msg: JointState):
        self.tesollo_joint_state = msg

    def _callback_ctrl_state(self, msg: JointTrajectoryControllerState):
        # 保存控制器权威的关节名顺序
        self.ctrl_joint_names = list(msg.joint_names)

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
    def move_hand(self, tesollo_angles: Iterable[float], dt: Optional[float] = 0.5):
        """
        Publish a single-point JointTrajectory to the controller.

        Args:
            tesollo_angles: Iterable of joint positions matching `self.joint_names` order.
            dt: time_from_start (seconds), default 0.5s.
        """
        q = list(map(float, tesollo_angles))
        if len(q) != self.ndof:
            raise ValueError(f"Expected {self.ndof} positions, got {len(q)}")

        msg = JointTrajectory()
        pt = JointTrajectoryPoint()

        # 若拿到了控制器的权威顺序，按其顺序重排并发送
        if self.ctrl_joint_names and self.auto_reorder_to_controller:
            # 集合必须一致
            if set(self.ctrl_joint_names) != set(self.joint_names):
                self.get_logger().error(
                    f"[ABORT] Controller joints {self.ctrl_joint_names} and cmd joints {self.joint_names} "
                    f"don't match as sets."
                )
                return
            # 重排
            name_to_pos = dict(zip(self.joint_names, q))
            q_send = [name_to_pos[n] for n in self.ctrl_joint_names]
            msg.joint_names = list(self.ctrl_joint_names)
            pt.positions = q_send
        else:
            # 否则按本地定义的顺序发送（controller 通常也会按名字映射）
            msg.joint_names = list(self.joint_names)
            pt.positions = q

        # time_from_start
        use_dt = float(self.dt if dt is None else dt)
        if use_dt <= 0.0:
            use_dt = 0.5
        pt.time_from_start.sec = int(use_dt)
        pt.time_from_start.nanosec = int((use_dt - int(use_dt)) * 1e9)

        msg.points = [pt]
        self.traj_pub.publish(msg)

        self._last_commanded = q
        self._last_commanded_time = time.time()

    def home_hand(self, dt: float = 0.8):
        self.move_hand(TESOLLO_HOME_VALUES, dt=dt)

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


# -------------------- Quick self-test (optional) -------------------- #
if __name__ == "__main__":
    # 示例：右手，发送一个轻微位姿
    node = DexArmControl(hand_type='right', rate_hz=60.0)
    # 等待几百毫秒获取一次控制器state（为了拿到 ctrl_joint_names）
    time.sleep(0.3)
    try:
        demo_q = TESOLLO_HOME_VALUES
        node.move_hand(demo_q, dt=0.6)
        print("Sent one-point trajectory.")
        time.sleep(0.5)
    finally:
        node.shutdown()
