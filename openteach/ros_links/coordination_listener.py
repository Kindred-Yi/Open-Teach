"""
Lightweight ROS2 node for subscribing to /coordination_mode.
Used by Franka arm operators which don't have tesollo_control2's DexArmControl.
Also handles cross-arm pose sharing for Tightly Symmetric mode.
"""

import threading
import numpy as np
from copy import deepcopy as copy

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError as e:
    raise ImportError("Failed to import rclpy.") from e

from std_msgs.msg import Int32, Float32MultiArray


class CoordinationListener(Node):
    """ROS2 node that subscribes to coordination_mode and arm target poses."""

    def __init__(self, hand_type='right'):
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__(f'coordination_listener_{hand_type}')

        self.hand_type = hand_type

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Coordination mode uses BEST_EFFORT (matching predictor's publisher)
        coord_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to coordination mode
        self._coordination_mode = 0
        self.create_subscription(
            Int32, '/coordination_mode',
            self._cb_coord_mode, coord_qos)

        # Publish own arm cartesian pose (7 floats: pos xyz + quat xyzw)
        self.arm_pose_pub = self.create_publisher(
            Float32MultiArray,
            f'/{hand_type}/arm_target_pose',
            cmd_qos)

        # Subscribe to the OTHER arm's target pose
        other = 'left' if hand_type == 'right' else 'right'
        self._other_arm_pose = None
        self.create_subscription(
            Float32MultiArray,
            f'/{other}/arm_target_pose',
            self._cb_other_arm_pose,
            cmd_qos)

        # Spin in background
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(f"CoordinationListener ({hand_type}) ready.")

    def _cb_coord_mode(self, msg):
        self._coordination_mode = msg.data

    def _cb_other_arm_pose(self, msg):
        self._other_arm_pose = np.array(msg.data, dtype=np.float64)

    def get_coordination_mode(self):
        return self._coordination_mode

    def get_other_arm_pose(self):
        """Get the other arm's cartesian target pose (7,): [x,y,z, qx,qy,qz,qw]."""
        if self._other_arm_pose is None:
            return None
        return copy(self._other_arm_pose)

    def publish_arm_pose(self, pose):
        """Publish own arm cartesian target pose (7,): [x,y,z, qx,qy,qz,qw]."""
        msg = Float32MultiArray()
        msg.data = pose.astype(np.float32).tolist()
        self.arm_pose_pub.publish(msg)

    def shutdown(self):
        try:
            self._executor.shutdown()
        finally:
            try:
                self.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
