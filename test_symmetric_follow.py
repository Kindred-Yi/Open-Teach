"""
Test script for Tightly Symmetric coordination mode.

This script:
1. Initializes both Franka arms
2. Forces coordination_mode = 6 (TightlySymmetric) via ROS2
3. Records the initial left-right offset
4. Sends a simple trajectory to the right arm
5. Left arm follows using the same offset logic as franka_left.py

Usage:
    python test_symmetric_follow.py
"""

import numpy as np
import time
import threading
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Float32MultiArray

from openteach.robot.franka_left import FrankaLeft
from openteach.robot.franka_right import FrankaRight


# ── Helpers (same as in franka_left.py) ──────────────────────────────

# Static transformation between right arm base frame and left arm base frame.
# Face-to-face, 1.1168m apart, 180° rotation around Z. Self-inverse.
T_R2L = np.array([[-1.,  0., 0., 1.1168],
                  [ 0., -1., 0., 0.    ],
                  [ 0.,  0., 1., 0.    ],
                  [ 0.,  0., 0., 1.    ]])


def homo2cart(homo_mat):
    t = homo_mat[:3, 3]
    q = Rotation.from_matrix(homo_mat[:3, :3]).as_quat()
    return np.concatenate([t, q])


def cart2homo(cart):
    H = np.eye(4)
    H[:3, 3] = cart[:3]
    H[:3, :3] = Rotation.from_quat(cart[3:7]).as_matrix()
    return H


def compute_follower_pose(right_cart, offset_world):
    """Compute left arm target given right arm pose and world-frame offset.

    1. Compute left target in R (world) frame: T_left_R = T_right_R @ offset_world
    2. Convert to L frame: T_left_L = T_R2L @ T_left_R
    """
    T_right_R = cart2homo(right_cart)
    T_left_R = T_right_R @ offset_world
    T_left_L = T_R2L @ T_left_R
    return homo2cart(T_left_L)


# ── ROS2 node that publishes coordination_mode ──────────────────────

class CoordModePublisher(Node):
    def __init__(self):
        super().__init__('test_coord_mode_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Int32, '/coordination_mode', qos)
        # Also publish right arm pose so the real CoordinationListener can read it
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.right_pose_pub = self.create_publisher(
            Float32MultiArray, '/right/arm_target_pose', cmd_qos)

    def publish_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.pub.publish(msg)

    def publish_right_pose(self, cart_pose):
        msg = Float32MultiArray()
        msg.data = cart_pose.astype(np.float32).tolist()
        self.right_pose_pub.publish(msg)


# ── Main ─────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    ros_node = CoordModePublisher()

    # Spin ROS2 in background
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # ── Init robots ──────────────────────────────────────────────────
    print("Initializing right arm...")
    right_robot = FrankaRight()
    print("Initializing left arm...")
    left_robot = FrankaLeft()

    time.sleep(1.0)

    # ── Read initial poses ───────────────────────────────────────────
    right_init_H = right_robot.get_pose()['position']
    left_init_H = left_robot.get_pose()['position']
    right_init_cart = homo2cart(right_init_H)
    left_init_cart = homo2cart(left_init_H)

    print(f"\nRight arm init pose: {right_init_cart[:3]}")
    print(f"Left  arm init pose: {left_init_cart[:3]}")

    # ── Record offset in R (world) frame ────────────────────────────
    # Convert left pose from L frame to R frame, then compute offset in R frame.
    T_right_R = cart2homo(right_init_cart)
    T_left_L = cart2homo(left_init_cart)
    T_left_R = T_R2L @ T_left_L  # T_R2L is self-inverse, so also serves as T_L2R
    offset = np.linalg.inv(T_right_R) @ T_left_R
    print(f"\nRecorded offset in world frame (translation part): {offset[:3, 3]}")

    # ── Publish coordination_mode = 6 ───────────────────────────────
    print("\nForcing coordination_mode = 6 (TightlySymmetric)...")
    for _ in range(10):
        ros_node.publish_mode(6)
        time.sleep(0.05)

    # ── Define test trajectory for right arm ─────────────────────────
    # Small sinusoidal motion along X axis (forward/back), ±3cm over 6 seconds
    duration = 6.0
    hz = 30.0
    n_steps = int(duration * hz)

    print(f"\nStarting test: right arm moves ±3cm in X for {duration}s")
    print("Left arm should follow with fixed offset.\n")
    print(f"{'step':>5} | {'R_x':>8} {'R_y':>8} {'R_z':>8} | {'L_x':>8} {'L_y':>8} {'L_z':>8} | {'err_x':>7} {'err_y':>7} {'err_z':>7}")
    print("-" * 95)

    try:
        for i in range(n_steps):
            t_start = time.time()

            # Sinusoidal offset in X
            phase = 2 * np.pi * i / n_steps
            dx = 0.03 * np.sin(phase)

            # Right arm target = initial + delta
            right_target = copy(right_init_cart)
            right_target[0] += dx

            # Send right arm command
            right_robot.arm_control(right_target)

            # Publish right arm pose (so a real CoordinationListener would pick it up)
            ros_node.publish_right_pose(right_target)
            ros_node.publish_mode(6)

            # Compute left arm follower target
            left_target = compute_follower_pose(right_target, offset)

            # Send left arm command
            left_robot.arm_control(left_target)

            # Read actual poses for logging
            if i % 10 == 0:
                right_actual = homo2cart(right_robot.get_pose()['position'])
                left_actual = homo2cart(left_robot.get_pose()['position'])
                # Expected left position
                left_expected = compute_follower_pose(right_actual, offset)
                err = left_actual[:3] - left_expected[:3]
                print(f"{i:5d} | {right_actual[0]:8.4f} {right_actual[1]:8.4f} {right_actual[2]:8.4f} | "
                      f"{left_actual[0]:8.4f} {left_actual[1]:8.4f} {left_actual[2]:8.4f} | "
                      f"{err[0]:7.4f} {err[1]:7.4f} {err[2]:7.4f}")

            # Rate control
            elapsed = time.time() - t_start
            sleep_time = 1.0 / hz - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    # ── Stop: publish mode = 0 ───────────────────────────────────────
    print("\nStopping: setting coordination_mode = 0")
    for _ in range(10):
        ros_node.publish_mode(0)
        time.sleep(0.05)

    # Print final poses
    right_final = homo2cart(right_robot.get_pose()['position'])
    left_final = homo2cart(left_robot.get_pose()['position'])
    print(f"\nRight arm final: {right_final[:3]}")
    print(f"Left  arm final: {left_final[:3]}")

    ros_node.destroy_node()
    rclpy.shutdown()
    print("Done.")


if __name__ == '__main__':
    main()
