"""
Continuously publish coordination_mode = 6 (TightlySymmetric) to /coordination_mode.

Usage:
    python publish_symmetric_mode.py

Then in another terminal:
    python teleop.py robot=both
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32

COORD_TIGHTLY_SYMMETRIC = 6


class SymmetricModePublisher(Node):
    def __init__(self):
        super().__init__('symmetric_mode_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Int32, '/coordination_mode', qos)
        self.timer = self.create_timer(0.1, self._publish)  # 10 Hz
        self.get_logger().info('Publishing coordination_mode=6 (TightlySymmetric) at 10Hz. Ctrl+C to stop.')

    def _publish(self):
        msg = Int32()
        msg.data = COORD_TIGHTLY_SYMMETRIC
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SymmetricModePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
