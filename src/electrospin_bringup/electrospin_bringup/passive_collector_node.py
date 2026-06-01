#!/usr/bin/env python3
"""
Passive fixed collector state publisher.

Publishes a stable CollectorStatus for setups where the collector is
physically present but not motorized or not connected to a programmable driver.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from electrospin_interfaces.msg import CollectorStatus


class PassiveCollectorNode(Node):
    """Publish a fixed collector state for passive hardware setups."""

    def __init__(self):
        super().__init__("passive_collector")

        self.declare_parameter("fixed_rpm", 0.0)
        self.declare_parameter("temperature_c", 25.0)
        self.declare_parameter("publish_frequency_hz", 5.0)

        self.fixed_rpm = float(self.get_parameter("fixed_rpm").value)
        self.temperature_c = float(self.get_parameter("temperature_c").value)
        pub_freq = float(self.get_parameter("publish_frequency_hz").value)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_status = self.create_publisher(
            CollectorStatus, "/collector_status", reliable_qos
        )
        self.pub_rpm = self.create_publisher(
            Float32, "/collector_rpm", sensor_qos
        )

        self.create_timer(1.0 / max(pub_freq, 0.5), self._publish_status)

        self.get_logger().info(
            f"[PassiveCollector] Initialized. FixedRPM={self.fixed_rpm}, Temp={self.temperature_c}C"
        )

    def _publish_status(self):
        status = CollectorStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.rpm = self.fixed_rpm
        status.target_rpm = self.fixed_rpm
        status.torque = 0.0
        status.running = self.fixed_rpm > 1.0
        status.at_setpoint = True
        status.vibration_score = 0.0
        status.temperature_c = self.temperature_c
        status.current_a = 0.0
        status.duty_cycle = 0.0
        status.emergency_stop = False
        status.ramp_active = False
        status.encoder_position = 0.0 % (2.0 * math.pi)
        status.control_mode = 0  # IDLE/passive
        self.pub_status.publish(status)

        rpm_msg = Float32()
        rpm_msg.data = self.fixed_rpm
        self.pub_rpm.publish(rpm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassiveCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
