#!/usr/bin/env python3
"""
ElectroSpin Command Bridge Node
=================================
Decomposes the compound ElectrospinCommand message into individual
Float32 setpoint messages for the collector and syringe pump controllers.

The AI controller publishes a single ElectrospinCommand containing
target_rpm, target_flowrate, target_voltage, etc. But the collector
and pump controllers only subscribe to /target_rpm and /target_flowrate
as individual Float32 topics. This bridge subscribes to ElectrospinCommand
and republishes the individual fields.

Topics Subscribed:
  /electrospin_command   → ElectrospinCommand

Topics Published:
  /target_rpm            → Float32
  /target_flowrate       → Float32
  /target_distance       → Float32

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Bool
from electrospin_interfaces.msg import ElectrospinCommand


class CommandBridgeNode(Node):
    """Bridges compound ElectrospinCommand to individual Float32 setpoints."""

    def __init__(self):
        super().__init__("command_bridge")

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("enable_collector_control", True)
        self.declare_parameter("enable_pump_control", True)
        raw_enable_collector = self.get_parameter("enable_collector_control").value
        if isinstance(raw_enable_collector, str):
            self.enable_collector_control = raw_enable_collector.strip().lower() in {
                "1", "true", "yes", "on"
            }
        else:
            self.enable_collector_control = bool(raw_enable_collector)
        raw_enable_pump = self.get_parameter("enable_pump_control").value
        if isinstance(raw_enable_pump, str):
            self.enable_pump_control = raw_enable_pump.strip().lower() in {
                "1", "true", "yes", "on"
            }
        else:
            self.enable_pump_control = bool(raw_enable_pump)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # Publishers
        self.pub_target_rpm = self.create_publisher(Float32, "/target_rpm", reliable_qos)
        self.pub_target_flow = self.create_publisher(Float32, "/target_flowrate", reliable_qos)
        self.pub_target_distance = self.create_publisher(Float32, "/target_distance", reliable_qos)
        self.pub_collector_enable = self.create_publisher(Bool, "/collector_enable", reliable_qos)
        self.pub_pump_enable = self.create_publisher(Bool, "/pump_enable", reliable_qos)

        # Subscriber
        self.sub_command = self.create_subscription(
            ElectrospinCommand, "/electrospin_command",
            self._on_command, reliable_qos
        )

        self.get_logger().info("[CommandBridge] Initialized — bridging ElectrospinCommand to individual setpoints")

    def _on_command(self, msg: ElectrospinCommand):
        # RPM
        if self.enable_collector_control and msg.target_rpm >= 0:
            rpm_msg = Float32()
            rpm_msg.data = float(msg.target_rpm)
            self.pub_target_rpm.publish(rpm_msg)

        # Flow rate
        if self.enable_pump_control and msg.target_flowrate >= 0:
            flow_msg = Float32()
            flow_msg.data = float(msg.target_flowrate)
            self.pub_target_flow.publish(flow_msg)

        # Distance
        if msg.target_distance > 0:
            dist_msg = Float32()
            dist_msg.data = float(msg.target_distance)
            self.pub_target_distance.publish(dist_msg)

        # Enable signals
        if self.enable_collector_control:
            ce = Bool()
            ce.data = msg.collector_enable
            self.pub_collector_enable.publish(ce)

        if self.enable_pump_control:
            pe = Bool()
            pe.data = msg.pump_enable
            self.pub_pump_enable.publish(pe)


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
