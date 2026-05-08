#!/usr/bin/env python3
"""
ElectroSpin System Monitor Node
=================================
Watchdog and system health aggregator for the electrospinning platform.

Responsibilities:
  - Monitor all node heartbeats
  - Aggregate system-wide status
  - Publish SystemStatus messages
  - Detect node failures and timeouts
  - Manage global emergency stop
  - Track system uptime and health

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import json
import threading
from typing import Dict, Optional
from enum import IntEnum

from std_msgs.msg import String, Bool
from electrospin_interfaces.msg import (
    FiberQuality, CollectorStatus, SystemStatus
)


class SystemState(IntEnum):
    OFF     = 0
    INIT    = 1
    READY   = 2
    RUNNING = 3
    ERROR   = 4
    ESTOP   = 5


class SystemMonitorNode(Node):
    """
    ROS2 system monitor and watchdog node.

    Topics Published:
        /system_status    → electrospin_interfaces/SystemStatus
        /emergency_stop   → std_msgs/Bool

    Topics Subscribed:
        /fiber_quality     → electrospin_interfaces/FiberQuality
        /collector_status  → electrospin_interfaces/CollectorStatus
        /robot_status      → std_msgs/String
        /ai_status         → std_msgs/String
        /pump_status       → std_msgs/String
    """

    NODE_TIMEOUT_S = 5.0

    def __init__(self):
        super().__init__("system_monitor")

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("watchdog_timeout_s", 5.0)

        self.sim_mode = self.get_parameter("simulation_mode").value
        self.timeout = self.get_parameter("watchdog_timeout_s").value

        # State
        self.system_state = SystemState.INIT
        self.start_time = time.time()
        self.global_estop = False

        # Node heartbeat tracking
        self._heartbeats: Dict[str, float] = {
            "robot": 0.0,
            "collector": 0.0,
            "pump": 0.0,
            "vision": 0.0,
            "ai": 0.0,
        }

        # Latest data cache
        self._latest_quality: Optional[float] = None
        self._quality_history = []
        self._quality_avg = 0.0
        self._quality_grade = 0

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # Publishers
        self.pub_status = self.create_publisher(
            SystemStatus, "/system_status", reliable_qos
        )
        self.pub_estop = self.create_publisher(
            Bool, "/emergency_stop", reliable_qos
        )

        # Subscribers
        self.sub_quality = self.create_subscription(
            FiberQuality, "/fiber_quality", self._on_quality, sensor_qos
        )
        self.sub_collector = self.create_subscription(
            CollectorStatus, "/collector_status", self._on_collector, reliable_qos
        )
        self.sub_robot = self.create_subscription(
            String, "/robot_status", self._on_robot, reliable_qos
        )
        self.sub_ai = self.create_subscription(
            String, "/ai_status", self._on_ai, reliable_qos
        )
        self.sub_pump = self.create_subscription(
            String, "/pump_status", self._on_pump, reliable_qos
        )
        self.sub_estop = self.create_subscription(
            Bool, "/emergency_stop", self._on_estop_input, reliable_qos
        )

        # Timers
        self.monitor_timer = self.create_timer(1.0, self._monitor_cycle)
        self.status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f"[SystemMonitor] Initialized. Sim={self.sim_mode}, Timeout={self.timeout}s"
        )

    # ── Subscriber Callbacks ──────────────────────────────────────────────────

    def _on_quality(self, msg: FiberQuality):
        self._heartbeats["vision"] = time.time()
        self._latest_quality = msg.overall_quality
        self._quality_grade = msg.quality_grade
        self._quality_history.append(msg.overall_quality)
        if len(self._quality_history) > 100:
            self._quality_history = self._quality_history[-100:]
        self._quality_avg = sum(self._quality_history) / len(self._quality_history)

    def _on_collector(self, msg: CollectorStatus):
        self._heartbeats["collector"] = time.time()
        if msg.emergency_stop and not self.global_estop:
            self._trigger_estop("Collector E-STOP detected")

    def _on_robot(self, msg: String):
        self._heartbeats["robot"] = time.time()

    def _on_ai(self, msg: String):
        self._heartbeats["ai"] = time.time()

    def _on_pump(self, msg: String):
        self._heartbeats["pump"] = time.time()

    def _on_estop_input(self, msg: Bool):
        if msg.data and not self.global_estop:
            self._trigger_estop("External E-STOP signal")
        elif not msg.data and self.global_estop:
            self._clear_estop()

    # ── Monitor Cycle ────────────────────────────────────────────────────────

    def _monitor_cycle(self):
        now = time.time()

        # Check node timeouts
        dead_nodes = []
        for name, last_beat in self._heartbeats.items():
            if now - last_beat > self.timeout and last_beat > 0:
                dead_nodes.append(name)

        if dead_nodes:
            self.get_logger().warn(
                f"[SystemMonitor] Node timeout: {', '.join(dead_nodes)}"
            )

        # Update system state
        if self.global_estop:
            self.system_state = SystemState.ESTOP
        elif dead_nodes:
            self.system_state = SystemState.ERROR
        elif all(t > 0 for t in self._heartbeats.values()):
            if self._latest_quality is not None:
                self.system_state = SystemState.RUNNING
            else:
                self.system_state = SystemState.READY
        else:
            self.system_state = SystemState.INIT

    def _trigger_estop(self, reason: str):
        self.global_estop = True
        self.system_state = SystemState.ESTOP
        msg = Bool()
        msg.data = True
        self.pub_estop.publish(msg)
        self.get_logger().error(f"[SystemMonitor] E-STOP triggered: {reason}")

    def _clear_estop(self):
        self.global_estop = False
        self.system_state = SystemState.READY
        self.get_logger().info("[SystemMonitor] E-STOP cleared")

    # ── Status Publisher ─────────────────────────────────────────────────────

    def _publish_status(self):
        now = time.time()
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()

        status.system_state = int(self.system_state)
        status.simulation_mode = self.sim_mode
        status.uptime_s = float(now - self.start_time)

        # Node alive bitmask: bit0=robot, bit1=collector, bit2=pump, bit3=vision, bit4=ai
        mask = 0
        node_order = ["robot", "collector", "pump", "vision", "ai"]
        for i, name in enumerate(node_order):
            if now - self._heartbeats[name] < self.timeout:
                mask |= (1 << i)
        status.nodes_alive_mask = mask

        status.emergency_stop = self.global_estop
        status.safety_interlock = not self.global_estop
        status.temperature_max_c = 25.0  # Placeholder

        status.quality_current = float(self._latest_quality or 0.0)
        status.quality_average = float(self._quality_avg)
        status.quality_grade = self._quality_grade

        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
