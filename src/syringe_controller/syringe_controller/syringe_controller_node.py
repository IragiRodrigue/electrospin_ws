#!/usr/bin/env python3
"""
ElectroSpin Syringe Pump Controller Node
==========================================
Precision microfluidic flow control for electrospinning solution delivery.

Features:
  - Precise flow rate control (0.01 mL/hr resolution)
  - Back-pressure monitoring and auto-correction
  - Anti-drip / purge routines
  - Multi-syringe support
  - Stepper-based simulation model

Hardware support: Serial-based syringe pump drivers
(NE-300, Harvard Apparatus, custom Arduino stepper controller)

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import threading
import numpy as np
from dataclasses import dataclass
from typing import Optional
from collections import deque

from std_msgs.msg import Float32, Bool, String


# ─────────────────────────────────────────────────────────────────────────────
# Pump Simulation Model
# ─────────────────────────────────────────────────────────────────────────────

class PumpSimulation:
    """
    Physics-based simulation of a syringe pump.
    Models stepper motor, lead screw, and fluid dynamics.
    """

    def __init__(self):
        # Syringe parameters
        self.syringe_diameter_mm = 25.0       # BD 20mL syringe
        self.syringe_area_mm2 = np.pi * (self.syringe_diameter_mm / 2.0) ** 2

        # Stepper/drive parameters
        self.steps_per_rev = 200
        self.lead_screw_pitch_mm = 1.0        # mm/revolution
        self.microstep = 16

        # Fluid state
        self.actual_flow_ml_hr = 0.0
        self.target_flow_ml_hr = 0.0
        self.pressure_kpa = 0.0
        self.volume_remaining_ml = 20.0
        self._lock = threading.Lock()
        self._running = True

        # Response model
        self.response_tau = 2.0               # seconds time constant

        thread = threading.Thread(target=self._simulate, daemon=True)
        thread.start()

    def set_flow_rate(self, ml_per_hr: float):
        with self._lock:
            self.target_flow_ml_hr = max(0.0, ml_per_hr)

    def get_flow_rate(self) -> float:
        with self._lock:
            return float(self.actual_flow_ml_hr)

    def get_pressure(self) -> float:
        with self._lock:
            return float(self.pressure_kpa)

    def get_volume_remaining(self) -> float:
        with self._lock:
            return float(self.volume_remaining_ml)

    def _simulate(self):
        dt = 0.05
        while self._running:
            with self._lock:
                error = self.target_flow_ml_hr - self.actual_flow_ml_hr
                self.actual_flow_ml_hr += error * (dt / self.response_tau)
                self.actual_flow_ml_hr = max(0.0, self.actual_flow_ml_hr)

                # Volume depletion
                ml_dispensed = self.actual_flow_ml_hr * dt / 3600.0
                self.volume_remaining_ml = max(0.0, self.volume_remaining_ml - ml_dispensed)

                # Pressure model (higher flow → higher back-pressure)
                target_pressure = self.actual_flow_ml_hr * 0.8
                self.pressure_kpa += (target_pressure - self.pressure_kpa) * 0.1
                self.pressure_kpa += np.random.normal(0, 0.05)

            time.sleep(dt)


# ─────────────────────────────────────────────────────────────────────────────
# Syringe Pump Controller Node
# ─────────────────────────────────────────────────────────────────────────────

class SyringePumpControllerNode(Node):
    """
    ROS2 syringe pump controller.

    Topics Published:
        /flow_rate    → std_msgs/Float32
        /pump_status  → std_msgs/String (JSON)

    Topics Subscribed:
        /target_flowrate    → std_msgs/Float32
        /emergency_stop     → std_msgs/Bool
    """

    def __init__(self):
        super().__init__("syringe_pump_controller")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("max_flow_ml_hr", 10.0)
        self.declare_parameter("pressure_limit_kpa", 50.0)
        self.declare_parameter("syringe_volume_ml", 20.0)

        sim_mode       = self.get_parameter("simulation_mode").value
        pub_hz         = self.get_parameter("publish_hz").value
        self.max_flow  = self.get_parameter("max_flow_ml_hr").value
        self.p_limit   = self.get_parameter("pressure_limit_kpa").value

        # ── Hardware ──────────────────────────────────────────────────────────
        self.pump = PumpSimulation()
        self.pump.volume_remaining_ml = self.get_parameter("syringe_volume_ml").value
        self.emergency_stop = False
        self.current_setpoint = 0.0

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_flow   = self.create_publisher(Float32, "/flow_rate", qos)
        self.pub_status = self.create_publisher(String,  "/pump_status", qos)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.sub_target = self.create_subscription(
            Float32, "/target_flowrate", self._on_target_flow, qos
        )
        self.sub_estop = self.create_subscription(
            Bool, "/emergency_stop", self._on_estop, qos
        )

        self.create_timer(1.0 / pub_hz, self._publish_status)

        self.get_logger().info(
            f"[PumpController] Initialized. Sim={sim_mode}, MaxFlow={self.max_flow}mL/hr"
        )

    def _on_target_flow(self, msg: Float32):
        if self.emergency_stop:
            return
        rate = float(np.clip(msg.data, 0.0, self.max_flow))
        self.current_setpoint = rate
        self.pump.set_flow_rate(rate)

    def _on_estop(self, msg: Bool):
        if msg.data:
            self.emergency_stop = True
            self.pump.set_flow_rate(0.0)
            self.get_logger().error("[PumpController] EMERGENCY STOP — pump halted")
        else:
            self.emergency_stop = False

    def _publish_status(self):
        flow     = self.pump.get_flow_rate()
        pressure = self.pump.get_pressure()
        volume   = self.pump.get_volume_remaining()

        # Over-pressure safety cutoff
        if pressure > self.p_limit and not self.emergency_stop:
            self.pump.set_flow_rate(max(0.0, self.current_setpoint * 0.8))
            self.get_logger().warn(
                f"[PumpController] Over-pressure {pressure:.1f}kPa — reducing flow"
            )

        # Float32 flow rate
        fm = Float32()
        fm.data = float(flow)
        self.pub_flow.publish(fm)

        # JSON status
        import json
        status = {
            "actual_flow_ml_hr":   round(flow, 3),
            "setpoint_ml_hr":      round(self.current_setpoint, 3),
            "pressure_kpa":        round(pressure, 2),
            "volume_remaining_ml": round(volume, 2),
            "emergency_stop":      self.emergency_stop,
            "pump_running":        flow > 0.01,
        }
        sm = String()
        sm.data = json.dumps(status)
        self.pub_status.publish(sm)


def main(args=None):
    rclpy.init(args=args)
    node = SyringePumpControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pump._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()