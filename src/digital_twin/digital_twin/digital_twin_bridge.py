#!/usr/bin/env python3
"""
ElectroSpin Digital Twin Bridge Node
======================================
Python-based bridge that simulates Gazebo plugin behavior for the
digital twin models when native C++ plugins are not available.

Simulates physics for:
  - Peristaltic pump (roller rotation, flow rate, pressure)
  - Collector drum (motor dynamics, RPM, vibration)
  - HV power supply (voltage ramp, current, status)
  - Environmental sensors (temperature, humidity with noise)
  - Fiber deposition (coverage map based on needle position + flow)

All topics match what the C++ Gazebo plugins would publish, so this
node can be used standalone or alongside Gazebo.

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
import time
import json
import threading
from collections import deque
from typing import Optional

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────────────────────
# Peristaltic Pump Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class PeristalticPumpTwin:
    """Physics simulation of a peristaltic pump with roller animation."""

    def __init__(self):
        self.running = False
        self.target_flow_ml_hr = 0.0
        self.actual_flow_ml_hr = 0.0
        self.pressure_kpa = 0.0
        self.roller_angle = 0.0
        self.roller_rpm = 0.0
        self.volume_remaining_ml = 20.0
        self._response_tau = 2.0
        self._lock = threading.Lock()

    def start(self):
        self.running = True

    def stop(self):
        self.running = False
        self.target_flow_ml_hr = 0.0

    def set_flow(self, ml_hr: float):
        with self._lock:
            self.target_flow_ml_hr = max(0.0, ml_hr)
            if ml_hr > 0:
                self.running = True

    def update(self, dt: float):
        with self._lock:
            if not self.running:
                self.actual_flow_ml_hr *= (1.0 - dt * 2.0)
                if self.actual_flow_ml_hr < 0.001:
                    self.actual_flow_ml_hr = 0.0
            else:
                error = self.target_flow_ml_hr - self.actual_flow_ml_hr
                self.actual_flow_ml_hr += error * (dt / self._response_tau)
                self.actual_flow_ml_hr = max(0.0, self.actual_flow_ml_hr)

            # Roller RPM proportional to flow
            self.roller_rpm = self.actual_flow_ml_hr * 6.0  # 6 RPM per mL/hr
            self.roller_angle += (self.roller_rpm / 60.0) * 2.0 * math.pi * dt

            # Pressure model
            target_pressure = self.actual_flow_ml_hr * 0.8
            self.pressure_kpa += (target_pressure - self.pressure_kpa) * 0.1
            self.pressure_kpa += np.random.normal(0, 0.05)

            # Volume depletion
            ml_dispensed = self.actual_flow_ml_hr * dt / 3600.0
            self.volume_remaining_ml = max(0.0, self.volume_remaining_ml - ml_dispensed)


# ─────────────────────────────────────────────────────────────────────────────
# Collector Drum Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class CollectorDrumTwin:
    """Motor dynamics simulation for the collector drum."""

    def __init__(self):
        self.running = False
        self.target_rpm = 0.0
        self.actual_rpm = 0.0
        self.drum_angle = 0.0
        self.vibration_score = 0.0
        self.temperature_c = 25.0
        self.duty_cycle = 0.0
        self._motor_inertia = 0.15
        self._rpm_history = deque(maxlen=50)
        self._lock = threading.Lock()

    def start(self):
        self.running = True

    def stop(self):
        self.running = False
        self.target_rpm = 0.0

    def set_rpm(self, rpm: float):
        with self._lock:
            self.target_rpm = max(0.0, rpm)
            if rpm > 0:
                self.running = True

    def update(self, dt: float):
        with self._lock:
            if not self.running:
                self.actual_rpm *= (1.0 - dt * 3.0)
                if self.actual_rpm < 0.5:
                    self.actual_rpm = 0.0
            else:
                error = self.target_rpm - self.actual_rpm
                self.actual_rpm += error * (dt / self._motor_inertia)
                self.actual_rpm = max(0.0, self.actual_rpm)

            # Drum rotation
            self.drum_angle += (self.actual_rpm / 60.0) * 2.0 * math.pi * dt

            # Vibration from RPM variance
            self._rpm_history.append(self.actual_rpm + np.random.normal(0, 2.0))
            if len(self._rpm_history) > 5:
                self.vibration_score = float(np.clip(np.std(self._rpm_history) / 50.0, 0, 1))

            # Temperature model
            target_temp = 25.0 + self.duty_cycle * 20.0
            self.temperature_c += (target_temp - self.temperature_c) * 0.01

            # Duty cycle
            self.duty_cycle = self.actual_rpm / 3000.0


# ─────────────────────────────────────────────────────────────────────────────
# HV Power Supply Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class HVSupplyTwin:
    """High voltage power supply simulation."""

    def __init__(self):
        self.enabled = False
        self.target_voltage_kv = 0.0
        self.actual_voltage_kv = 0.0
        self.current_ua = 0.0
        self._ramp_rate = 2.0  # kV/s

    def enable(self, on: bool):
        self.enabled = on
        if not on:
            self.target_voltage_kv = 0.0

    def set_voltage(self, kv: float):
        self.target_voltage_kv = max(0.0, min(30.0, kv))

    def update(self, dt: float):
        if not self.enabled:
            self.actual_voltage_kv *= (1.0 - dt * 5.0)
            if self.actual_voltage_kv < 0.01:
                self.actual_voltage_kv = 0.0
        else:
            error = self.target_voltage_kv - self.actual_voltage_kv
            max_step = self._ramp_rate * dt
            step = max(-max_step, min(max_step, error))
            self.actual_voltage_kv += step

        # Current model (leakage + load)
        self.current_ua = self.actual_voltage_kv * 0.5 + np.random.normal(0, 0.1)


# ─────────────────────────────────────────────────────────────────────────────
# Environmental Sensor Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class EnvSensorTwin:
    """Temperature and humidity sensor with realistic drift."""

    def __init__(self, base_temp=22.0, base_humidity=45.0):
        self.base_temp = base_temp
        self.base_humidity = base_humidity
        self.temperature = base_temp
        self.humidity = base_humidity
        self._temp_drift = 0.0
        self._humid_drift = 0.0

    def update(self, dt: float, hv_active=False, pump_flow=0.0):
        # Slow drift
        self._temp_drift += np.random.normal(0, 0.01)
        self._temp_drift *= 0.99

        # HV heating effect
        hv_heat = 0.5 if hv_active else 0.0
        # Pump motor heat
        pump_heat = pump_flow * 0.02

        target_temp = self.base_temp + self._temp_drift + hv_heat + pump_heat
        self.temperature += (target_temp - self.temperature) * 0.05

        # Humidity
        self._humid_drift += np.random.normal(0, 0.05)
        self._humid_drift *= 0.98
        target_humid = self.base_humidity + self._humid_drift
        self.humidity += (target_humid - self.humidity) * 0.03
        self.humidity = max(10.0, min(90.0, self.humidity))


# ─────────────────────────────────────────────────────────────────────────────
# Fiber Deposition Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class FiberDepositionTwin:
    """Simulates nanofiber deposition on the collector surface."""

    def __init__(self, num_zones=64):
        self.num_zones = num_zones
        self.coverage_map = np.zeros(num_zones)
        self.deposition_rate = 0.0
        self.total_deposited_mg = 0.0

    def update(self, dt: float, collector_rpm: float, flow_ml_hr: float,
               needle_x_norm: float = 0.5):
        if collector_rpm < 1.0 or flow_ml_hr < 0.01:
            return

        # Deposition rate proportional to flow and inversely to RPM
        self.deposition_rate = flow_ml_hr * 0.5 / (collector_rpm / 500.0 + 1.0)

        # Deposit at needle position
        zone = int(needle_x_norm * (self.num_zones - 1))
        zone = max(0, min(self.num_zones - 1, zone))

        # Gaussian spread around needle position
        for i in range(self.num_zones):
            dist = abs(i - zone)
            weight = math.exp(-dist * dist / 8.0)
            self.coverage_map[i] = min(1.0, self.coverage_map[i] + weight * self.deposition_rate * dt * 0.01)

        self.total_deposited_mg += self.deposition_rate * dt * 0.1


# ─────────────────────────────────────────────────────────────────────────────
# Digital Twin Bridge Node
# ─────────────────────────────────────────────────────────────────────────────

class DigitalTwinBridgeNode(Node):
    """
    ROS2 node that bridges digital twin physics simulation with ROS2 topics.

    Simulates all Gazebo plugin behavior and publishes to the same topics
    that the C++ plugins would use.
    """

    def __init__(self):
        super().__init__("digital_twin_bridge")

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("update_rate_hz", 50.0)

        sim = self.get_parameter("simulation_mode").value
        rate = self.get_parameter("update_rate_hz").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # ── Digital Twin Models ──────────────────────────────────────────────
        self.pump = PeristalticPumpTwin()
        self.collector = CollectorDrumTwin()
        self.hv = HVSupplyTwin()
        self.env = EnvSensorTwin()
        self.deposition = FiberDepositionTwin()

        # ── Publishers (Pump) ────────────────────────────────────────────────
        self.pub_pump_flow = self.create_publisher(Float32, "/pump/flow_rate", qos)
        self.pub_pump_pressure = self.create_publisher(Float32, "/pump/pressure", qos)
        self.pub_pump_status = self.create_publisher(String, "/pump/status", qos)

        # ── Publishers (Collector) ────────────────────────────────────────────
        self.pub_collector_rpm = self.create_publisher(Float32, "/collector/rpm", qos)
        self.pub_collector_state = self.create_publisher(String, "/collector/state", qos)

        # ── Publishers (HV) ──────────────────────────────────────────────────
        self.pub_hv_voltage = self.create_publisher(Float32, "/hv/voltage", qos)
        self.pub_hv_status = self.create_publisher(String, "/hv/status", qos)

        # ── Publishers (Environment) ──────────────────────────────────────────
        self.pub_env_temp = self.create_publisher(Float32, "/env/temperature", qos)
        self.pub_env_humidity = self.create_publisher(Float32, "/env/humidity", qos)

        # ── Publishers (Deposition) ──────────────────────────────────────────
        self.pub_deposition = self.create_publisher(String, "/fiber_deposition", qos)

        # ── Publishers (Joint States for Gazebo models) ──────────────────────
        self.pub_joint_states = self.create_publisher(JointState, "/digital_twin/joint_states", sensor_qos)

        # ── Subscribers (Commands) ───────────────────────────────────────────
        self.sub_pump_start = self.create_subscription(
            Bool, "/pump/start", self._on_pump_start, qos
        )
        self.sub_pump_flow_cmd = self.create_subscription(
            Float32, "/target_flowrate", self._on_pump_flow_cmd, qos
        )
        self.sub_collector_start = self.create_subscription(
            Bool, "/collector/start", self._on_collector_start, qos
        )
        self.sub_collector_rpm_cmd = self.create_subscription(
            Float32, "/target_rpm", self._on_collector_rpm_cmd, qos
        )
        self.sub_hv_enable = self.create_subscription(
            Bool, "/hv/enable", self._on_hv_enable, qos
        )
        self.sub_hv_voltage_cmd = self.create_subscription(
            Float32, "/hv/voltage_cmd", self._on_hv_voltage_cmd, qos
        )

        # ── Simulation Timer ─────────────────────────────────────────────────
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._update_loop)

        self.get_logger().info(
            f"[DigitalTwinBridge] Initialized. Rate={rate}Hz, Sim={sim}"
        )

    # ── Command Callbacks ────────────────────────────────────────────────────

    def _on_pump_start(self, msg: Bool):
        if msg.data:
            self.pump.start()
        else:
            self.pump.stop()

    def _on_pump_flow_cmd(self, msg: Float32):
        self.pump.set_flow(msg.data)

    def _on_collector_start(self, msg: Bool):
        if msg.data:
            self.collector.start()
        else:
            self.collector.stop()

    def _on_collector_rpm_cmd(self, msg: Float32):
        self.collector.set_rpm(msg.data)

    def _on_hv_enable(self, msg: Bool):
        self.hv.enable(msg.data)

    def _on_hv_voltage_cmd(self, msg: Float32):
        self.hv.set_voltage(msg.data)

    # ── Main Update Loop ─────────────────────────────────────────────────────

    def _update_loop(self):
        dt = self._dt

        # Update physics
        self.pump.update(dt)
        self.collector.update(dt)
        self.hv.update(dt)
        self.env.update(dt, hv_active=self.hv.enabled, pump_flow=self.pump.actual_flow_ml_hr)
        self.deposition.update(dt, self.collector.actual_rpm, self.pump.actual_flow_ml_hr)

        # ── Publish Pump ─────────────────────────────────────────────────────
        flow_msg = Float32()
        flow_msg.data = float(self.pump.actual_flow_ml_hr)
        self.pub_pump_flow.publish(flow_msg)

        press_msg = Float32()
        press_msg.data = float(self.pump.pressure_kpa)
        self.pub_pump_pressure.publish(press_msg)

        pump_status = String()
        pump_status.data = json.dumps({
            "actual_flow_ml_hr": round(self.pump.actual_flow_ml_hr, 3),
            "setpoint_ml_hr": round(self.pump.target_flow_ml_hr, 3),
            "pressure_kpa": round(self.pump.pressure_kpa, 2),
            "volume_remaining_ml": round(self.pump.volume_remaining_ml, 2),
            "roller_rpm": round(self.pump.roller_rpm, 1),
            "running": self.pump.running,
        })
        self.pub_pump_status.publish(pump_status)

        # ── Publish Collector ───────────────────────────────────────────────
        rpm_msg = Float32()
        rpm_msg.data = float(self.collector.actual_rpm)
        self.pub_collector_rpm.publish(rpm_msg)

        coll_state = String()
        coll_state.data = json.dumps({
            "rpm": round(self.collector.actual_rpm, 1),
            "target_rpm": round(self.collector.target_rpm, 1),
            "vibration": round(self.collector.vibration_score, 3),
            "temperature_c": round(self.collector.temperature_c, 1),
            "duty_cycle": round(self.collector.duty_cycle, 3),
            "running": self.collector.running,
        })
        self.pub_collector_state.publish(coll_state)

        # ── Publish HV ──────────────────────────────────────────────────────
        volt_msg = Float32()
        volt_msg.data = float(self.hv.actual_voltage_kv)
        self.pub_hv_voltage.publish(volt_msg)

        hv_status = String()
        hv_status.data = json.dumps({
            "voltage_kv": round(self.hv.actual_voltage_kv, 2),
            "target_kv": round(self.hv.target_voltage_kv, 2),
            "current_ua": round(self.hv.current_ua, 2),
            "enabled": self.hv.enabled,
        })
        self.pub_hv_status.publish(hv_status)

        # ── Publish Environment ──────────────────────────────────────────────
        temp_msg = Float32()
        temp_msg.data = float(self.env.temperature)
        self.pub_env_temp.publish(temp_msg)

        humid_msg = Float32()
        humid_msg.data = float(self.env.humidity)
        self.pub_env_humidity.publish(humid_msg)

        # ── Publish Deposition ──────────────────────────────────────────────
        dep_msg = String()
        dep_msg.data = json.dumps({
            "coverage_mean": round(float(self.deposition.coverage_map.mean()), 3),
            "coverage_max": round(float(self.deposition.coverage_map.max()), 3),
            "deposition_rate": round(self.deposition.deposition_rate, 3),
            "total_mg": round(self.deposition.total_deposited_mg, 2),
        })
        self.pub_deposition.publish(dep_msg)

        # ── Publish Joint States for Gazebo model animation ──────────────────
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            "pump_roller_joint",
            "drum_rotation_joint",
        ]
        js.position = [
            self.pump.roller_angle,
            self.collector.drum_angle,
        ]
        js.velocity = [
            self.pump.roller_rpm / 60.0 * 2.0 * math.pi,
            self.collector.actual_rpm / 60.0 * 2.0 * math.pi,
        ]
        self.pub_joint_states.publish(js)


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
