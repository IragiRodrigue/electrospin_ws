#!/usr/bin/env python3
"""
ElectroSpin Digital Twin Bridge Node
======================================
Complete digital twin simulation with physics, camera, predictive maintenance,
and real-time recording to Supabase.

Simulates:
  - Peristaltic pump (roller rotation, flow rate, pressure, wear)
  - Collector drum (motor dynamics, RPM, vibration, bearing wear)
  - HV power supply (voltage ramp, current, arc detection)
  - Environmental sensors (temperature, humidity with noise)
  - Fiber deposition (coverage map based on needle position + flow)
  - Camera (synthetic frames responding to process state)
  - Predictive maintenance (component wear, lifetime estimation, alerts)
  - Real-time recording (all metrics persisted to Supabase)

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
import traceback
from collections import deque
from typing import Optional, Dict, List
from dataclasses import dataclass, asdict

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image, JointState

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

try:
    import urllib.request
    SUPABASE_AVAILABLE = True
except ImportError:
    SUPABASE_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Peristaltic Pump Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class PeristalticPumpTwin:
    """Physics simulation of a peristaltic pump with roller animation and wear."""

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
        # Wear tracking
        self.total_run_hours = 0.0
        self.tubing_wear = 0.0       # 0-1, 1 = replace
        self.roller_wear = 0.0       # 0-1
        self.motor_temp_c = 25.0

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
                self.total_run_hours += dt / 3600.0

            self.roller_rpm = self.actual_flow_ml_hr * 6.0
            self.roller_angle += (self.roller_rpm / 60.0) * 2.0 * math.pi * dt

            target_pressure = self.actual_flow_ml_hr * 0.8
            self.pressure_kpa += (target_pressure - self.pressure_kpa) * 0.1
            self.pressure_kpa += np.random.normal(0, 0.05)

            ml_dispensed = self.actual_flow_ml_hr * dt / 3600.0
            self.volume_remaining_ml = max(0.0, self.volume_remaining_ml - ml_dispensed)

            # Wear models
            if self.running:
                self.tubing_wear = min(1.0, self.total_run_hours / 2000.0)
                self.roller_wear = min(1.0, self.total_run_hours / 5000.0)
                self.motor_temp_c = 25.0 + self.actual_flow_ml_hr * 2.0 + np.random.normal(0, 0.3)
            else:
                self.motor_temp_c += (25.0 - self.motor_temp_c) * 0.05


# ─────────────────────────────────────────────────────────────────────────────
# Collector Drum Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class CollectorDrumTwin:
    """Motor dynamics simulation for the collector drum with bearing wear."""

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
        # Wear tracking
        self.total_run_hours = 0.0
        self.bearing_wear = 0.0     # 0-1
        self.belt_wear = 0.0        # 0-1
        self.motor_wear = 0.0       # 0-1

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
                self.total_run_hours += dt / 3600.0

            self.drum_angle += (self.actual_rpm / 60.0) * 2.0 * math.pi * dt

            noise_std = 2.0 + self.bearing_wear * 10.0
            self._rpm_history.append(self.actual_rpm + np.random.normal(0, noise_std))
            if len(self._rpm_history) > 5:
                self.vibration_score = float(np.clip(np.std(self._rpm_history) / 50.0, 0, 1))

            target_temp = 25.0 + self.duty_cycle * 20.0 + self.bearing_wear * 5.0
            self.temperature_c += (target_temp - self.temperature_c) * 0.01

            self.duty_cycle = self.actual_rpm / 3000.0

            # Wear models
            if self.running:
                self.bearing_wear = min(1.0, self.total_run_hours / 3000.0)
                self.belt_wear = min(1.0, self.total_run_hours / 1500.0)
                self.motor_wear = min(1.0, self.total_run_hours / 8000.0)


# ─────────────────────────────────────────────────────────────────────────────
# HV Power Supply Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class HVSupplyTwin:
    """High voltage power supply simulation with arc detection."""

    def __init__(self):
        self.enabled = False
        self.target_voltage_kv = 0.0
        self.actual_voltage_kv = 0.0
        self.current_ua = 0.0
        self._ramp_rate = 2.0
        # Wear
        self.total_run_hours = 0.0
        self.insulation_wear = 0.0
        self.arc_count = 0
        self.last_arc_time = 0.0

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
            self.total_run_hours += dt / 3600.0

        self.current_ua = self.actual_voltage_kv * 0.5 + np.random.normal(0, 0.1)

        # Arc detection (random, more likely with high voltage and worn insulation)
        arc_prob = (self.actual_voltage_kv / 30.0) * (0.001 + self.insulation_wear * 0.01)
        if np.random.random() < arc_prob * dt:
            self.arc_count += 1
            self.last_arc_time = time.time()

        self.insulation_wear = min(1.0, self.total_run_hours / 5000.0)


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

    def update(self, dt: float, hv_active=False, pump_flow=0.0, collector_rpm=0.0):
        self._temp_drift += np.random.normal(0, 0.01)
        self._temp_drift *= 0.99
        hv_heat = 0.5 if hv_active else 0.0
        pump_heat = pump_flow * 0.02
        motor_heat = collector_rpm / 3000.0 * 0.3
        target_temp = self.base_temp + self._temp_drift + hv_heat + pump_heat + motor_heat
        self.temperature += (target_temp - self.temperature) * 0.05

        self._humid_drift += np.random.normal(0, 0.05)
        self._humid_drift *= 0.98
        target_humid = self.base_humidity + self._humid_drift
        if hv_active:
            target_humid -= 0.5
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
        self.deposition_rate = flow_ml_hr * 0.5 / (collector_rpm / 500.0 + 1.0)
        zone = int(needle_x_norm * (self.num_zones - 1))
        zone = max(0, min(self.num_zones - 1, zone))
        for i in range(self.num_zones):
            dist = abs(i - zone)
            weight = math.exp(-dist * dist / 8.0)
            self.coverage_map[i] = min(1.0, self.coverage_map[i] + weight * self.deposition_rate * dt * 0.01)
        self.total_deposited_mg += self.deposition_rate * dt * 0.1


# ─────────────────────────────────────────────────────────────────────────────
# Camera Digital Twin
# ─────────────────────────────────────────────────────────────────────────────

class CameraTwin:
    """
    Generates synthetic camera frames that respond to process state.
    Simulates: Taylor cone, jet, fiber deposition, collector rotation,
    ambient lighting changes, and camera noise.
    """

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self._frame_count = 0
        self._bridge = CvBridge() if CV_AVAILABLE else None

    def generate_frame(self, process_state: dict) -> Optional[object]:
        """Generate a synthetic frame reflecting current process state."""
        if not CV_AVAILABLE:
            return None

        t = time.time()
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = (12, 12, 20)

        hv_on = process_state.get("hv_enabled", False)
        hv_kv = process_state.get("hv_voltage_kv", 0)
        flow = process_state.get("pump_flow_ml_hr", 0)
        rpm = process_state.get("collector_rpm", 0)
        dep_coverage = process_state.get("dep_coverage", 0)
        temp = process_state.get("env_temp", 22.0)

        # Ambient lighting variation
        ambient = int(np.clip(12 + (temp - 22.0) * 0.5, 8, 25))
        frame[:] = (ambient, ambient, ambient + 5)

        # Collector drum
        cy = int(self.height * 0.72)
        drum_color = (35 + int(dep_coverage * 40), 35 + int(dep_coverage * 30), 50)
        cv2.rectangle(frame, (60, cy), (self.width - 60, cy + 70), drum_color, -1)
        cv2.rectangle(frame, (60, cy), (self.width - 60, cy + 70), (60, 60, 80), 2)

        # Rotation marks on collector
        if rpm > 0:
            num_marks = 4
            for i in range(num_marks):
                angle = (t * rpm / 60.0 * 2 * math.pi + i * math.pi / 2) % (2 * math.pi)
                mx = int(self.width / 2 + 200 * math.cos(angle))
                if 60 < mx < self.width - 60:
                    cv2.line(frame, (mx, cy + 2), (mx, cy + 68), (50, 50, 65), 1)

        # Fiber deposition on collector
        if dep_coverage > 0.01:
            np.random.seed(int(t * 3) % 10000)
            num_fibers = int(dep_coverage * 200)
            for _ in range(num_fibers):
                x1 = np.random.randint(65, self.width - 65)
                x2 = x1 + np.random.randint(-30, 30)
                y1 = np.random.randint(cy + 3, cy + 67)
                y2 = y1 + np.random.randint(-3, 3)
                b = np.random.randint(120, 220)
                cv2.line(frame, (x1, y1), (x2, y2), (b, b, b + 10), 1)

        # Syringe/needle
        needle_x = self.width // 2
        needle_top = int(self.height * 0.05)
        needle_bottom = int(self.height * 0.28)
        cv2.rectangle(frame, (needle_x - 4, needle_top), (needle_x + 4, needle_bottom),
                      (180, 180, 190), -1)
        cv2.rectangle(frame, (needle_x - 12, needle_top - 15), (needle_x + 12, needle_top),
                      (60, 60, 70), -1)

        # Taylor cone (only when HV is on and flow > 0)
        if hv_on and flow > 0.01:
            cone_y = needle_bottom
            cone_height = int(30 + hv_kv * 2)
            cone_width = int(15 + flow * 3)
            cone_pts = np.array([
                [needle_x, cone_y],
                [needle_x - cone_width, cone_y + cone_height],
                [needle_x + cone_width, cone_y + cone_height]
            ])
            # Cone brightness proportional to voltage
            cone_bright = int(np.clip(150 + hv_kv * 5, 150, 255))
            cv2.fillPoly(frame, [cone_pts], (cone_bright, cone_bright, cone_bright + 10))

            # Jet from cone to collector
            jet_wobble = int(8 * math.sin(t * 3.0) * (1.0 + flow * 0.1))
            jet_start_y = cone_y + cone_height
            jet_end_y = cy
            jet_color = (200, 200, 220)
            cv2.line(frame,
                     (needle_x + jet_wobble, jet_start_y),
                     (needle_x + jet_wobble // 2, jet_end_y),
                     jet_color, 2)

            # Glow effect around jet
            glow_frame = frame.copy()
            cv2.line(glow_frame,
                     (needle_x + jet_wobble, jet_start_y),
                     (needle_x + jet_wobble // 2, jet_end_y),
                     (100, 100, 150), 6)
            frame = cv2.addWeighted(frame, 0.85, glow_frame, 0.15, 0)

            # Droplets at jet end (beading simulation)
            if np.random.random() < 0.3 * flow:
                dx = np.random.randint(-15, 15)
                dy = np.random.randint(-5, 5)
                cv2.circle(frame, (needle_x + jet_wobble // 2 + dx, jet_end_y + dy),
                           np.random.randint(2, 5), (180, 180, 200), -1)
        else:
            # Drip from needle when no HV
            if flow > 0.01 and np.random.random() < 0.05:
                drip_y = needle_bottom + int((t * 20) % (cy - needle_bottom))
                cv2.circle(frame, (needle_x, drip_y), 3, (150, 150, 170), -1)

        # HV cable (yellow)
        cv2.line(frame, (needle_x - 4, needle_top - 5), (needle_x - 80, needle_top - 30),
                 (200, 200, 0), 2)

        # Status overlay
        status_lines = [
            f"HV: {'ON' if hv_on else 'OFF'} {hv_kv:.1f}kV",
            f"Flow: {flow:.2f} mL/hr",
            f"RPM: {rpm:.0f}",
            f"Dep: {dep_coverage*100:.1f}%",
            f"T: {temp:.1f}C",
        ]
        for i, line in enumerate(status_lines):
            color = (0, 200, 100) if i == 0 and hv_on else (180, 180, 180)
            cv2.putText(frame, line, (10, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Camera noise
        noise = np.random.normal(0, 3, frame.shape).astype(np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Timestamp watermark
        ts = time.strftime("%H:%M:%S", time.localtime(t))
        ms = int((t % 1) * 1000)
        cv2.putText(frame, f"{ts}.{ms:03d}", (self.width - 120, self.height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        self._frame_count += 1
        return frame

    def frame_to_msg(self, frame) -> Optional[Image]:
        if self._bridge and frame is not None:
            try:
                return self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            except Exception:
                pass
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Predictive Maintenance System
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ComponentHealth:
    name: str
    wear: float = 0.0          # 0-1
    remaining_hours: float = 0.0
    status: str = "OK"         # OK, WARNING, CRITICAL, REPLACE
    message: str = ""


class PredictiveMaintenance:
    """Tracks component wear and predicts remaining useful life."""

    def __init__(self):
        self.components: Dict[str, ComponentHealth] = {}
        self.alerts: List[str] = []
        self.last_alert_time: float = 0.0

    def update(self, pump: PeristalticPumpTwin, collector: CollectorDrumTwin,
               hv: HVSupplyTwin):
        self.components = {
            "pump_tubing": ComponentHealth(
                name="Pump Tubing",
                wear=pump.tubing_wear,
                remaining_hours=max(0, 2000.0 * (1.0 - pump.tubing_wear)),
                status=self._wear_status(pump.tubing_wear),
                message="Replace tubing" if pump.tubing_wear > 0.8 else "Normal wear"
            ),
            "pump_roller": ComponentHealth(
                name="Pump Roller",
                wear=pump.roller_wear,
                remaining_hours=max(0, 5000.0 * (1.0 - pump.roller_wear)),
                status=self._wear_status(pump.roller_wear),
                message="Replace roller" if pump.roller_wear > 0.8 else "Normal wear"
            ),
            "pump_motor": ComponentHealth(
                name="Pump Motor",
                wear=min(1.0, pump.motor_temp_c / 80.0),
                remaining_hours=max(0, 10000.0 * (1.0 - min(1.0, pump.motor_temp_c / 80.0))),
                status="CRITICAL" if pump.motor_temp_c > 70 else ("WARNING" if pump.motor_temp_c > 55 else "OK"),
                message=f"Temp {pump.motor_temp_c:.1f}C" + (" OVERHEAT" if pump.motor_temp_c > 70 else "")
            ),
            "collector_bearing": ComponentHealth(
                name="Collector Bearing",
                wear=collector.bearing_wear,
                remaining_hours=max(0, 3000.0 * (1.0 - collector.bearing_wear)),
                status=self._wear_status(collector.bearing_wear),
                message="Replace bearing" if collector.bearing_wear > 0.8 else "Normal wear"
            ),
            "collector_belt": ComponentHealth(
                name="Collector Belt",
                wear=collector.belt_wear,
                remaining_hours=max(0, 1500.0 * (1.0 - collector.belt_wear)),
                status=self._wear_status(collector.belt_wear),
                message="Replace belt" if collector.belt_wear > 0.8 else "Normal wear"
            ),
            "collector_motor": ComponentHealth(
                name="Collector Motor",
                wear=collector.motor_wear,
                remaining_hours=max(0, 8000.0 * (1.0 - collector.motor_wear)),
                status=self._wear_status(collector.motor_wear),
                message="Replace motor" if collector.motor_wear > 0.8 else "Normal wear"
            ),
            "hv_insulation": ComponentHealth(
                name="HV Insulation",
                wear=hv.insulation_wear,
                remaining_hours=max(0, 5000.0 * (1.0 - hv.insulation_wear)),
                status=self._wear_status(hv.insulation_wear),
                message="Replace HV cables" if hv.insulation_wear > 0.8 else "Normal wear"
            ),
        }

        # Generate alerts
        self.alerts = []
        now = time.time()
        for key, comp in self.components.items():
            if comp.status == "CRITICAL":
                self.alerts.append(f"CRITICAL: {comp.name} - {comp.message}")
            elif comp.status == "WARNING":
                self.alerts.append(f"WARNING: {comp.name} - {comp.message} ({comp.remaining_hours:.0f}h left)")

        if hv.arc_count > 0 and now - hv.last_arc_time < 5.0:
            self.alerts.append(f"ALERT: HV arc detected (total: {hv.arc_count})")

    def _wear_status(self, wear: float) -> str:
        if wear > 0.8:
            return "REPLACE"
        elif wear > 0.6:
            return "CRITICAL"
        elif wear > 0.4:
            return "WARNING"
        return "OK"

    def to_dict(self) -> dict:
        return {
            "components": {k: asdict(v) for k, v in self.components.items()},
            "alerts": self.alerts,
            "health_score": self._overall_health(),
        }

    def _overall_health(self) -> float:
        if not self.components:
            return 1.0
        wears = [c.wear for c in self.components.values()]
        return float(np.clip(1.0 - max(wears), 0.0, 1.0))


# ─────────────────────────────────────────────────────────────────────────────
# Supabase Recorder
# ─────────────────────────────────────────────────────────────────────────────

class SupabaseRecorder:
    """Records digital twin telemetry to Supabase in real-time."""

    def __init__(self, supabase_url: str, supabase_key: str):
        self.url = supabase_url.rstrip("/")
        self.key = supabase_key
        self._buffer = []
        self._buffer_lock = threading.Lock()
        self._flush_interval = 2.0  # seconds
        self._last_flush = time.time()
        self._running = True
        self._flush_thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._flush_thread.start()

    def record(self, table: str, data: dict):
        entry = {"table": table, "data": data, "ts": time.time()}
        with self._buffer_lock:
            self._buffer.append(entry)

    def _flush_loop(self):
        while self._running:
            time.sleep(self._flush_interval)
            self._flush()

    def _flush(self):
        with self._buffer_lock:
            if not self._buffer:
                return
            batch = list(self._buffer)
            self._buffer.clear()

        # Group by table
        tables = {}
        for entry in batch:
            t = entry["table"]
            if t not in tables:
                tables[t] = []
            d = entry["data"].copy()
            d["recorded_at"] = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(entry["ts"]))
            tables[t].append(d)

        for table, rows in tables.items():
            self._post_table(table, rows)

    def _post_table(self, table: str, rows: list):
        try:
            url = f"{self.url}/rest/v1/{table}"
            data = json.dumps(rows).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={
                    "Content-Type": "application/json",
                    "apikey": self.key,
                    "Authorization": f"Bearer {self.key}",
                    "Prefer": "return=minimal",
                },
                method="POST",
            )
            urllib.request.urlopen(req, timeout=5)
        except Exception:
            pass  # Silently fail to not disrupt simulation

    def stop(self):
        self._running = False
        self._flush()


# ─────────────────────────────────────────────────────────────────────────────
# Digital Twin Bridge Node
# ─────────────────────────────────────────────────────────────────────────────

class DigitalTwinBridgeNode(Node):
    """
    ROS2 node that bridges digital twin physics simulation with ROS2 topics.
    Includes camera twin, predictive maintenance, and Supabase recording.
    """

    def __init__(self):
        super().__init__("digital_twin_bridge")

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("camera_fps", 10.0)
        self.declare_parameter("enable_recording", True)
        self.declare_parameter("supabase_url", "")
        self.declare_parameter("supabase_key", "")

        sim = self.get_parameter("simulation_mode").value
        rate = self.get_parameter("update_rate_hz").value
        cam_fps = self.get_parameter("camera_fps").value
        enable_rec = self.get_parameter("enable_recording").value
        sb_url = self.get_parameter("supabase_url").value
        sb_key = self.get_parameter("supabase_key").value

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
        self.camera = CameraTwin()
        self.maintenance = PredictiveMaintenance()

        # ── Supabase Recorder ────────────────────────────────────────────────
        self.recorder = None
        if enable_rec and sb_url and sb_key:
            self.recorder = SupabaseRecorder(sb_url, sb_key)
            self.get_logger().info("[DigitalTwinBridge] Supabase recording enabled")
        elif enable_rec:
            # Try from environment
            import os
            env_url = os.environ.get("VITE_SUPABASE_URL", "")
            env_key = os.environ.get("VITE_SUPABASE_SUPABASE_ANON_KEY", "")
            if env_url and env_key:
                self.recorder = SupabaseRecorder(env_url, env_key)
                self.get_logger().info("[DigitalTwinBridge] Supabase recording from env")

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

        # ── Publishers (Environment) ─────────────────────────────────────────
        self.pub_env_temp = self.create_publisher(Float32, "/env/temperature", qos)
        self.pub_env_humidity = self.create_publisher(Float32, "/env/humidity", qos)

        # ── Publishers (Deposition) ─────────────────────────────────────────
        self.pub_deposition = self.create_publisher(String, "/fiber_deposition", qos)

        # ── Publishers (Joint States for Gazebo) ─────────────────────────────
        self.pub_joint_states = self.create_publisher(JointState, "/digital_twin/joint_states", sensor_qos)

        # ── Publishers (Camera) ──────────────────────────────────────────────
        self.pub_camera = self.create_publisher(Image, "/digital_twin/camera", sensor_qos)
        self.pub_camera_debug = self.create_publisher(Image, "/digital_twin/camera_debug", sensor_qos)

        # ── Publishers (Maintenance) ────────────────────────────────────────
        self.pub_maintenance = self.create_publisher(String, "/maintenance/status", qos)
        self.pub_maintenance_alerts = self.create_publisher(String, "/maintenance/alerts", qos)

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

        # ── Camera Timer (lower rate) ────────────────────────────────────────
        self._cam_timer = self.create_timer(1.0 / cam_fps, self._camera_loop)

        # ── Recording Timer ──────────────────────────────────────────────────
        self._rec_interval = 1.0  # record every 1 second
        self._last_rec_time = 0.0

        self.get_logger().info(
            f"[DigitalTwinBridge] Initialized. Rate={rate}Hz, Cam={cam_fps}fps, "
            f"Sim={sim}, Recording={self.recorder is not None}"
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

        self.pump.update(dt)
        self.collector.update(dt)
        self.hv.update(dt)
        self.env.update(dt, hv_active=self.hv.enabled, pump_flow=self.pump.actual_flow_ml_hr,
                        collector_rpm=self.collector.actual_rpm)
        self.deposition.update(dt, self.collector.actual_rpm, self.pump.actual_flow_ml_hr)
        self.maintenance.update(self.pump, self.collector, self.hv)

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
            "tubing_wear": round(self.pump.tubing_wear, 3),
            "roller_wear": round(self.pump.roller_wear, 3),
            "motor_temp_c": round(self.pump.motor_temp_c, 1),
        })
        self.pub_pump_status.publish(pump_status)

        # ── Publish Collector ────────────────────────────────────────────────
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
            "bearing_wear": round(self.collector.bearing_wear, 3),
            "belt_wear": round(self.collector.belt_wear, 3),
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
            "arc_count": self.hv.arc_count,
            "insulation_wear": round(self.hv.insulation_wear, 3),
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

        # ── Publish Joint States ─────────────────────────────────────────────
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["pump_roller_joint", "drum_rotation_joint"]
        js.position = [self.pump.roller_angle, self.collector.drum_angle]
        js.velocity = [
            self.pump.roller_rpm / 60.0 * 2.0 * math.pi,
            self.collector.actual_rpm / 60.0 * 2.0 * math.pi,
        ]
        self.pub_joint_states.publish(js)

        # ── Publish Maintenance ──────────────────────────────────────────────
        maint_data = self.maintenance.to_dict()
        maint_msg = String()
        maint_msg.data = json.dumps(maint_data)
        self.pub_maintenance.publish(maint_msg)

        if self.maintenance.alerts:
            alert_msg = String()
            alert_msg.data = json.dumps({"alerts": self.maintenance.alerts,
                                         "timestamp": time.time()})
            self.pub_maintenance_alerts.publish(alert_msg)

        # ── Record to Supabase ──────────────────────────────────────────────
        now = time.time()
        if self.recorder and (now - self._last_rec_time) >= self._rec_interval:
            self._last_rec_time = now
            self._record_telemetry()

    # ── Camera Loop ──────────────────────────────────────────────────────────

    def _camera_loop(self):
        process_state = {
            "hv_enabled": self.hv.enabled,
            "hv_voltage_kv": self.hv.actual_voltage_kv,
            "pump_flow_ml_hr": self.pump.actual_flow_ml_hr,
            "collector_rpm": self.collector.actual_rpm,
            "dep_coverage": float(self.deposition.coverage_map.mean()),
            "env_temp": self.env.temperature,
        }

        frame = self.camera.generate_frame(process_state)
        if frame is not None:
            img_msg = self.camera.frame_to_msg(frame)
            if img_msg is not None:
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_camera.publish(img_msg)

            # Debug frame with annotations
            if CV_AVAILABLE:
                debug = frame.copy()
                self._draw_camera_debug(debug, process_state)
                dbg_msg = self.camera.frame_to_msg(debug)
                if dbg_msg is not None:
                    dbg_msg.header.stamp = self.get_clock().now().to_msg()
                    self.pub_camera_debug.publish(dbg_msg)

    def _draw_camera_debug(self, frame, state: dict):
        """Draw additional debug overlays on camera frame."""
        if not CV_AVAILABLE:
            return
        h, w = frame.shape[:2]

        # Maintenance status bar at bottom
        maint = self.maintenance.to_dict()
        health = maint.get("health_score", 1.0)
        bar_w = int(w * health)
        color = (0, 200, 0) if health > 0.6 else ((0, 200, 200) if health > 0.3 else (0, 0, 200))
        cv2.rectangle(frame, (0, h - 25), (bar_w, h - 20), color, -1)
        cv2.putText(frame, f"Health: {health*100:.0f}%", (5, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        # Alert count
        num_alerts = len(maint.get("alerts", []))
        if num_alerts > 0:
            cv2.putText(frame, f"Alerts: {num_alerts}", (w - 100, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    # ── Recording ────────────────────────────────────────────────────────────

    def _record_telemetry(self):
        if not self.recorder:
            return

        ts = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())

        self.recorder.record("twin_telemetry", {
            "recorded_at": ts,
            "pump_flow_ml_hr": round(self.pump.actual_flow_ml_hr, 3),
            "pump_pressure_kpa": round(self.pump.pressure_kpa, 2),
            "pump_running": self.pump.running,
            "pump_tubing_wear": round(self.pump.tubing_wear, 4),
            "pump_roller_wear": round(self.pump.roller_wear, 4),
            "pump_motor_temp_c": round(self.pump.motor_temp_c, 1),
            "collector_rpm": round(self.collector.actual_rpm, 1),
            "collector_vibration": round(self.collector.vibration_score, 3),
            "collector_temp_c": round(self.collector.temperature_c, 1),
            "collector_bearing_wear": round(self.collector.bearing_wear, 4),
            "collector_belt_wear": round(self.collector.belt_wear, 4),
            "hv_voltage_kv": round(self.hv.actual_voltage_kv, 2),
            "hv_enabled": self.hv.enabled,
            "hv_arc_count": self.hv.arc_count,
            "hv_insulation_wear": round(self.hv.insulation_wear, 4),
            "env_temp_c": round(self.env.temperature, 1),
            "env_humidity_pct": round(self.env.humidity, 1),
            "dep_coverage_pct": round(float(self.deposition.coverage_map.mean()) * 100, 1),
            "dep_total_mg": round(self.deposition.total_deposited_mg, 2),
            "health_score": round(self.maintenance._overall_health(), 3),
        })

        if self.maintenance.alerts:
            for alert in self.maintenance.alerts:
                self.recorder.record("twin_alerts", {
                    "recorded_at": ts,
                    "alert_text": alert,
                })

    def destroy_node(self):
        if self.recorder:
            self.recorder.stop()
        super().destroy_node()


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
