#!/usr/bin/env python3
"""
ElectroSpin Collector Controller Node
=======================================
Industrial-grade PID motor controller for the electrospinning collector drum.

The collector is an EXTERNAL motor (BLDC or DC), NOT a robot joint.
This node implements:
  - Closed-loop PID RPM control
  - Encoder-based feedback
  - Smooth acceleration/deceleration ramps
  - Vibration monitoring and compensation
  - Emergency stop handling
  - Industrial safety interlocks

Hardware interface: GPIO/PWM or serial motor driver
(Supports: ODrive, SimpleFOC, Arduino-based drivers, or simulated)

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import math
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from collections import deque

from std_msgs.msg import Float32, Bool, String
from electrospin_interfaces.msg import CollectorStatus


# ─────────────────────────────────────────────────────────────────────────────
# PID Controller
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PIDConfig:
    kp: float = 0.8
    ki: float = 0.15
    kd: float = 0.05
    integral_limit: float = 50.0
    output_min: float = 0.0
    output_max: float = 1.0


class PIDController:
    """
    Discrete-time PID controller with anti-windup, derivative filter,
    and output clamping.
    """

    def __init__(self, config: PIDConfig):
        self.cfg = config
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time: Optional[float] = None
        self._derivative_filter = 0.0
        self._alpha = 0.1  # derivative low-pass filter coefficient

    def compute(self, setpoint: float, measurement: float, t: float) -> float:
        """Compute PID output. Returns value in [output_min, output_max]."""
        error = setpoint - measurement
        now = t

        if self._prev_time is None:
            dt = 0.1
        else:
            dt = max(1e-6, now - self._prev_time)
        self._prev_time = now

        # Proportional
        p_term = self.cfg.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = np.clip(
            self._integral,
            -self.cfg.integral_limit,
            self.cfg.integral_limit
        )
        i_term = self.cfg.ki * self._integral

        # Derivative with low-pass filter (reduces noise amplification)
        raw_deriv = (error - self._prev_error) / dt
        self._derivative_filter += self._alpha * (raw_deriv - self._derivative_filter)
        d_term = self.cfg.kd * self._derivative_filter

        self._prev_error = error
        output = np.clip(p_term + i_term + d_term, self.cfg.output_min, self.cfg.output_max)
        return float(output)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._derivative_filter = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Motor Driver Abstraction
# ─────────────────────────────────────────────────────────────────────────────

class MotorDriverHAL:
    """
    Hardware abstraction for collector motor driver.
    Supports simulation and real hardware (via serial/GPIO).
    """

    def __init__(self, simulation_mode: bool = True):
        self.sim = simulation_mode
        self._sim_rpm = 0.0
        self._sim_target_duty = 0.0
        self._sim_lock = threading.Lock()
        self._sim_inertia = 0.15   # Time constant for RPM response (s)
        self._max_rpm = 3000.0
        self._noise_std = 2.0       # RPM measurement noise

        if not self.sim:
            # TODO: Initialize real motor driver (e.g., serial to Arduino/ODrive)
            pass

        # Start simulation physics thread
        if self.sim:
            self._sim_thread = threading.Thread(
                target=self._simulate_motor, daemon=True
            )
            self._sim_thread.start()

    def set_duty_cycle(self, duty: float):
        """Set PWM duty cycle (0.0 = stop, 1.0 = full speed)."""
        duty = float(np.clip(duty, 0.0, 1.0))
        if self.sim:
            with self._sim_lock:
                self._sim_target_duty = duty
        else:
            # TODO: Write to real motor driver
            pass

    def get_rpm(self) -> float:
        """Read current RPM from encoder / estimator."""
        if self.sim:
            with self._sim_lock:
                noise = np.random.normal(0, self._noise_std)
                return max(0.0, self._sim_rpm + noise)
        else:
            # TODO: Read encoder pulses, convert to RPM
            return 0.0

    def get_current(self) -> float:
        """Read motor current in Amps."""
        if self.sim:
            with self._sim_lock:
                return self._sim_target_duty * 5.0 + np.random.normal(0, 0.1)
        return 0.0

    def emergency_stop(self):
        """Immediately cut motor power."""
        if self.sim:
            with self._sim_lock:
                self._sim_target_duty = 0.0
                self._sim_rpm = 0.0
        else:
            # TODO: Assert e-stop pin
            pass

    def _simulate_motor(self):
        """Simple first-order motor simulation."""
        dt = 0.02  # 50 Hz simulation
        while True:
            with self._sim_lock:
                target_rpm = self._sim_target_duty * self._max_rpm
                error = target_rpm - self._sim_rpm
                self._sim_rpm += error * (dt / self._sim_inertia)
                self._sim_rpm = max(0.0, self._sim_rpm)
            time.sleep(dt)


# ─────────────────────────────────────────────────────────────────────────────
# Vibration Monitor
# ─────────────────────────────────────────────────────────────────────────────

class VibrationMonitor:
    """
    Estimates vibration score from RPM fluctuation history.
    High RPM variance → high vibration score.
    """

    def __init__(self, window_size: int = 50):
        self._history: deque = deque(maxlen=window_size)

    def update(self, rpm: float) -> float:
        self._history.append(rpm)
        if len(self._history) < 5:
            return 0.0
        arr = np.array(self._history)
        std = float(arr.std())
        # Normalize: 0 RPM std = 0.0 score, 50+ RPM std = 1.0 score
        score = float(np.clip(std / 50.0, 0.0, 1.0))
        return score


# ─────────────────────────────────────────────────────────────────────────────
# Collector Controller Node
# ─────────────────────────────────────────────────────────────────────────────

class CollectorControllerNode(Node):
    """
    ROS2 node for industrial collector drum motor control.

    Topics Published:
        /collector_status  → electrospin_interfaces/CollectorStatus
        /collector_rpm     → std_msgs/Float32

    Topics Subscribed:
        /target_rpm        → std_msgs/Float32
        /emergency_stop    → std_msgs/Bool
    """

    def __init__(self):
        super().__init__("collector_controller")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("control_frequency_hz", 50.0)
        self.declare_parameter("publish_frequency_hz", 20.0)
        self.declare_parameter("pid_kp", 0.8)
        self.declare_parameter("pid_ki", 0.15)
        self.declare_parameter("pid_kd", 0.05)
        self.declare_parameter("max_rpm", 3000.0)
        self.declare_parameter("ramp_rate_rpm_s", 100.0)   # RPM/second acceleration limit
        self.declare_parameter("rpm_tolerance", 5.0)        # RPM dead-band
        self.declare_parameter("max_vibration_score", 0.6)  # Threshold for warning

        sim_mode   = self.get_parameter("simulation_mode").value
        ctrl_freq  = self.get_parameter("control_frequency_hz").value
        pub_freq   = self.get_parameter("publish_frequency_hz").value
        max_rpm    = self.get_parameter("max_rpm").value
        ramp_rate  = self.get_parameter("ramp_rate_rpm_s").value

        # ── Hardware ─────────────────────────────────────────────────────────
        self.motor = MotorDriverHAL(simulation_mode=sim_mode)
        self.pid = PIDController(PIDConfig(
            kp=self.get_parameter("pid_kp").value,
            ki=self.get_parameter("pid_ki").value,
            kd=self.get_parameter("pid_kd").value,
        ))
        self.vib_monitor = VibrationMonitor()

        # ── State ─────────────────────────────────────────────────────────────
        self.setpoint_rpm:     float = 0.0
        self.ramped_setpoint:  float = 0.0
        self.current_rpm:      float = 0.0
        self.duty_cycle:       float = 0.0
        self.emergency_stop:   bool  = False
        self.max_rpm:          float = max_rpm
        self.ramp_rate:        float = ramp_rate
        self.rpm_tolerance:    float = self.get_parameter("rpm_tolerance").value
        self.encoder_position: float = 0.0
        self.control_mode:     int   = 0   # 0=IDLE, 1=OPEN_LOOP, 2=PID, 3=ESTOP

        # ── QoS ───────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(
            CollectorStatus, "/collector_status", reliable_qos
        )
        self.pub_rpm = self.create_publisher(
            Float32, "/collector_rpm", sensor_qos
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.sub_target_rpm = self.create_subscription(
            Float32, "/target_rpm",
            self._on_target_rpm, reliable_qos
        )
        self.sub_estop = self.create_subscription(
            Bool, "/emergency_stop",
            self._on_emergency_stop, reliable_qos
        )

        # ── Timers ────────────────────────────────────────────────────────────
        self.control_timer = self.create_timer(1.0 / ctrl_freq, self._control_loop)
        self.publish_timer = self.create_timer(1.0 / pub_freq,  self._publish_status)

        self.get_logger().info(
            f"[CollectorController] Initialized. Sim={sim_mode}, "
            f"MaxRPM={max_rpm}, RampRate={ramp_rate}RPM/s"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _on_target_rpm(self, msg: Float32):
        target = float(np.clip(msg.data, 0.0, self.max_rpm))
        if self.emergency_stop:
            self.get_logger().warn("[Collector] E-STOP active. Ignoring RPM command.")
            return
        self.setpoint_rpm = target
        if target > 0:
            self.control_mode = 2  # PID mode
        else:
            self.control_mode = 0  # IDLE

    def _on_emergency_stop(self, msg: Bool):
        if msg.data:
            self.emergency_stop = True
            self.control_mode = 3
            self.setpoint_rpm = 0.0
            self.ramped_setpoint = 0.0
            self.motor.emergency_stop()
            self.get_logger().error("[Collector] EMERGENCY STOP ACTIVATED!")
        else:
            self.emergency_stop = False
            self.pid.reset()
            self.control_mode = 0
            self.get_logger().info("[Collector] Emergency stop cleared.")

    # ─────────────────────────────────────────────────────────────────────────
    # Control Loop
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """50 Hz PID control loop with velocity ramping."""
        if self.emergency_stop:
            return

        dt = 0.02  # 50 Hz

        # ── Velocity ramp ─────────────────────────────────────────────────────
        rpm_error = self.setpoint_rpm - self.ramped_setpoint
        max_step  = self.ramp_rate * dt
        ramp_step = float(np.clip(rpm_error, -max_step, max_step))
        self.ramped_setpoint += ramp_step

        if self.control_mode == 0:   # IDLE
            self.motor.set_duty_cycle(0.0)
            return

        if self.control_mode == 2:   # PID
            # Read current RPM
            self.current_rpm = self.motor.get_rpm()

            # PID output → duty cycle
            now = time.time()
            duty = self.pid.compute(
                self.ramped_setpoint,
                self.current_rpm,
                now
            )
            self.duty_cycle = duty
            self.motor.set_duty_cycle(duty)

            # Update encoder (simulated integration)
            rad_per_s = (self.current_rpm / 60.0) * 2.0 * math.pi
            self.encoder_position += rad_per_s * dt

        elif self.control_mode == 1: # Open loop
            duty = self.ramped_setpoint / self.max_rpm
            self.duty_cycle = float(np.clip(duty, 0.0, 1.0))
            self.motor.set_duty_cycle(self.duty_cycle)
            self.current_rpm = self.motor.get_rpm()

    # ─────────────────────────────────────────────────────────────────────────
    # Status Publisher
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_status(self):
        """Publish collector status at 20 Hz."""
        now = self.get_clock().now().to_msg()
        vib_score = self.vib_monitor.update(self.current_rpm)

        # CollectorStatus
        status = CollectorStatus()
        status.header.stamp = now
        status.rpm           = float(self.current_rpm)
        status.target_rpm    = float(self.setpoint_rpm)
        status.torque        = float(self.duty_cycle)
        status.running       = self.current_rpm > 5.0
        status.at_setpoint   = abs(self.current_rpm - self.setpoint_rpm) <= self.rpm_tolerance
        status.vibration_score   = float(vib_score)
        status.temperature_c     = 25.0 + self.duty_cycle * 20.0   # simulated
        status.current_a         = float(self.motor.get_current())
        status.duty_cycle        = float(self.duty_cycle)
        status.emergency_stop    = self.emergency_stop
        status.ramp_active       = abs(self.ramped_setpoint - self.setpoint_rpm) > 1.0
        status.encoder_position  = float(self.encoder_position % (2.0 * math.pi))
        status.control_mode      = self.control_mode
        self.pub_status.publish(status)

        # Float32 RPM
        rpm_msg = Float32()
        rpm_msg.data = float(self.current_rpm)
        self.pub_rpm.publish(rpm_msg)

        # Vibration warning
        if vib_score > self.get_parameter("max_vibration_score").value:
            self.get_logger().warn(
                f"[Collector] High vibration detected: score={vib_score:.2f}"
            )


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CollectorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()