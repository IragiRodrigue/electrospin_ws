#!/usr/bin/env python3
"""
ElectroSpin AI Controller Node
================================
Intelligent decision-making system for autonomous nanofiber fabrication.

The AI controller closes the loop between quality perception and actuation:
  FiberQuality + System States
        ↓
  Rule-Based + Adaptive Engine
        ↓
  Optimized ElectrospinCommand

Optimization Strategies:
  1. Rule-Based Correction (fast, deterministic, safety layer)
  2. Gradient-Based Adaptation (continuous parameter tuning)
  3. Reinforcement Learning Hooks (future: PPO/SAC policy)
  4. Multi-Objective Pareto Optimization (quality vs throughput)

Decision Logic Examples:
  - Beading detected     → reduce flow rate, increase RPM
  - Jet instability      → reduce voltage or flow rate
  - Thin fibers wanted   → increase distance, reduce flow
  - Non-uniform coverage → bias robot scan trajectory
  - Low density          → decrease scan speed

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import time
import json
import threading
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Tuple
from collections import deque
from enum import IntEnum

from std_msgs.msg import String, Float32, Bool

from electrospin_interfaces.msg import (
    FiberQuality, CollectorStatus, ElectrospinCommand, SystemStatus
)


# ─────────────────────────────────────────────────────────────────────────────
# Parameter Space
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ProcessParams:
    """Represents a complete set of electrospinning process parameters."""

    # Robot
    distance_mm:   float = 150.0    # Needle-to-collector gap
    scan_speed:    float = 5.0      # mm/s lateral scan
    scan_amplitude: float = 40.0   # mm peak-to-peak

    # Collector
    rpm:           float = 500.0    # Collector RPM

    # Syringe
    flow_rate:     float = 1.0      # mL/hour

    # High Voltage
    voltage_kv:    float = 15.0     # kV

    def clamp(self, limits: "ProcessLimits") -> "ProcessParams":
        """Return a copy with all values clamped to safe limits."""
        return ProcessParams(
            distance_mm   = float(np.clip(self.distance_mm,   limits.dist_min,   limits.dist_max)),
            scan_speed    = float(np.clip(self.scan_speed,    limits.scan_min,   limits.scan_max)),
            scan_amplitude= float(np.clip(self.scan_amplitude,limits.amp_min,    limits.amp_max)),
            rpm           = float(np.clip(self.rpm,           limits.rpm_min,    limits.rpm_max)),
            flow_rate     = float(np.clip(self.flow_rate,     limits.flow_min,   limits.flow_max)),
            voltage_kv    = float(np.clip(self.voltage_kv,    limits.voltage_min,limits.voltage_max)),
        )


@dataclass
class ProcessLimits:
    """Safety boundaries for process parameters."""
    dist_min:    float = 80.0;    dist_max:    float = 250.0
    scan_min:    float = 1.0;     scan_max:    float = 20.0
    amp_min:     float = 10.0;    amp_max:     float = 80.0
    rpm_min:     float = 0.0;     rpm_max:     float = 2500.0
    flow_min:    float = 0.1;     flow_max:    float = 5.0
    voltage_min: float = 5.0;     voltage_max: float = 30.0


# ─────────────────────────────────────────────────────────────────────────────
# Optimization Mode Enum
# ─────────────────────────────────────────────────────────────────────────────

class OptMode(IntEnum):
    IDLE         = 0
    MANUAL       = 1
    RULE_BASED   = 2
    ADAPTIVE     = 3
    RL_POLICY    = 4   # Future: reinforcement learning


# ─────────────────────────────────────────────────────────────────────────────
# Rule-Based Correction Engine
# ─────────────────────────────────────────────────────────────────────────────

class RuleEngine:
    """
    Fast, deterministic rule-based correction layer.
    Acts as safety net and initial optimizer.

    Rules are additive deltas — multiple rules can fire simultaneously.
    """

    def __init__(self, limits: ProcessLimits):
        self.limits = limits

    def evaluate(
        self,
        quality: FiberQuality,
        params:  ProcessParams
    ) -> Tuple[ProcessParams, List[str]]:
        """
        Evaluate all rules and return adjusted params + decision log.
        """
        delta = ProcessParams(0, 0, 0, 0, 0, 0)
        log:  List[str] = []

        # ── Rule 1: Severe beading → reduce flow, increase RPM ───────────────
        if quality.bead_score > 0.6:
            delta.flow_rate -= 0.3
            delta.rpm       += 100.0
            log.append(f"R1: Severe beading ({quality.bead_score:.2f}) → ↓flow ↑RPM")

        elif quality.bead_score > 0.3:
            delta.flow_rate -= 0.15
            delta.rpm       += 50.0
            log.append(f"R1: Moderate beading ({quality.bead_score:.2f}) → ↓flow ↑RPM")

        # ── Rule 2: Jet instability → reduce flow/voltage ─────────────────────
        if not quality.jet_stable:
            delta.flow_rate  -= 0.1
            delta.voltage_kv -= 0.5
            log.append("R2: Jet unstable → ↓flow ↓voltage")

        elif quality.taylor_cone_score < 0.3:
            delta.voltage_kv += 0.3
            log.append(f"R2: Weak Taylor cone ({quality.taylor_cone_score:.2f}) → ↑voltage")

        # ── Rule 3: Poor fiber uniformity → adjust distance ───────────────────
        if quality.uniformity < 0.4:
            delta.distance_mm += 10.0
            delta.scan_speed  -= 1.0
            log.append(f"R3: Low uniformity ({quality.uniformity:.2f}) → ↑distance ↓scan")

        # ── Rule 4: Fiber too thick → increase distance + RPM ────────────────
        if quality.diameter > 2000.0:  # > 2000 nm is too thick
            delta.distance_mm += 15.0
            delta.rpm         += 80.0
            log.append(f"R4: Thick fibers ({quality.diameter:.0f}nm) → ↑distance ↑RPM")

        # ── Rule 5: Fiber too thin → decrease distance ────────────────────────
        elif quality.diameter < 100.0:  # < 100 nm is too thin / fragile
            delta.distance_mm -= 10.0
            log.append(f"R5: Thin fibers ({quality.diameter:.0f}nm) → ↓distance")

        # ── Rule 6: Non-uniform deposition → increase scan amplitude ─────────
        if quality.coverage_uniformity < 0.35:
            delta.scan_amplitude += 5.0
            delta.scan_speed     += 1.0
            log.append(f"R6: Non-uniform coverage ({quality.coverage_uniformity:.2f}) → ↑amp")

        # ── Rule 7: Low deposition density → slow down scan ──────────────────
        if quality.deposition_density < 0.15:
            delta.scan_speed    -= 1.0
            delta.flow_rate     += 0.1
            log.append(f"R7: Low density ({quality.deposition_density:.2f}) → ↓scan ↑flow")

        # Apply deltas and clamp
        adjusted = ProcessParams(
            distance_mm    = params.distance_mm    + delta.distance_mm,
            scan_speed     = params.scan_speed     + delta.scan_speed,
            scan_amplitude = params.scan_amplitude + delta.scan_amplitude,
            rpm            = params.rpm            + delta.rpm,
            flow_rate      = params.flow_rate      + delta.flow_rate,
            voltage_kv     = params.voltage_kv     + delta.voltage_kv,
        ).clamp(self.limits)

        return adjusted, log


# ─────────────────────────────────────────────────────────────────────────────
# Adaptive Gradient Optimizer
# ─────────────────────────────────────────────────────────────────────────────

class AdaptiveOptimizer:
    """
    Gradient-free adaptive optimizer using finite-difference estimation.
    Tracks quality over time and nudges parameters in directions that
    improved quality in recent history.

    Future extension: Replace with PPO/SAC RL agent.
    """

    def __init__(self, limits: ProcessLimits, history_len: int = 20):
        self.limits = limits
        self.history: deque = deque(maxlen=history_len)
        self._step_size = 0.05   # Fractional step size of parameter range

    def record(self, params: ProcessParams, quality: float):
        self.history.append((asdict(params), quality))

    def suggest(self, current: ProcessParams) -> ProcessParams:
        """
        Suggest parameter adjustments based on quality gradient history.
        """
        if len(self.history) < 4:
            return current

        # Estimate quality gradient via finite difference over history
        recent = list(self.history)[-10:]
        qualities = np.array([r[1] for r in recent])

        if qualities.std() < 0.01:
            # No variation → explore slightly
            noise = {
                "distance_mm":    np.random.normal(0, 3.0),
                "scan_speed":     np.random.normal(0, 0.3),
                "scan_amplitude": np.random.normal(0, 2.0),
                "rpm":            np.random.normal(0, 20.0),
                "flow_rate":      np.random.normal(0, 0.05),
                "voltage_kv":     np.random.normal(0, 0.2),
            }
            return ProcessParams(
                distance_mm    = current.distance_mm    + noise["distance_mm"],
                scan_speed     = current.scan_speed     + noise["scan_speed"],
                scan_amplitude = current.scan_amplitude + noise["scan_amplitude"],
                rpm            = current.rpm            + noise["rpm"],
                flow_rate      = current.flow_rate      + noise["flow_rate"],
                voltage_kv     = current.voltage_kv     + noise["voltage_kv"],
            ).clamp(self.limits)

        # Weighted average of best parameter sets
        weights = np.exp((qualities - qualities.max()) / (qualities.std() + 1e-6))
        weights /= weights.sum()

        blended = {}
        fields = ["distance_mm", "scan_speed", "scan_amplitude", "rpm", "flow_rate", "voltage_kv"]
        for f in fields:
            vals = np.array([r[0][f] for r in recent])
            blended[f] = float(np.dot(weights, vals))

        return ProcessParams(**{f: blended[f] for f in fields}).clamp(self.limits)


# ─────────────────────────────────────────────────────────────────────────────
# AI Controller Node
# ─────────────────────────────────────────────────────────────────────────────

class AIControllerNode(Node):
    """
    Autonomous AI decision controller for electrospinning optimization.

    Topics Published:
        /electrospin_command   → electrospin_interfaces/ElectrospinCommand
        /ai_status             → std_msgs/String (JSON decision log)

    Topics Subscribed:
        /fiber_quality         → electrospin_interfaces/FiberQuality
        /collector_status      → electrospin_interfaces/CollectorStatus
        /robot_status          → std_msgs/String
        /pump_status           → std_msgs/String
    """

    def __init__(self):
        super().__init__("ai_controller")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("optimization_mode", "adaptive")  # rule|adaptive|off
        self.declare_parameter("decision_frequency_hz", 2.0)
        self.declare_parameter("quality_target", 0.75)
        self.declare_parameter("command_smoothing_alpha", 0.3)   # EMA for commands
        self.declare_parameter("enable_voltage_control", False)  # Safety: off by default
        self.declare_parameter("min_quality_for_adaptation", 0.1)

        mode_str   = self.get_parameter("optimization_mode").value
        dec_freq   = self.get_parameter("decision_frequency_hz").value
        self.quality_target = self.get_parameter("quality_target").value
        self.cmd_alpha      = self.get_parameter("command_smoothing_alpha").value
        self.hv_enabled     = self.get_parameter("enable_voltage_control").value

        # ── Optimization mode ─────────────────────────────────────────────────
        self.opt_mode = {
            "off":      OptMode.IDLE,
            "rule":     OptMode.RULE_BASED,
            "adaptive": OptMode.ADAPTIVE,
            "rl":       OptMode.RL_POLICY,
        }.get(mode_str, OptMode.ADAPTIVE)

        # ── Components ────────────────────────────────────────────────────────
        self.limits   = ProcessLimits()
        self.rules    = RuleEngine(self.limits)
        self.adaptive = AdaptiveOptimizer(self.limits)

        # ── State ─────────────────────────────────────────────────────────────
        self.current_params = ProcessParams()   # Current setpoints
        self.smoothed_params = ProcessParams()  # EMA-smoothed command output

        self.latest_quality:   Optional[FiberQuality]   = None
        self.latest_collector: Optional[CollectorStatus] = None
        self.latest_robot:     Dict = {}
        self.opt_cycle_count:  int  = 0
        self.session_quality_history: deque = deque(maxlen=200)
        self.decision_log: List[str] = []
        self.autonomous_mode: bool = True

        # ── QoS ───────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_command   = self.create_publisher(
            ElectrospinCommand, "/electrospin_command", reliable_qos
        )
        self.pub_ai_status = self.create_publisher(
            String, "/ai_status", reliable_qos
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.sub_quality = self.create_subscription(
            FiberQuality, "/fiber_quality",
            self._on_fiber_quality, sensor_qos
        )
        self.sub_collector = self.create_subscription(
            CollectorStatus, "/collector_status",
            self._on_collector_status, sensor_qos
        )
        self.sub_robot = self.create_subscription(
            String, "/robot_status",
            self._on_robot_status, reliable_qos
        )
        self.sub_manual_override = self.create_subscription(
            Bool, "/manual_override",
            self._on_manual_override, reliable_qos
        )

        # ── Decision Timer ────────────────────────────────────────────────────
        self.decision_timer = self.create_timer(
            1.0 / dec_freq, self._decision_cycle
        )

        # Publish initial default command
        self._publish_command(self.current_params, confidence=0.5,
                              rationale="Initialization — default params")

        self.get_logger().info(
            f"[AIController] Initialized. Mode={mode_str}, "
            f"TargetQuality={self.quality_target}, Freq={dec_freq}Hz"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Subscriber Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _on_fiber_quality(self, msg: FiberQuality):
        self.latest_quality = msg
        if msg.overall_quality > 0:
            self.session_quality_history.append(msg.overall_quality)

    def _on_collector_status(self, msg: CollectorStatus):
        self.latest_collector = msg

    def _on_robot_status(self, msg: String):
        try:
            self.latest_robot = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_manual_override(self, msg: Bool):
        self.autonomous_mode = not msg.data
        mode_str = "MANUAL" if msg.data else "AUTONOMOUS"
        self.get_logger().info(f"[AIController] Switched to {mode_str} mode")

    # ─────────────────────────────────────────────────────────────────────────
    # Decision Cycle
    # ─────────────────────────────────────────────────────────────────────────

    def _decision_cycle(self):
        """Main AI decision cycle, called at decision_frequency_hz."""
        if not self.autonomous_mode or self.opt_mode == OptMode.IDLE:
            return

        if self.latest_quality is None:
            self.get_logger().debug("[AIController] Waiting for quality data...")
            return

        quality = self.latest_quality
        self.opt_cycle_count += 1
        self.decision_log.clear()

        # ── Check if quality is already acceptable ────────────────────────────
        if quality.overall_quality >= self.quality_target:
            self.decision_log.append(
                f"Quality {quality.overall_quality:.2f} ≥ target {self.quality_target:.2f} → holding"
            )
            # Only minor adaptive tweaks when at target
            if self.opt_mode == OptMode.ADAPTIVE:
                self.adaptive.record(self.current_params, quality.overall_quality)

        else:
            # ── Rule-based correction ─────────────────────────────────────────
            rule_params, rule_log = self.rules.evaluate(quality, self.current_params)
            self.decision_log.extend(rule_log)

            # ── Adaptive refinement ───────────────────────────────────────────
            if self.opt_mode == OptMode.ADAPTIVE:
                self.adaptive.record(self.current_params, quality.overall_quality)
                adaptive_params = self.adaptive.suggest(rule_params)

                # Blend: 70% rule-based + 30% adaptive
                def blend(r, a, w=0.3):
                    return r * (1 - w) + a * w

                final_params = ProcessParams(
                    distance_mm    = blend(rule_params.distance_mm,    adaptive_params.distance_mm),
                    scan_speed     = blend(rule_params.scan_speed,     adaptive_params.scan_speed),
                    scan_amplitude = blend(rule_params.scan_amplitude, adaptive_params.scan_amplitude),
                    rpm            = blend(rule_params.rpm,            adaptive_params.rpm),
                    flow_rate      = blend(rule_params.flow_rate,      adaptive_params.flow_rate),
                    voltage_kv     = blend(rule_params.voltage_kv,     adaptive_params.voltage_kv),
                ).clamp(self.limits)
                self.decision_log.append("Adaptive blending applied")
            else:
                final_params = rule_params

            self.current_params = final_params

        # ── Apply EMA smoothing to commands ───────────────────────────────────
        α = self.cmd_alpha
        self.smoothed_params = ProcessParams(
            distance_mm    = α * self.current_params.distance_mm    + (1-α) * self.smoothed_params.distance_mm,
            scan_speed     = α * self.current_params.scan_speed     + (1-α) * self.smoothed_params.scan_speed,
            scan_amplitude = α * self.current_params.scan_amplitude + (1-α) * self.smoothed_params.scan_amplitude,
            rpm            = α * self.current_params.rpm            + (1-α) * self.smoothed_params.rpm,
            flow_rate      = α * self.current_params.flow_rate      + (1-α) * self.smoothed_params.flow_rate,
            voltage_kv     = α * self.current_params.voltage_kv     + (1-α) * self.smoothed_params.voltage_kv,
        )

        # ── Compute confidence ────────────────────────────────────────────────
        confidence = float(np.clip(
            quality.overall_quality + 0.1 * len(self.session_quality_history) / 50.0,
            0.1, 1.0
        ))

        rationale = " | ".join(self.decision_log) if self.decision_log else "Nominal operation"

        # ── Publish command ───────────────────────────────────────────────────
        self._publish_command(self.smoothed_params, confidence, rationale)

        # ── Publish AI status ─────────────────────────────────────────────────
        self._publish_status(quality, rationale)

    # ─────────────────────────────────────────────────────────────────────────
    # Command Publisher
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_command(
        self,
        params: ProcessParams,
        confidence: float,
        rationale: str
    ):
        """Build and publish ElectrospinCommand."""
        cmd = ElectrospinCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.target_distance   = float(params.distance_mm)
        cmd.target_scan_speed = float(params.scan_speed)
        cmd.scan_amplitude    = float(params.scan_amplitude)
        cmd.target_rpm        = float(params.rpm)
        cmd.target_flowrate   = float(params.flow_rate)
        cmd.target_voltage    = float(params.voltage_kv) if self.hv_enabled else 0.0
        cmd.collector_enable  = params.rpm > 0
        cmd.pump_enable       = params.flow_rate > 0
        cmd.hv_enable         = self.hv_enabled

        cmd.source            = 1  # AI_AUTO
        cmd.confidence        = float(np.clip(confidence, 0.0, 1.0))
        cmd.rationale         = rationale[:200]
        cmd.immediate         = False

        self.pub_command.publish(cmd)

    def _publish_status(self, quality: FiberQuality, rationale: str):
        """Publish AI controller status as JSON."""
        history = list(self.session_quality_history)
        status = {
            "mode":             self.opt_mode.name,
            "autonomous":       self.autonomous_mode,
            "cycle":            self.opt_cycle_count,
            "current_quality":  round(float(quality.overall_quality), 3),
            "quality_target":   self.quality_target,
            "quality_trend":    round(float(np.mean(history[-10:])), 3) if history else 0.0,
            "quality_grade":    int(quality.quality_grade),
            "diagnosis":        quality.diagnosis,
            "params": {
                "distance_mm":    round(self.smoothed_params.distance_mm, 1),
                "rpm":            round(self.smoothed_params.rpm, 0),
                "flow_rate":      round(self.smoothed_params.flow_rate, 2),
                "voltage_kv":     round(self.smoothed_params.voltage_kv, 1),
                "scan_speed":     round(self.smoothed_params.scan_speed, 1),
                "scan_amplitude": round(self.smoothed_params.scan_amplitude, 1),
            },
            "rationale": rationale,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_ai_status.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AIControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()