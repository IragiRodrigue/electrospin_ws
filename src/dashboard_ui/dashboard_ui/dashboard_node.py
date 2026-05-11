#!/usr/bin/env python3
"""
ElectroSpin Industrial Dashboard UI
=====================================
Professional PyQt5-based industrial control interface for the
autonomous nanofiber fabrication platform.

Real-time displays:
  - 6-DOF robot arm joint visualization
  - Teleoperation skeleton overlay
  - Collector RPM gauge + vibration
  - Syringe pump flow + pressure
  - AI decision parameters
  - Motion command tracking
  - Live trend graphs for all metrics
  - Camera feed with quality overlay
  - Emergency stop + mode switch

Author: ElectroSpin Platform
"""

import sys
import json
import math
import time
import threading
from collections import deque
from typing import Optional, Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QGroupBox, QFrame, QProgressBar,
    QTabWidget, QSlider, QComboBox, QCheckBox, QSplitter, QStatusBar,
    QSizePolicy, QScrollArea, QDial, QLCDNumber
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QSize
from PyQt5.QtGui import (
    QFont, QColor, QPalette, QPainter, QPen, QBrush,
    QLinearGradient, QRadialGradient, QConicalGradient, QImage, QPixmap
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image, JointState
from electrospin_interfaces.msg import (
    FiberQuality, CollectorStatus, ElectrospinCommand,
    SystemStatus, HumanPose, HandGesture, MotionCommand
)

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Color Palette — Industrial Dark Theme
# ─────────────────────────────────────────────────────────────────────────────

class Colors:
    BG_DARK       = "#0d1117"
    BG_PANEL      = "#161b22"
    BG_CARD       = "#1c2333"
    BG_INPUT      = "#21262d"
    BORDER        = "#30363d"
    TEXT_PRIMARY   = "#e6edf3"
    TEXT_SECONDARY = "#8b949e"
    TEXT_MUTED     = "#484f58"
    ACCENT_BLUE   = "#58a6ff"
    ACCENT_GREEN  = "#3fb950"
    ACCENT_YELLOW = "#d29922"
    ACCENT_RED    = "#f85149"
    ACCENT_CYAN   = "#39d2c0"
    ACCENT_ORANGE = "#d18616"
    GAUGE_BG      = "#21262d"
    SUCCESS       = "#3fb950"
    WARNING       = "#d29922"
    ERROR         = "#f85149"
    ESTOP_RED     = "#da3633"
    SKELETON_CLR  = "#58a6ff"
    JOINT_CLR     = "#3fb950"


# ─────────────────────────────────────────────────────────────────────────────
# Custom Widgets
# ─────────────────────────────────────────────────────────────────────────────

class CircularGauge(QWidget):
    """Industrial circular gauge widget."""

    def __init__(self, title: str, unit: str = "", min_val: float = 0,
                 max_val: float = 100, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.value = 0.0
        self.target = 0.0
        self.color = QColor(Colors.ACCENT_BLUE)
        self.setMinimumSize(120, 120)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def set_value(self, val: float):
        self.value = val
        self.update()

    def set_target(self, val: float):
        self.target = val
        self.update()

    def set_color(self, color: str):
        self.color = QColor(color)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 10

        # Background arc
        pen = QPen(QColor(Colors.GAUGE_BG), 8, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        rect = max(0, cx - radius), max(0, cy - radius), radius * 2, radius * 2
        painter.drawArc(*rect, 30 * 16, 300 * 16)

        # Value arc
        frac = (self.value - self.min_val) / (self.max_val - self.min_val + 1e-6)
        frac = max(0.0, min(1.0, frac))
        pen = QPen(self.color, 8, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        span = int(frac * 300 * 16)
        painter.drawArc(*rect, 30 * 16, span)

        # Target tick
        if self.target > 0:
            tfrac = (self.target - self.min_val) / (self.max_val - self.min_val + 1e-6)
            tfrac = max(0.0, min(1.0, tfrac))
            angle_deg = 30 + tfrac * 300
            angle_rad = math.radians(angle_deg)
            tx = cx + int((radius + 2) * math.cos(angle_rad))
            ty = cy - int((radius + 2) * math.sin(angle_rad))
            painter.setPen(QPen(QColor(Colors.ACCENT_ORANGE), 3))
            painter.drawPoint(tx, ty)

        # Value text
        painter.setPen(QColor(Colors.TEXT_PRIMARY))
        font = QFont("JetBrains Mono", 14, QFont.Bold)
        painter.setFont(font)
        text = f"{self.value:.0f}"
        painter.drawText(self.rect(), Qt.AlignCenter, text)

        # Unit
        font = QFont("JetBrains Mono", 8)
        painter.setFont(font)
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        unit_rect = max(0, cx - 30), cy + 12, 60, 20
        painter.drawText(*unit_rect, Qt.AlignCenter, self.unit)

        # Title
        font = QFont("JetBrains Mono", 8)
        painter.setFont(font)
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        title_rect = max(0, cx - 40), h - 22, 80, 20
        painter.drawText(*title_rect, Qt.AlignCenter, self.title)

        painter.end()


class QualityBar(QWidget):
    """Horizontal quality indicator bar."""

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.label = label
        self.value = 0.0
        self.setMinimumWidth(200)
        self.setFixedHeight(24)

    def set_value(self, val: float):
        self.value = max(0.0, min(1.0, val))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        bar_h = 8
        y = (h - bar_h) // 2

        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(0, 0, 80, h, Qt.AlignVCenter | Qt.AlignLeft, self.label)

        painter.setBrush(QColor(Colors.GAUGE_BG))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(85, y, w - 140, bar_h, 4, 4)

        fill_w = int((w - 140) * self.value)
        if self.value > 0.7:
            color = QColor(Colors.SUCCESS)
        elif self.value > 0.4:
            color = QColor(Colors.WARNING)
        else:
            color = QColor(Colors.ERROR)
        painter.setBrush(color)
        painter.drawRoundedRect(85, y, fill_w, bar_h, 4, 4)

        painter.setPen(QColor(Colors.TEXT_PRIMARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(w - 50, 0, 50, h, Qt.AlignVCenter | Qt.AlignRight,
                         f"{self.value:.2f}")
        painter.end()


class TrendGraph(QWidget):
    """Real-time trend line graph with multiple series."""

    def __init__(self, title: str, max_points: int = 200, parent=None):
        super().__init__(parent)
        self.title = title
        self.series: Dict[str, deque] = {}
        self.colors: Dict[str, str] = {}
        self.max_points = max_points
        self.setMinimumSize(300, 100)

    def add_series(self, name: str, color: str):
        self.series[name] = deque(maxlen=self.max_points)
        self.colors[name] = color

    def add_point(self, series_name: str, val: float):
        if series_name in self.series:
            self.series[series_name].append(val)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        margin = 30
        plot_w = w - margin * 2
        plot_h = h - margin * 2

        painter.setBrush(QColor(Colors.BG_CARD))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, w, h, 6, 6)

        pen = QPen(QColor(Colors.BORDER), 1, Qt.DotLine)
        painter.setPen(pen)
        for i in range(5):
            y = margin + int(plot_h * i / 4)
            painter.drawLine(margin, y, margin + plot_w, y)

        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(margin, 15, self.title)

        for name, data in self.series.items():
            if len(data) < 2:
                continue
            color = self.colors.get(name, Colors.ACCENT_CYAN)
            pen = QPen(QColor(color), 2, Qt.SolidLine)
            painter.setPen(pen)
            points = []
            for i, val in enumerate(data):
                x = margin + int(plot_w * i / (len(data) - 1))
                y = margin + int(plot_h * (1.0 - max(0.0, min(1.0, val))))
                points.append((x, y))
            for i in range(len(points) - 1):
                painter.drawLine(points[i][0], points[i][1],
                                 points[i+1][0], points[i+1][1])

        painter.setPen(QColor(Colors.TEXT_MUTED))
        painter.setFont(QFont("JetBrains Mono", 7))
        for i, val in enumerate([1.0, 0.75, 0.5, 0.25, 0.0]):
            y = margin + int(plot_h * i / 4)
            painter.drawText(2, y + 4, f"{val:.1f}")

        # Legend
        lx = margin + 5
        ly = h - 14
        painter.setFont(QFont("JetBrains Mono", 7))
        for name in self.series:
            color = self.colors.get(name, Colors.ACCENT_CYAN)
            painter.setPen(QColor(color))
            painter.drawText(lx, ly, name)
            lx += len(name) * 7 + 12

        painter.end()


class RobotArmWidget(QWidget):
    """2D side-view visualization of the MyCobot 6-DOF arm."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.joint_angles = [0.0] * 6  # radians
        self.target_angles = [0.0] * 6
        self.setMinimumSize(280, 260)

    def set_angles(self, angles_rad):
        self.joint_angles = list(angles_rad)
        self.update()

    def set_targets(self, angles_rad):
        self.target_angles = list(angles_rad)
        self.update()

    def _fk_2d(self, angles):
        """Simplified 2D forward kinematics for visualization."""
        j1, j2, j3, j4, j5, j6 = angles
        # Link lengths (pixels, scaled)
        L = [0, 50, 70, 70, 45, 35, 25]
        # Base at bottom center
        bx, by = 140, 230
        # Joint 1 = base rotation (shown as horizontal offset)
        x_off = math.sin(j1) * 20
        # Build chain upward
        pts = [(bx + x_off, by)]
        cum_angle = 0.0
        for i in range(1, 7):
            if i == 1:
                cum_angle = j2
            elif i == 2:
                cum_angle = j2 + j3
            elif i == 3:
                cum_angle = j2 + j3 + j4
            elif i == 4:
                cum_angle = j2 + j3 + j4 + j5
            elif i == 5:
                cum_angle = j2 + j3 + j4 + j5 + j6
            else:
                cum_angle = j2 + j3 + j4 + j5 + j6
            dx = L[i] * math.sin(cum_angle)
            dy = -L[i] * math.cos(cum_angle)
            prev = pts[-1]
            pts.append((prev[0] + dx, prev[1] + dy))
        return pts

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        painter.setBrush(QColor(Colors.BG_CARD))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, w, h, 6, 6)

        # Draw target arm (ghost)
        target_pts = self._fk_2d(self.target_angles)
        pen = QPen(QColor(Colors.ACCENT_ORANGE), 2, Qt.DashLine)
        painter.setPen(pen)
        for i in range(len(target_pts) - 1):
            painter.drawLine(int(target_pts[i][0]), int(target_pts[i][1]),
                              int(target_pts[i+1][0]), int(target_pts[i+1][1]))

        # Draw current arm
        pts = self._fk_2d(self.joint_angles)
        pen = QPen(QColor(Colors.ACCENT_BLUE), 3, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        for i in range(len(pts) - 1):
            painter.drawLine(int(pts[i][0]), int(pts[i][1]),
                              int(pts[i+1][0]), int(pts[i+1][1]))

        # Draw joints
        for i, (x, y) in enumerate(pts):
            r = 6 if i > 0 else 8
            color = QColor(Colors.JOINT_CLR) if i > 0 else QColor(Colors.TEXT_MUTED)
            painter.setBrush(color)
            painter.setPen(QPen(QColor(Colors.BG_DARK), 1))
            painter.drawEllipse(int(x) - r, int(y) - r, r * 2, r * 2)

        # End-effector label
        if pts:
            ex, ey = pts[-1]
            painter.setPen(QColor(Colors.ACCENT_CYAN))
            painter.setFont(QFont("JetBrains Mono", 7))
            painter.drawText(int(ex) + 8, int(ey) - 4, "EE")

        # Joint angle readouts
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 7))
        for i, angle in enumerate(self.joint_angles):
            deg = math.degrees(angle)
            painter.drawText(5, 15 + i * 13, f"J{i+1}: {deg:+6.1f} deg")

        painter.end()


class SkeletonWidget(QWidget):
    """2D visualization of tracked human skeleton for teleoperation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pose_data: Optional[Dict] = None
        self.gesture_name = "none"
        self.tracking = False
        self.setMinimumSize(280, 220)

    def set_pose(self, data: dict):
        self.pose_data = data
        self.tracking = data.get("person_detected", False)
        self.update()

    def set_gesture(self, name: str):
        self.gesture_name = name
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        painter.setBrush(QColor(Colors.BG_CARD))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, w, h, 6, 6)

        if self.pose_data is None or not self.tracking:
            painter.setPen(QColor(Colors.TEXT_MUTED))
            painter.setFont(QFont("JetBrains Mono", 10))
            painter.drawText(self.rect(), Qt.AlignCenter, "No person detected")
            painter.end()
            return

        # Map normalized coords to widget
        def to_px(norm_val, axis):
            if axis == "x":
                return int(norm_val * w)
            return int(norm_val * h)

        pd = self.pose_data

        # Draw skeleton connections
        connections = [
            ("left_shoulder", "right_shoulder"),
            ("left_shoulder", "left_elbow"),
            ("right_shoulder", "right_elbow"),
            ("left_elbow", "left_wrist"),
            ("right_elbow", "right_wrist"),
        ]

        pen = QPen(QColor(Colors.SKELETON_CLR), 3, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)

        for start_key, end_key in connections:
            s = pd.get(start_key)
            e = pd.get(end_key)
            if s and e and s.get("vis", 0) > 0.3 and e.get("vis", 0) > 0.3:
                painter.drawLine(
                    to_px(s["x"], "x"), to_px(s["y"], "y"),
                    to_px(e["x"], "x"), to_px(e["y"], "y")
                )

        # Draw joints
        joint_keys = [
            "left_shoulder", "right_shoulder",
            "left_elbow", "right_elbow",
            "left_wrist", "right_wrist",
        ]
        for key in joint_keys:
            j = pd.get(key)
            if j and j.get("vis", 0) > 0.3:
                painter.setBrush(QColor(Colors.JOINT_CLR))
                painter.setPen(QPen(QColor(Colors.BG_DARK), 1))
                px, py = to_px(j["x"], "x"), to_px(j["y"], "y")
                painter.drawEllipse(px - 5, py - 5, 10, 10)

        # Gesture label
        painter.setPen(QColor(Colors.ACCENT_CYAN))
        painter.setFont(QFont("JetBrains Mono", 9, QFont.Bold))
        painter.drawText(5, 18, f"Gesture: {self.gesture_name}")

        # Confidence
        conf = pd.get("overall_confidence", 0)
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(5, h - 8, f"Confidence: {conf:.2f}")

        painter.end()


class EStopButton(QPushButton):
    """Industrial emergency stop button."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setText("E-STOP")
        self.setFixedSize(100, 100)
        self.setCheckable(True)
        self._active = False
        self._update_style()

    def set_active(self, active: bool):
        self._active = active
        self.setChecked(active)
        self._update_style()

    def _update_style(self):
        if self._active:
            self.setStyleSheet("""
                QPushButton {
                    background-color: #da3633;
                    color: white;
                    border: 4px solid #f85149;
                    border-radius: 50px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)
        else:
            self.setStyleSheet("""
                QPushButton {
                    background-color: #21262d;
                    color: #f85149;
                    border: 4px solid #f85149;
                    border-radius: 50px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #30363d;
                }
            """)


# ─────────────────────────────────────────────────────────────────────────────
# Dashboard ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class DashboardNode(Node):
    """ROS2 node that bridges the Qt UI with ROS2 topics."""

    quality_updated = pyqtSignal(dict)
    collector_updated = pyqtSignal(dict)
    robot_updated = pyqtSignal(dict)
    ai_updated = pyqtSignal(dict)
    system_updated = pyqtSignal(dict)
    pump_updated = pyqtSignal(dict)
    motion_updated = pyqtSignal(dict)
    pose_updated = pyqtSignal(dict)
    gesture_updated = pyqtSignal(dict)
    image_updated = pyqtSignal(object)
    joint_updated = pyqtSignal(list)
    digital_twin_updated = pyqtSignal(dict)
    maintenance_updated = pyqtSignal(dict)
    twin_camera_updated = pyqtSignal(object)

    def __init__(self):
        super().__init__("dashboard")

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("window_title", "ElectroSpin Control Platform")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # Publishers
        self.pub_target_rpm = self.create_publisher(Float32, "/target_rpm", reliable_qos)
        self.pub_target_flow = self.create_publisher(Float32, "/target_flowrate", reliable_qos)
        self.pub_estop = self.create_publisher(Bool, "/emergency_stop", reliable_qos)
        self.pub_manual_override = self.create_publisher(Bool, "/manual_override", reliable_qos)

        # Subscribers — core system
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
        self.sub_system = self.create_subscription(
            SystemStatus, "/system_status", self._on_system, reliable_qos
        )
        self.sub_pump = self.create_subscription(
            String, "/pump_status", self._on_pump, reliable_qos
        )
        self.sub_image = self.create_subscription(
            Image, "/vision_debug", self._on_image, sensor_qos
        )
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._on_joints, sensor_qos
        )

        # Subscribers — teleoperation
        self.sub_pose = self.create_subscription(
            HumanPose, "/human_pose", self._on_pose, sensor_qos
        )
        self.sub_gesture = self.create_subscription(
            HandGesture, "/hand_gesture", self._on_gesture, reliable_qos
        )
        self.sub_motion = self.create_subscription(
            MotionCommand, "/motion_command", self._on_motion, reliable_qos
        )

        # Subscribers — digital twin
        self.sub_hv_status = self.create_subscription(
            String, "/hv/status", self._on_hv_status, reliable_qos
        )
        self.sub_env_temp = self.create_subscription(
            Float32, "/env/temperature", self._on_env_temp, reliable_qos
        )
        self.sub_env_humidity = self.create_subscription(
            Float32, "/env/humidity", self._on_env_humidity, reliable_qos
        )
        self.sub_deposition = self.create_subscription(
            String, "/fiber_deposition", self._on_deposition, reliable_qos
        )
        self.sub_pump_twin = self.create_subscription(
            String, "/pump/status", self._on_pump_twin, reliable_qos
        )
        self.sub_collector_twin = self.create_subscription(
            String, "/collector/state", self._on_collector_twin, reliable_qos
        )
        self.sub_maintenance = self.create_subscription(
            String, "/maintenance/status", self._on_maintenance, reliable_qos
        )
        self.sub_maintenance_alerts = self.create_subscription(
            String, "/maintenance/alerts", self._on_maintenance_alerts, reliable_qos
        )
        self.sub_twin_camera = self.create_subscription(
            Image, "/digital_twin/camera_debug", self._on_twin_camera, sensor_qos
        )

        # Digital twin state cache
        self._dt_state = {
            "hv_voltage_kv": 0.0, "hv_enabled": False, "hv_current_ua": 0.0,
            "env_temp": 22.0, "env_humidity": 45.0,
            "dep_coverage": 0.0, "dep_total_mg": 0.0,
            "pump_roller_rpm": 0.0,
            "collector_vibration": 0.0, "collector_temp": 25.0,
            "pump_tubing_wear": 0.0, "pump_roller_wear": 0.0,
            "pump_motor_temp_c": 25.0,
            "collector_bearing_wear": 0.0, "collector_belt_wear": 0.0,
            "hv_arc_count": 0, "hv_insulation_wear": 0.0,
            "health_score": 1.0,
        }
        self._maintenance_data = {}
        self._maintenance_alerts = []

        self._bridge = CvBridge() if CV_AVAILABLE else None

    def _on_quality(self, msg: FiberQuality):
        self.quality_updated.emit({
            "overall": msg.overall_quality,
            "uniformity": msg.uniformity,
            "diameter": msg.diameter,
            "bead_score": msg.bead_score,
            "cone_score": msg.taylor_cone_score,
            "jet_stable": msg.jet_stable,
            "density": msg.deposition_density,
            "coverage": msg.coverage_uniformity,
            "grade": msg.quality_grade,
            "diagnosis": msg.diagnosis,
        })

    def _on_collector(self, msg: CollectorStatus):
        self.collector_updated.emit({
            "rpm": msg.rpm,
            "target_rpm": msg.target_rpm,
            "running": msg.running,
            "at_setpoint": msg.at_setpoint,
            "vibration": msg.vibration_score,
            "temperature": msg.temperature_c,
            "duty": msg.duty_cycle,
            "estop": msg.emergency_stop,
        })

    def _on_robot(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.robot_updated.emit(data)
        except json.JSONDecodeError:
            pass

    def _on_ai(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.ai_updated.emit(data)
        except json.JSONDecodeError:
            pass

    def _on_system(self, msg: SystemStatus):
        self.system_updated.emit({
            "state": msg.system_state,
            "sim": msg.simulation_mode,
            "uptime": msg.uptime_s,
            "estop": msg.emergency_stop,
            "quality": msg.quality_current,
        })

    def _on_pump(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.pump_updated.emit(data)
        except json.JSONDecodeError:
            pass

    def _on_image(self, msg: Image):
        if self._bridge:
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                self.image_updated.emit(rgb_img)
            except Exception:
                pass

    def _on_joints(self, msg: JointState):
        self.joint_updated.emit(list(msg.position))

    def _on_pose(self, msg: HumanPose):
        self.pose_updated.emit({
            "person_detected": msg.person_detected,
            "overall_confidence": msg.overall_confidence,
            "left_shoulder": {"x": msg.left_shoulder_position[0], "y": msg.left_shoulder_position[1], "vis": msg.left_shoulder_visibility},
            "right_shoulder": {"x": msg.right_shoulder_position[0], "y": msg.right_shoulder_position[1], "vis": msg.right_shoulder_visibility},
            "left_elbow": {"x": msg.left_elbow_position[0], "y": msg.left_elbow_position[1], "vis": msg.left_elbow_visibility},
            "right_elbow": {"x": msg.right_elbow_position[0], "y": msg.right_elbow_position[1], "vis": msg.right_elbow_visibility},
            "left_wrist": {"x": msg.left_wrist_position[0], "y": msg.left_wrist_position[1], "vis": msg.left_wrist_visibility},
            "right_wrist": {"x": msg.right_wrist_position[0], "y": msg.right_wrist_position[1], "vis": msg.right_wrist_visibility},
            "left_shoulder_angle": msg.left_shoulder_angle,
            "left_elbow_angle": msg.left_elbow_angle,
            "right_shoulder_angle": msg.right_shoulder_angle,
            "right_elbow_angle": msg.right_elbow_angle,
        })

    def _on_gesture(self, msg: HandGesture):
        self.gesture_updated.emit({
            "gesture_name": msg.gesture_name,
            "confidence": msg.confidence,
            "left_hand": msg.left_hand,
            "right_hand": msg.right_hand,
            "command": msg.command,
        })

    def _on_motion(self, msg: MotionCommand):
        self.motion_updated.emit({
            "joint_angles": list(msg.target_joint_angles),
            "is_safe": msg.is_safe,
            "confidence": msg.confidence,
            "source": msg.source,
            "latency_ms": msg.latency_ms,
        })

    def _on_hv_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._dt_state["hv_voltage_kv"] = data.get("voltage_kv", 0)
            self._dt_state["hv_enabled"] = data.get("enabled", False)
            self._dt_state["hv_current_ua"] = data.get("current_ua", 0)
            self.digital_twin_updated.emit(dict(self._dt_state))
        except json.JSONDecodeError:
            pass

    def _on_env_temp(self, msg: Float32):
        self._dt_state["env_temp"] = msg.data
        self.digital_twin_updated.emit(dict(self._dt_state))

    def _on_env_humidity(self, msg: Float32):
        self._dt_state["env_humidity"] = msg.data
        self.digital_twin_updated.emit(dict(self._dt_state))

    def _on_deposition(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._dt_state["dep_coverage"] = data.get("coverage_mean", 0)
            self._dt_state["dep_total_mg"] = data.get("total_mg", 0)
            self.digital_twin_updated.emit(dict(self._dt_state))
        except json.JSONDecodeError:
            pass

    def _on_pump_twin(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._dt_state["pump_roller_rpm"] = data.get("roller_rpm", 0)
            self.digital_twin_updated.emit(dict(self._dt_state))
        except json.JSONDecodeError:
            pass

    def _on_collector_twin(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._dt_state["collector_vibration"] = data.get("vibration", 0)
            self._dt_state["collector_temp"] = data.get("temperature_c", 25)
            self._dt_state["collector_bearing_wear"] = data.get("bearing_wear", 0)
            self._dt_state["collector_belt_wear"] = data.get("belt_wear", 0)
            self.digital_twin_updated.emit(dict(self._dt_state))
        except json.JSONDecodeError:
            pass

    def _on_maintenance(self, msg: String):
        try:
            self._maintenance_data = json.loads(msg.data)
            self._dt_state["health_score"] = self._maintenance_data.get("health_score", 1.0)
            self.digital_twin_updated.emit(dict(self._dt_state))
            self.maintenance_updated.emit(self._maintenance_data)
        except json.JSONDecodeError:
            pass

    def _on_maintenance_alerts(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._maintenance_alerts = data.get("alerts", [])
        except json.JSONDecodeError:
            pass

    def _on_twin_camera(self, msg: Image):
        if self._bridge:
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                self.twin_camera_updated.emit(rgb_img)
            except Exception:
                pass

    def publish_target_rpm(self, rpm: float):
        msg = Float32()
        msg.data = float(rpm)
        self.pub_target_rpm.publish(msg)

    def publish_target_flow(self, flow: float):
        msg = Float32()
        msg.data = float(flow)
        self.pub_target_flow.publish(msg)

    def publish_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.pub_estop.publish(msg)

    def publish_manual_override(self, manual: bool):
        msg = Bool()
        msg.data = manual
        self.pub_manual_override.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Main Dashboard Window
# ─────────────────────────────────────────────────────────────────────────────

class DashboardWindow(QMainWindow):
    """Main industrial dashboard window with real-time movement displays."""

    def __init__(self, ros_node: DashboardNode):
        super().__init__()
        self.ros = ros_node
        self._estop_active = False
        self._manual_mode = False

        title = ros_node.get_parameter("window_title").value
        self.setWindowTitle(title)
        self.setMinimumSize(1600, 950)
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {Colors.BG_DARK}; }}
            QWidget {{ color: {Colors.TEXT_PRIMARY}; font-family: 'JetBrains Mono', 'Consolas', monospace; }}
            QGroupBox {{
                background-color: {Colors.BG_PANEL};
                border: 1px solid {Colors.BORDER};
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 16px;
                font-size: 11px;
                font-weight: bold;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: {Colors.ACCENT_BLUE};
            }}
            QLabel {{ font-size: 10px; }}
            QPushButton {{
                background-color: {Colors.BG_INPUT};
                border: 1px solid {Colors.BORDER};
                border-radius: 4px;
                padding: 6px 12px;
                font-size: 10px;
            }}
            QPushButton:hover {{ background-color: {Colors.BORDER}; }}
            QSlider::groove:horizontal {{
                background: {Colors.GAUGE_BG};
                height: 6px;
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {Colors.ACCENT_BLUE};
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }}
            QTabWidget::pane {{ border: 1px solid {Colors.BORDER}; background: {Colors.BG_PANEL}; }}
            QTabBar::tab {{
                background: {Colors.BG_INPUT};
                color: {Colors.TEXT_SECONDARY};
                padding: 6px 14px;
                border: 1px solid {Colors.BORDER};
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background: {Colors.BG_PANEL};
                color: {Colors.ACCENT_BLUE};
            }}
        """)

        self._build_ui()
        self._connect_signals()

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(6, 6, 6, 6)

        # ── Left Panel: Camera + Quality + Trends ────────────────────────────
        left_panel = QVBoxLayout()

        # Camera feed
        cam_group = QGroupBox("Vision System")
        cam_layout = QVBoxLayout(cam_group)
        self.camera_label = QLabel("No camera feed")
        self.camera_label.setMinimumSize(420, 300)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; border: 1px solid {Colors.BORDER}; border-radius: 4px;"
        )
        cam_layout.addWidget(self.camera_label)
        left_panel.addWidget(cam_group)

        # Quality metrics
        quality_group = QGroupBox("Fiber Quality")
        q_layout = QVBoxLayout(quality_group)
        self.q_overall = QualityBar("Overall")
        self.q_uniformity = QualityBar("Uniformity")
        self.q_bead = QualityBar("Bead Score")
        self.q_cone = QualityBar("Cone Score")
        self.q_coverage = QualityBar("Coverage")
        self.q_density = QualityBar("Density")
        for bar in [self.q_overall, self.q_uniformity, self.q_bead,
                    self.q_cone, self.q_coverage, self.q_density]:
            q_layout.addWidget(bar)
        left_panel.addWidget(quality_group)

        # Multi-series trend graphs
        trend_tabs = QTabWidget()

        self.quality_trend = TrendGraph("Quality Trend")
        self.quality_trend.add_series("overall", Colors.ACCENT_CYAN)
        self.quality_trend.add_series("uniformity", Colors.ACCENT_GREEN)
        self.quality_trend.add_series("coverage", Colors.ACCENT_YELLOW)
        trend_tabs.addTab(self.quality_trend, "Quality")

        self.rpm_trend = TrendGraph("RPM Trend")
        self.rpm_trend.add_series("actual", Colors.ACCENT_BLUE)
        self.rpm_trend.add_series("target", Colors.ACCENT_ORANGE)
        trend_tabs.addTab(self.rpm_trend, "RPM")

        self.flow_trend = TrendGraph("Flow Trend")
        self.flow_trend.add_series("actual", Colors.ACCENT_CYAN)
        self.flow_trend.add_series("pressure", Colors.ACCENT_RED)
        trend_tabs.addTab(self.flow_trend, "Flow")

        left_panel.addWidget(trend_tabs)
        left_panel.addStretch()
        main_layout.addLayout(left_panel, stretch=3)

        # ── Center Panel: Gauges + Robot + Controls ───────────────────────────
        center_panel = QVBoxLayout()

        # Gauges row
        gauges_group = QGroupBox("Process Parameters")
        gauges_layout = QHBoxLayout(gauges_group)

        self.rpm_gauge = CircularGauge("RPM", "rpm", 0, 3000)
        self.rpm_gauge.set_color(Colors.ACCENT_BLUE)
        self.quality_gauge = CircularGauge("Quality", "%", 0, 100)
        self.quality_gauge.set_color(Colors.ACCENT_GREEN)
        self.flow_gauge = CircularGauge("Flow", "mL/h", 0, 10)
        self.flow_gauge.set_color(Colors.ACCENT_CYAN)
        self.distance_gauge = CircularGauge("Distance", "mm", 0, 250)
        self.distance_gauge.set_color(Colors.ACCENT_YELLOW)

        gauges_layout.addWidget(self.rpm_gauge)
        gauges_layout.addWidget(self.quality_gauge)
        gauges_layout.addWidget(self.flow_gauge)
        gauges_layout.addWidget(self.distance_gauge)
        center_panel.addWidget(gauges_group)

        # Robot arm + Skeleton side by side
        viz_group = QGroupBox("Real-Time Visualization")
        viz_layout = QHBoxLayout(viz_group)

        self.robot_arm = RobotArmWidget()
        viz_layout.addWidget(self.robot_arm)

        self.skeleton = SkeletonWidget()
        viz_layout.addWidget(self.skeleton)

        center_panel.addWidget(viz_group)

        # Manual controls
        controls_group = QGroupBox("Manual Controls")
        ctrl_layout = QGridLayout(controls_group)

        ctrl_layout.addWidget(QLabel("Target RPM:"), 0, 0)
        self.rpm_slider = QSlider(Qt.Horizontal)
        self.rpm_slider.setRange(0, 3000)
        self.rpm_slider.setValue(500)
        self.rpm_slider.valueChanged.connect(self._on_rpm_changed)
        ctrl_layout.addWidget(self.rpm_slider, 0, 1)
        self.rpm_label = QLabel("500")
        self.rpm_label.setMinimumWidth(50)
        ctrl_layout.addWidget(self.rpm_label, 0, 2)

        ctrl_layout.addWidget(QLabel("Flow Rate:"), 1, 0)
        self.flow_slider = QSlider(Qt.Horizontal)
        self.flow_slider.setRange(0, 1000)
        self.flow_slider.setValue(100)
        self.flow_slider.valueChanged.connect(self._on_flow_changed)
        ctrl_layout.addWidget(self.flow_slider, 1, 1)
        self.flow_label = QLabel("1.00")
        self.flow_label.setMinimumWidth(50)
        ctrl_layout.addWidget(self.flow_label, 1, 2)

        ctrl_layout.addWidget(QLabel("Mode:"), 2, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["AUTONOMOUS", "MANUAL"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        ctrl_layout.addWidget(self.mode_combo, 2, 1)

        center_panel.addWidget(controls_group)

        # AI + Motion status
        status_tabs = QTabWidget()

        ai_group = QWidget()
        ai_layout = QVBoxLayout(ai_group)
        self.ai_mode_label = QLabel("Mode: --")
        self.ai_cycle_label = QLabel("Cycle: --")
        self.ai_quality_label = QLabel("Quality: --")
        self.ai_params_label = QLabel("Params: --")
        self.ai_rationale_label = QLabel("Rationale: --")
        self.ai_rationale_label.setWordWrap(True)
        self.ai_rationale_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; font-size: 9px;"
        )
        for lbl in [self.ai_mode_label, self.ai_cycle_label,
                    self.ai_quality_label, self.ai_params_label,
                    self.ai_rationale_label]:
            ai_layout.addWidget(lbl)
        ai_layout.addStretch()
        status_tabs.addTab(ai_group, "AI")

        motion_group = QWidget()
        motion_layout = QVBoxLayout(motion_group)
        self.motion_angles_label = QLabel("Joint Angles: --")
        self.motion_safe_label = QLabel("Safe: --")
        self.motion_conf_label = QLabel("Confidence: --")
        self.motion_latency_label = QLabel("Latency: --")
        self.motion_source_label = QLabel("Source: --")
        for lbl in [self.motion_angles_label, self.motion_safe_label,
                    self.motion_conf_label, self.motion_latency_label,
                    self.motion_source_label]:
            motion_layout.addWidget(lbl)
        motion_layout.addStretch()
        status_tabs.addTab(motion_group, "Motion")

        pump_group = QWidget()
        pump_layout = QVBoxLayout(pump_group)
        self.pump_flow_label = QLabel("Flow: -- mL/hr")
        self.pump_setpoint_label = QLabel("Setpoint: -- mL/hr")
        self.pump_pressure_label = QLabel("Pressure: -- kPa")
        self.pump_volume_label = QLabel("Volume: -- mL")
        self.pump_status_label = QLabel("Status: --")
        for lbl in [self.pump_flow_label, self.pump_setpoint_label,
                    self.pump_pressure_label, self.pump_volume_label,
                    self.pump_status_label]:
            pump_layout.addWidget(lbl)
        pump_layout.addStretch()
        status_tabs.addTab(pump_group, "Pump")

        center_panel.addWidget(status_tabs)
        center_panel.addStretch()
        main_layout.addLayout(center_panel, stretch=4)

        # ── Right Panel: Safety + System + Collector + Teleop ─────────────────
        right_panel = QVBoxLayout()

        # E-Stop
        safety_group = QGroupBox("Safety")
        safety_layout = QVBoxLayout(safety_group)
        safety_layout.setAlignment(Qt.AlignCenter)
        self.estop_btn = EStopButton()
        self.estop_btn.clicked.connect(self._on_estop)
        safety_layout.addWidget(self.estop_btn, alignment=Qt.AlignCenter)
        self.estop_status = QLabel("E-STOP: CLEAR")
        self.estop_status.setAlignment(Qt.AlignCenter)
        self.estop_status.setStyleSheet(f"color: {Colors.SUCCESS}; font-weight: bold;")
        safety_layout.addWidget(self.estop_status)
        right_panel.addWidget(safety_group)

        # System status
        sys_group = QGroupBox("System")
        sys_layout = QVBoxLayout(sys_group)
        self.sys_state_label = QLabel("State: INIT")
        self.sys_sim_label = QLabel("Mode: SIM")
        self.sys_uptime_label = QLabel("Uptime: 0s")
        self.sys_nodes_label = QLabel("Nodes: --")
        for lbl in [self.sys_state_label, self.sys_sim_label,
                    self.sys_uptime_label, self.sys_nodes_label]:
            sys_layout.addWidget(lbl)
        right_panel.addWidget(sys_group)

        # Collector detail
        coll_group = QGroupBox("Collector Detail")
        coll_layout = QVBoxLayout(coll_group)
        self.coll_running_label = QLabel("Running: --")
        self.coll_setpoint_label = QLabel("At Setpoint: --")
        self.coll_vibration_label = QLabel("Vibration: --")
        self.coll_temp_label = QLabel("Temp: -- C")
        self.coll_duty_label = QLabel("Duty: -- %")
        for lbl in [self.coll_running_label, self.coll_setpoint_label,
                    self.coll_vibration_label, self.coll_temp_label,
                    self.coll_duty_label]:
            coll_layout.addWidget(lbl)
        right_panel.addWidget(coll_group)

        # Teleoperation status
        teleop_group = QGroupBox("Teleoperation")
        teleop_layout = QVBoxLayout(teleop_group)
        self.teleop_gesture_label = QLabel("Gesture: --")
        self.teleop_conf_label = QLabel("Confidence: --")
        self.teleop_l_angle_label = QLabel("L Arm: -- / --")
        self.teleop_r_angle_label = QLabel("R Arm: -- / --")
        self.teleop_tracking_label = QLabel("Tracking: NO")
        self.teleop_tracking_label.setStyleSheet(f"color: {Colors.ERROR}; font-weight: bold;")
        for lbl in [self.teleop_gesture_label, self.teleop_conf_label,
                    self.teleop_l_angle_label, self.teleop_r_angle_label,
                    self.teleop_tracking_label]:
            teleop_layout.addWidget(lbl)
        right_panel.addWidget(teleop_group)

        # Diagnosis
        diag_group = QGroupBox("Diagnosis")
        diag_layout = QVBoxLayout(diag_group)
        self.diag_label = QLabel("nominal")
        self.diag_label.setWordWrap(True)
        self.diag_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; font-size: 10px;"
        )
        diag_layout.addWidget(self.diag_label)
        right_panel.addWidget(diag_group)

        # Digital Twin
        dt_group = QGroupBox("Digital Twin")
        dt_layout = QVBoxLayout(dt_group)
        self.dt_hv_label = QLabel("HV: OFF  0.00 kV")
        self.dt_hv_current_label = QLabel("HV Current: 0.0 uA")
        self.dt_temp_label = QLabel("Temp: 22.0 C")
        self.dt_humidity_label = QLabel("Humidity: 45.0 %")
        self.dt_deposition_label = QLabel("Deposition: 0.0%  0.0 mg")
        self.dt_roller_label = QLabel("Pump Roller: 0.0 RPM")
        self.dt_vibration_label = QLabel("Vibration: 0.00")
        self.dt_collector_temp_label = QLabel("Motor Temp: 25.0 C")
        self.dt_health_label = QLabel("Health: 100%")
        self.dt_health_label.setStyleSheet(f"color: {Colors.SUCCESS}; font-weight: bold;")
        self.dt_pump_wear_label = QLabel("Pump Wear: Tubing 0% / Roller 0%")
        self.dt_coll_wear_label = QLabel("Coll Wear: Bearing 0% / Belt 0%")
        self.dt_hv_wear_label = QLabel("HV Wear: Insulation 0% / Arcs 0")
        for lbl in [self.dt_hv_label, self.dt_hv_current_label,
                    self.dt_temp_label, self.dt_humidity_label,
                    self.dt_deposition_label, self.dt_roller_label,
                    self.dt_vibration_label, self.dt_collector_temp_label,
                    self.dt_health_label, self.dt_pump_wear_label,
                    self.dt_coll_wear_label, self.dt_hv_wear_label]:
            dt_layout.addWidget(lbl)
        right_panel.addWidget(dt_group)

        # Maintenance Alerts
        maint_group = QGroupBox("Maintenance Alerts")
        maint_layout = QVBoxLayout(maint_group)
        self.maint_alert_label = QLabel("No alerts")
        self.maint_alert_label.setWordWrap(True)
        self.maint_alert_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; font-size: 9px;"
        )
        maint_layout.addWidget(self.maint_alert_label)
        right_panel.addWidget(maint_group)

        # Digital Twin Camera
        twin_cam_group = QGroupBox("Twin Camera")
        twin_cam_layout = QVBoxLayout(twin_cam_group)
        self.twin_cam_label = QLabel("No twin camera")
        self.twin_cam_label.setMinimumSize(200, 150)
        self.twin_cam_label.setAlignment(Qt.AlignCenter)
        self.twin_cam_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; border: 1px solid {Colors.BORDER}; border-radius: 4px;"
        )
        twin_cam_layout.addWidget(self.twin_cam_label)
        right_panel.addWidget(twin_cam_group)

        right_panel.addStretch()
        main_layout.addLayout(right_panel, stretch=2)

        # Status bar
        self.statusBar().setStyleSheet(
            f"background-color: {Colors.BG_PANEL}; color: {Colors.TEXT_SECONDARY}; font-size: 9px;"
        )
        self.statusBar().showMessage("ElectroSpin Platform — Initializing...")

    def _connect_signals(self):
        self.ros.quality_updated.connect(self._update_quality)
        self.ros.collector_updated.connect(self._update_collector)
        self.ros.robot_updated.connect(self._update_robot)
        self.ros.ai_updated.connect(self._update_ai)
        self.ros.system_updated.connect(self._update_system)
        self.ros.pump_updated.connect(self._update_pump)
        self.ros.pose_updated.connect(self._update_pose)
        self.ros.gesture_updated.connect(self._update_gesture)
        self.ros.motion_updated.connect(self._update_motion)
        self.ros.image_updated.connect(self._update_image)
        self.ros.joint_updated.connect(self._update_joints)
        self.ros.digital_twin_updated.connect(self._update_digital_twin)
        self.ros.maintenance_updated.connect(self._update_maintenance)
        self.ros.twin_camera_updated.connect(self._update_twin_camera)

    # ── Slot handlers ────────────────────────────────────────────────────────

    def _update_quality(self, data: dict):
        overall = data.get("overall", 0)
        self.q_overall.set_value(overall)
        self.q_uniformity.set_value(data.get("uniformity", 0))
        self.q_bead.set_value(data.get("bead_score", 0))
        self.q_cone.set_value(data.get("cone_score", 0))
        self.q_coverage.set_value(data.get("coverage", 0))
        self.q_density.set_value(data.get("density", 0))
        self.quality_gauge.set_value(overall * 100)
        self.quality_trend.add_point("overall", overall)
        self.quality_trend.add_point("uniformity", data.get("uniformity", 0))
        self.quality_trend.add_point("coverage", data.get("coverage", 0))
        self.diag_label.setText(data.get("diagnosis", "nominal"))
        grade = data.get("grade", 0)
        grade_names = ["UNUSABLE", "POOR", "FAIR", "GOOD", "EXCELLENT"]
        color = [Colors.ERROR, Colors.ERROR, Colors.WARNING,
                 Colors.SUCCESS, Colors.SUCCESS][min(grade, 4)]
        self.diag_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; "
            f"font-size: 10px; color: {color};"
        )

    def _update_collector(self, data: dict):
        rpm = data.get("rpm", 0)
        target = data.get("target_rpm", 0)
        self.rpm_gauge.set_value(rpm)
        self.rpm_gauge.set_target(target)
        self.coll_running_label.setText(f"Running: {'YES' if data.get('running') else 'NO'}")
        self.coll_setpoint_label.setText(f"At Setpoint: {'YES' if data.get('at_setpoint') else 'NO'}")
        vib = data.get("vibration", 0)
        vib_color = Colors.SUCCESS if vib < 0.3 else (Colors.WARNING if vib < 0.6 else Colors.ERROR)
        self.coll_vibration_label.setText(f"Vibration: {vib:.2f}")
        self.coll_vibration_label.setStyleSheet(f"color: {vib_color};")
        self.coll_temp_label.setText(f"Temp: {data.get('temperature', 0):.1f} C")
        self.coll_duty_label.setText(f"Duty: {data.get('duty', 0) * 100:.0f} %")
        self.rpm_trend.add_point("actual", rpm / 3000.0)
        self.rpm_trend.add_point("target", target / 3000.0)

    def _update_robot(self, data: dict):
        self.distance_gauge.set_value(data.get("distance_mm", 0))
        angles = data.get("joint_angles", [])
        if angles:
            self.robot_arm.set_angles([math.radians(a) for a in angles])

    def _update_ai(self, data: dict):
        self.ai_mode_label.setText(f"Mode: {data.get('mode', '--')}")
        self.ai_cycle_label.setText(f"Cycle: {data.get('cycle', 0)}")
        self.ai_quality_label.setText(f"Quality: {data.get('current_quality', 0):.3f}")
        params = data.get("params", {})
        if params:
            self.ai_params_label.setText(
                f"Dist={params.get('distance_mm', 0):.0f}mm "
                f"RPM={params.get('rpm', 0):.0f} "
                f"Flow={params.get('flow_rate', 0):.2f}"
            )
        self.ai_rationale_label.setText(data.get("rationale", "--"))

    def _update_system(self, data: dict):
        states = ["OFF", "INIT", "READY", "RUNNING", "ERROR", "E-STOP"]
        state = states[data.get("state", 0)]
        self.sys_state_label.setText(f"State: {state}")
        self.sys_sim_label.setText(f"Mode: {'SIM' if data.get('sim') else 'REAL'}")
        uptime = data.get("uptime", 0)
        self.sys_uptime_label.setText(f"Uptime: {uptime:.0f}s")

    def _update_pump(self, data: dict):
        flow = data.get("actual_flow_ml_hr", 0)
        pressure = data.get("pressure_kpa", 0)
        self.flow_gauge.set_value(flow)
        self.pump_flow_label.setText(f"Flow: {flow:.3f} mL/hr")
        self.pump_setpoint_label.setText(f"Setpoint: {data.get('setpoint_ml_hr', 0):.3f} mL/hr")
        self.pump_pressure_label.setText(f"Pressure: {pressure:.1f} kPa")
        self.pump_volume_label.setText(f"Volume: {data.get('volume_remaining_ml', 0):.1f} mL")
        self.pump_status_label.setText(f"Status: {'RUNNING' if data.get('pump_running') else 'STOPPED'}")
        self.flow_trend.add_point("actual", flow / 10.0)
        self.flow_trend.add_point("pressure", min(1.0, pressure / 50.0))

    def _update_pose(self, data: dict):
        self.skeleton.set_pose(data)
        tracking = data.get("person_detected", False)
        self.teleop_tracking_label.setText(f"Tracking: {'YES' if tracking else 'NO'}")
        self.teleop_tracking_label.setStyleSheet(
            f"color: {Colors.SUCCESS if tracking else Colors.ERROR}; font-weight: bold;"
        )
        self.teleop_conf_label.setText(f"Confidence: {data.get('overall_confidence', 0):.2f}")
        la = data.get("left_shoulder_angle", 0)
        le = data.get("left_elbow_angle", 0)
        ra = data.get("right_shoulder_angle", 0)
        re = data.get("right_elbow_angle", 0)
        self.teleop_l_angle_label.setText(f"L Arm: {math.degrees(la):.0f} / {math.degrees(le):.0f} deg")
        self.teleop_r_angle_label.setText(f"R Arm: {math.degrees(ra):.0f} / {math.degrees(re):.0f} deg")

    def _update_gesture(self, data: dict):
        self.skeleton.set_gesture(data.get("gesture_name", "none"))
        self.teleop_gesture_label.setText(f"Gesture: {data.get('gesture_name', 'none')}")

    def _update_motion(self, data: dict):
        angles = data.get("joint_angles", [])
        if angles:
            self.robot_arm.set_targets(angles)
            deg_str = " / ".join(f"{math.degrees(a):+.0f}" for a in angles)
            self.motion_angles_label.setText(f"Angles: {deg_str} deg")
        self.motion_safe_label.setText(f"Safe: {'YES' if data.get('is_safe') else 'NO'}")
        self.motion_safe_label.setStyleSheet(
            f"color: {Colors.SUCCESS if data.get('is_safe') else Colors.ERROR};"
        )
        self.motion_conf_label.setText(f"Confidence: {data.get('confidence', 0):.2f}")
        self.motion_latency_label.setText(f"Latency: {data.get('latency_ms', 0):.1f} ms")
        sources = ["TELEOP_L", "TELEOP_R", "GESTURE", "AUTO"]
        src = sources[data.get("source", 0)] if data.get("source", 0) < len(sources) else "--"
        self.motion_source_label.setText(f"Source: {src}")

    def _update_image(self, img):
        if img is not None:
            h, w, ch = img.shape
            q_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.camera_label.setPixmap(pixmap)

    def _update_joints(self, angles: list):
        if angles:
            self.robot_arm.set_angles(angles)

    def _update_digital_twin(self, data: dict):
        hv_v = data.get("hv_voltage_kv", 0)
        hv_on = data.get("hv_enabled", False)
        hv_i = data.get("hv_current_ua", 0)
        self.dt_hv_label.setText(f"HV: {'ON' if hv_on else 'OFF'}  {hv_v:.2f} kV")
        self.dt_hv_label.setStyleSheet(
            f"color: {Colors.ACCENT_YELLOW if hv_on else Colors.TEXT_SECONDARY}; font-weight: bold;"
        )
        self.dt_hv_current_label.setText(f"HV Current: {hv_i:.1f} uA")
        temp = data.get("env_temp", 22.0)
        humid = data.get("env_humidity", 45.0)
        self.dt_temp_label.setText(f"Temp: {temp:.1f} C")
        self.dt_humidity_label.setText(f"Humidity: {humid:.1f} %")
        dep_cov = data.get("dep_coverage", 0)
        dep_mg = data.get("dep_total_mg", 0)
        self.dt_deposition_label.setText(f"Deposition: {dep_cov*100:.1f}%  {dep_mg:.1f} mg")
        roller = data.get("pump_roller_rpm", 0)
        self.dt_roller_label.setText(f"Pump Roller: {roller:.1f} RPM")
        vib = data.get("collector_vibration", 0)
        ct = data.get("collector_temp", 25.0)
        vib_color = Colors.SUCCESS if vib < 0.3 else (Colors.WARNING if vib < 0.6 else Colors.ERROR)
        self.dt_vibration_label.setText(f"Vibration: {vib:.3f}")
        self.dt_vibration_label.setStyleSheet(f"color: {vib_color};")
        self.dt_collector_temp_label.setText(f"Motor Temp: {ct:.1f} C")

        # Health score
        health = data.get("health_score", 1.0)
        health_color = Colors.SUCCESS if health > 0.6 else (Colors.WARNING if health > 0.3 else Colors.ERROR)
        self.dt_health_label.setText(f"Health: {health*100:.0f}%")
        self.dt_health_label.setStyleSheet(f"color: {health_color}; font-weight: bold;")

        # Wear indicators
        ptw = data.get("pump_tubing_wear", 0)
        prw = data.get("pump_roller_wear", 0)
        self.dt_pump_wear_label.setText(f"Pump Wear: Tub {ptw*100:.0f}% / Roller {prw*100:.0f}%")
        cbw = data.get("collector_bearing_wear", 0)
        cbtw = data.get("collector_belt_wear", 0)
        self.dt_coll_wear_label.setText(f"Coll Wear: Bearing {cbw*100:.0f}% / Belt {cbtw*100:.0f}%")
        hiw = data.get("hv_insulation_wear", 0)
        hac = data.get("hv_arc_count", 0)
        self.dt_hv_wear_label.setText(f"HV Wear: Insul {hiw*100:.0f}% / Arcs {hac}")

    def _update_maintenance(self, data: dict):
        alerts = data.get("alerts", [])
        if alerts:
            alert_text = "\n".join(alerts[:5])
            self.maint_alert_label.setText(alert_text)
            self.maint_alert_label.setStyleSheet(
                f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; "
                f"font-size: 9px; color: {Colors.ERROR};"
            )
        else:
            self.maint_alert_label.setText("No alerts")
            self.maint_alert_label.setStyleSheet(
                f"background-color: {Colors.BG_CARD}; padding: 6px; border-radius: 4px; "
                f"font-size: 9px; color: {Colors.SUCCESS};"
            )

    def _update_twin_camera(self, img):
        if img is not None:
            h, w, ch = img.shape
            q_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                self.twin_cam_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.twin_cam_label.setPixmap(pixmap)

    # ── Control handlers ─────────────────────────────────────────────────────

    def _on_rpm_changed(self, val):
        self.rpm_label.setText(str(val))
        if self._manual_mode:
            self.ros.publish_target_rpm(float(val))

    def _on_flow_changed(self, val):
        flow = val / 100.0
        self.flow_label.setText(f"{flow:.2f}")
        if self._manual_mode:
            self.ros.publish_target_flow(flow)

    def _on_mode_changed(self, idx):
        self._manual_mode = (idx == 1)
        self.ros.publish_manual_override(self._manual_mode)
        mode_str = "MANUAL" if self._manual_mode else "AUTONOMOUS"
        self.statusBar().showMessage(f"Mode switched to {mode_str}")

    def _on_estop(self):
        self._estop_active = not self._estop_active
        self.estop_btn.set_active(self._estop_active)
        self.ros.publish_estop(self._estop_active)
        if self._estop_active:
            self.estop_status.setText("E-STOP: ACTIVE")
            self.estop_status.setStyleSheet(f"color: {Colors.ERROR}; font-weight: bold;")
            self.statusBar().showMessage("EMERGENCY STOP ACTIVATED")
        else:
            self.estop_status.setText("E-STOP: CLEAR")
            self.estop_status.setStyleSheet(f"color: {Colors.SUCCESS}; font-weight: bold;")
            self.statusBar().showMessage("Emergency stop cleared")

    def _tick(self):
        self.statusBar().showMessage(
            f"ElectroSpin Platform — "
            f"{'SIMULATION' if self.ros.get_parameter('simulation_mode').value else 'HARDWARE'} | "
            f"{'MANUAL' if self._manual_mode else 'AUTONOMOUS'} | "
            f"{'E-STOP' if self._estop_active else 'OK'}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    import os

    # Fix Qt plugin conflict between OpenCV and PyQt5
    # Remove OpenCV's Qt plugin path so PyQt5 uses its own
    cv_plugin_path = os.path.join(
        os.path.dirname(__file__), '..', '..', '..',
        'cv2', 'qt', 'plugins'
    )
    current_plugin_path = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '')
    if cv_plugin_path in current_plugin_path:
        paths = [p for p in current_plugin_path.split(os.pathsep) if cv_plugin_path not in p]
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.pathsep.join(paths)

    # Allow headless mode with offscreen rendering
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'

    rclpy.init(args=args)
    ros_node = DashboardNode()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(Colors.BG_DARK))
    palette.setColor(QPalette.WindowText, QColor(Colors.TEXT_PRIMARY))
    palette.setColor(QPalette.Base, QColor(Colors.BG_CARD))
    palette.setColor(QPalette.AlternateBase, QColor(Colors.BG_PANEL))
    palette.setColor(QPalette.ToolTipBase, QColor(Colors.BG_CARD))
    palette.setColor(QPalette.ToolTipText, QColor(Colors.TEXT_PRIMARY))
    palette.setColor(QPalette.Text, QColor(Colors.TEXT_PRIMARY))
    palette.setColor(QPalette.Button, QColor(Colors.BG_INPUT))
    palette.setColor(QPalette.ButtonText, QColor(Colors.TEXT_PRIMARY))
    palette.setColor(QPalette.Highlight, QColor(Colors.ACCENT_BLUE))
    palette.setColor(QPalette.HighlightedText, QColor(Colors.BG_DARK))
    app.setPalette(palette)

    window = DashboardWindow(ros_node)
    window.show()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    ret = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
