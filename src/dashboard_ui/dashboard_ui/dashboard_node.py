#!/usr/bin/env python3
"""
ElectroSpin Industrial Dashboard UI
=====================================
Professional PyQt5-based industrial control interface for the
autonomous nanofiber fabrication platform.

Features:
  - Live camera feed with quality overlay
  - Real-time collector RPM gauge
  - Robot joint state monitor
  - AI decision visualization
  - System health dashboard
  - Emergency stop button
  - Manual/Autonomous mode switch
  - Parameter adjustment panels
  - Quality trend graphs

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
    QLinearGradient, QRadialGradient, QConicalGradient
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image
from electrospin_interfaces.msg import (
    FiberQuality, CollectorStatus, ElectrospinCommand, SystemStatus
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
    GAUGE_BG      = "#21262d"
    GAUGE_FILL    = "#58a6ff"
    SUCCESS       = "#3fb950"
    WARNING       = "#d29922"
    ERROR         = "#f85149"
    ESTOP_RED     = "#da3633"


# ─────────────────────────────────────────────────────────────────────────────
# Custom Widgets
# ─────────────────────────────────────────────────────────────────────────────

class CircularGauge(QWidget):
    """Industrial circular gauge widget for RPM, quality, etc."""

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

        # Label
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(0, 0, 80, h, Qt.AlignVCenter | Qt.AlignLeft, self.label)

        # Background
        painter.setBrush(QColor(Colors.GAUGE_BG))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(85, y, w - 140, bar_h, 4, 4)

        # Fill
        fill_w = int((w - 140) * self.value)
        if self.value > 0.7:
            color = QColor(Colors.SUCCESS)
        elif self.value > 0.4:
            color = QColor(Colors.WARNING)
        else:
            color = QColor(Colors.ERROR)
        painter.setBrush(color)
        painter.drawRoundedRect(85, y, fill_w, bar_h, 4, 4)

        # Value text
        painter.setPen(QColor(Colors.TEXT_PRIMARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(w - 50, 0, 50, h, Qt.AlignVCenter | Qt.AlignRight,
                         f"{self.value:.2f}")

        painter.end()


class TrendGraph(QWidget):
    """Simple real-time trend line graph."""

    def __init__(self, title: str, max_points: int = 200, parent=None):
        super().__init__(parent)
        self.title = title
        self.data = deque(maxlen=max_points)
        self.setMinimumSize(300, 100)
        self.color = QColor(Colors.ACCENT_CYAN)

    def add_point(self, val: float):
        self.data.append(val)
        self.update()

    def set_color(self, color: str):
        self.color = QColor(color)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        margin = 30
        plot_w = w - margin * 2
        plot_h = h - margin * 2

        # Background
        painter.setBrush(QColor(Colors.BG_CARD))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, w, h, 6, 6)

        # Grid lines
        pen = QPen(QColor(Colors.BORDER), 1, Qt.DotLine)
        painter.setPen(pen)
        for i in range(5):
            y = margin + int(plot_h * i / 4)
            painter.drawLine(margin, y, margin + plot_w, y)

        # Title
        painter.setPen(QColor(Colors.TEXT_SECONDARY))
        painter.setFont(QFont("JetBrains Mono", 8))
        painter.drawText(margin, 15, self.title)

        # Data line
        if len(self.data) > 1:
            pen = QPen(self.color, 2, Qt.SolidLine)
            painter.setPen(pen)
            points = []
            for i, val in enumerate(self.data):
                x = margin + int(plot_w * i / (len(self.data) - 1))
                y = margin + int(plot_h * (1.0 - val))
                points.append((x, y))
            for i in range(len(points) - 1):
                painter.drawLine(points[i][0], points[i][1],
                                 points[i+1][0], points[i+1][1])

        # Y-axis labels
        painter.setPen(QColor(Colors.TEXT_MUTED))
        painter.setFont(QFont("JetBrains Mono", 7))
        for i, val in enumerate([1.0, 0.75, 0.5, 0.25, 0.0]):
            y = margin + int(plot_h * i / 4)
            painter.drawText(2, y + 4, f"{val:.1f}")

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

    # Signals for thread-safe UI updates
    quality_updated = pyqtSignal(dict)
    collector_updated = pyqtSignal(dict)
    robot_updated = pyqtSignal(dict)
    ai_updated = pyqtSignal(dict)
    system_updated = pyqtSignal(dict)
    image_updated = pyqtSignal(object)

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
        self.sub_system = self.create_subscription(
            SystemStatus, "/system_status", self._on_system, reliable_qos
        )
        self.sub_image = self.create_subscription(
            Image, "/vision_debug", self._on_image, sensor_qos
        )

        self._bridge = CvBridge() if CV_AVAILABLE else None
        self._latest_frame = None

    def _on_quality(self, msg: FiberQuality):
        data = {
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
        }
        self.quality_updated.emit(data)

    def _on_collector(self, msg: CollectorStatus):
        data = {
            "rpm": msg.rpm,
            "target_rpm": msg.target_rpm,
            "running": msg.running,
            "at_setpoint": msg.at_setpoint,
            "vibration": msg.vibration_score,
            "temperature": msg.temperature_c,
            "duty": msg.duty_cycle,
            "estop": msg.emergency_stop,
        }
        self.collector_updated.emit(data)

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
        data = {
            "state": msg.system_state,
            "sim": msg.simulation_mode,
            "uptime": msg.uptime_s,
            "estop": msg.emergency_stop,
            "quality": msg.quality_current,
        }
        self.system_updated.emit(data)

    def _on_image(self, msg: Image):
        if self._bridge:
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                self.image_updated.emit(rgb_img)
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
    """Main industrial dashboard window."""

    def __init__(self, ros_node: DashboardNode):
        super().__init__()
        self.ros = ros_node
        self._estop_active = False
        self._manual_mode = False

        title = ros_node.get_parameter("window_title").value
        self.setWindowTitle(title)
        self.setMinimumSize(1400, 900)
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
        """)

        self._build_ui()
        self._connect_signals()

        # Refresh timer
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # ── Left Panel: Camera + Quality ─────────────────────────────────────
        left_panel = QVBoxLayout()

        # Camera feed
        cam_group = QGroupBox("Vision System")
        cam_layout = QVBoxLayout(cam_group)
        self.camera_label = QLabel("No camera feed")
        self.camera_label.setMinimumSize(480, 360)
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

        # Quality trend
        self.quality_trend = TrendGraph("Quality Trend")
        self.quality_trend.set_color(Colors.ACCENT_CYAN)
        left_panel.addWidget(self.quality_trend)

        left_panel.addStretch()
        main_layout.addLayout(left_panel, stretch=3)

        # ── Center Panel: Gauges + Controls ──────────────────────────────────
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

        # Manual controls
        controls_group = QGroupBox("Manual Controls")
        ctrl_layout = QGridLayout(controls_group)

        # RPM control
        ctrl_layout.addWidget(QLabel("Target RPM:"), 0, 0)
        self.rpm_slider = QSlider(Qt.Horizontal)
        self.rpm_slider.setRange(0, 3000)
        self.rpm_slider.setValue(500)
        self.rpm_slider.valueChanged.connect(self._on_rpm_changed)
        ctrl_layout.addWidget(self.rpm_slider, 0, 1)
        self.rpm_label = QLabel("500")
        self.rpm_label.setMinimumWidth(50)
        ctrl_layout.addWidget(self.rpm_label, 0, 2)

        # Flow control
        ctrl_layout.addWidget(QLabel("Flow Rate:"), 1, 0)
        self.flow_slider = QSlider(Qt.Horizontal)
        self.flow_slider.setRange(0, 1000)
        self.flow_slider.setValue(100)
        self.flow_slider.valueChanged.connect(self._on_flow_changed)
        ctrl_layout.addWidget(self.flow_slider, 1, 1)
        self.flow_label = QLabel("1.00")
        self.flow_label.setMinimumWidth(50)
        ctrl_layout.addWidget(self.flow_label, 1, 2)

        # Mode switch
        ctrl_layout.addWidget(QLabel("Mode:"), 2, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["AUTONOMOUS", "MANUAL"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        ctrl_layout.addWidget(self.mode_combo, 2, 1)

        center_panel.addWidget(controls_group)

        # AI Status
        ai_group = QGroupBox("AI Controller")
        ai_layout = QVBoxLayout(ai_group)
        self.ai_mode_label = QLabel("Mode: --")
        self.ai_cycle_label = QLabel("Cycle: --")
        self.ai_quality_label = QLabel("Quality: --")
        self.ai_rationale_label = QLabel("Rationale: --")
        self.ai_rationale_label.setWordWrap(True)
        self.ai_rationale_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 8px; border-radius: 4px;"
        )
        for lbl in [self.ai_mode_label, self.ai_cycle_label,
                    self.ai_quality_label, self.ai_rationale_label]:
            ai_layout.addWidget(lbl)
        center_panel.addWidget(ai_group)

        # Robot status
        robot_group = QGroupBox("Robot Status")
        robot_layout = QGridLayout(robot_group)
        self.robot_state_label = QLabel("State: --")
        self.robot_dist_label = QLabel("Distance: -- mm")
        self.robot_scan_label = QLabel("Scan: -- mm/s")
        self.robot_coverage_label = QLabel("Coverage: --")
        robot_layout.addWidget(self.robot_state_label, 0, 0)
        robot_layout.addWidget(self.robot_dist_label, 0, 1)
        robot_layout.addWidget(self.robot_scan_label, 1, 0)
        robot_layout.addWidget(self.robot_coverage_label, 1, 1)
        center_panel.addWidget(robot_group)

        center_panel.addStretch()
        main_layout.addLayout(center_panel, stretch=4)

        # ── Right Panel: Safety + System ─────────────────────────────────────
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

        # Diagnosis
        diag_group = QGroupBox("Diagnosis")
        diag_layout = QVBoxLayout(diag_group)
        self.diag_label = QLabel("nominal")
        self.diag_label.setWordWrap(True)
        self.diag_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 8px; border-radius: 4px; font-size: 11px;"
        )
        diag_layout.addWidget(self.diag_label)
        right_panel.addWidget(diag_group)

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
        self.ros.image_updated.connect(self._update_image)

    # ── Slot handlers ────────────────────────────────────────────────────────

    def _update_quality(self, data: dict):
        self.q_overall.set_value(data.get("overall", 0))
        self.q_uniformity.set_value(data.get("uniformity", 0))
        self.q_bead.set_value(data.get("bead_score", 0))
        self.q_cone.set_value(data.get("cone_score", 0))
        self.q_coverage.set_value(data.get("coverage", 0))
        self.q_density.set_value(data.get("density", 0))
        self.quality_gauge.set_value(data.get("overall", 0) * 100)
        self.quality_trend.add_point(data.get("overall", 0))
        self.diag_label.setText(data.get("diagnosis", "nominal"))
        grade = data.get("grade", 0)
        grade_names = ["UNUSABLE", "POOR", "FAIR", "GOOD", "EXCELLENT"]
        color = [Colors.ERROR, Colors.ERROR, Colors.WARNING,
                 Colors.SUCCESS, Colors.SUCCESS][min(grade, 4)]
        self.diag_label.setStyleSheet(
            f"background-color: {Colors.BG_CARD}; padding: 8px; border-radius: 4px; "
            f"font-size: 11px; color: {color};"
        )

    def _update_collector(self, data: dict):
        rpm = data.get("rpm", 0)
        self.rpm_gauge.set_value(rpm)
        self.coll_running_label.setText(f"Running: {'YES' if data.get('running') else 'NO'}")
        self.coll_setpoint_label.setText(f"At Setpoint: {'YES' if data.get('at_setpoint') else 'NO'}")
        vib = data.get("vibration", 0)
        vib_color = Colors.SUCCESS if vib < 0.3 else (Colors.WARNING if vib < 0.6 else Colors.ERROR)
        self.coll_vibration_label.setText(f"Vibration: {vib:.2f}")
        self.coll_vibration_label.setStyleSheet(f"color: {vib_color};")
        self.coll_temp_label.setText(f"Temp: {data.get('temperature', 0):.1f} C")
        self.coll_duty_label.setText(f"Duty: {data.get('duty', 0) * 100:.0f} %")

    def _update_robot(self, data: dict):
        self.robot_state_label.setText(f"State: {data.get('state', '--')}")
        self.robot_dist_label.setText(f"Distance: {data.get('distance_mm', 0):.1f} mm")
        self.robot_scan_label.setText(f"Scan: {data.get('scan_speed', 0):.1f} mm/s")
        self.robot_coverage_label.setText(f"Coverage: {data.get('coverage_mean', 0):.3f}")
        self.distance_gauge.set_value(data.get('distance_mm', 0))

    def _update_ai(self, data: dict):
        self.ai_mode_label.setText(f"Mode: {data.get('mode', '--')}")
        self.ai_cycle_label.setText(f"Cycle: {data.get('cycle', 0)}")
        self.ai_quality_label.setText(f"Quality: {data.get('current_quality', 0):.3f}")
        self.ai_rationale_label.setText(data.get('rationale', '--'))

    def _update_system(self, data: dict):
        states = ["OFF", "INIT", "READY", "RUNNING", "ERROR", "E-STOP"]
        state = states[data.get("state", 0)]
        self.sys_state_label.setText(f"State: {state}")
        self.sys_sim_label.setText(f"Mode: {'SIM' if data.get('sim') else 'REAL'}")
        uptime = data.get("uptime", 0)
        self.sys_uptime_label.setText(f"Uptime: {uptime:.0f}s")

    def _update_image(self, img):
        if img is not None:
            h, w, ch = img.shape
            from PyQt5.QtGui import QImage, QPixmap
            q_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.camera_label.setPixmap(pixmap)

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
            self.estop_status.setStyleSheet(
                f"color: {Colors.ERROR}; font-weight: bold;"
            )
            self.statusBar().showMessage("EMERGENCY STOP ACTIVATED")
        else:
            self.estop_status.setText("E-STOP: CLEAR")
            self.estop_status.setStyleSheet(
                f"color: {Colors.SUCCESS}; font-weight: bold;"
            )
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
    rclpy.init(args=args)
    ros_node = DashboardNode()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
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

    # Spin ROS2 in a background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    ret = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
