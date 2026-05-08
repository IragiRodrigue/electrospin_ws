#!/usr/bin/env python3
"""
ElectroSpin Vision System Node
================================
Real-time AI computer vision for nanofiber quality monitoring.

Capabilities:
  - Taylor cone stability detection
  - Bead formation detection (YOLOv8 or classical CV)
  - Fiber diameter estimation
  - Deposition density analysis
  - Jet stability tracking
  - Collector surface coverage monitoring

Pipeline:
  Camera → Frame Acquisition
         → Preprocessing (denoise, enhance)
         → Taylor Cone ROI → Cone Stability Score
         → Fiber Region → Diameter Estimation
         → Bead Detector → Bead Score
         → Coverage Mask → Deposition Density
         → FiberQuality message → /fiber_quality topic

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import time
import threading
import math
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from electrospin_interfaces.msg import FiberQuality

# Optional deep learning imports
try:
    import torch
    import torchvision.transforms as T
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VisionConfig:
    # Camera
    camera_index:        int   = 0
    frame_width:         int   = 1280
    frame_height:        int   = 720
    fps:                 int   = 30

    # Processing regions of interest (normalized 0.0–1.0)
    cone_roi_x:          float = 0.4
    cone_roi_y:          float = 0.1
    cone_roi_w:          float = 0.2
    cone_roi_h:          float = 0.35

    fiber_roi_x:         float = 0.1
    fiber_roi_y:         float = 0.45
    fiber_roi_w:         float = 0.8
    fiber_roi_h:         float = 0.45

    # Pixel-to-nm calibration (empirical, depends on optics)
    px_per_nm:           float = 0.0025   # 1px = 400nm at default magnification

    # Algorithm thresholds
    bead_min_area_px:    int   = 20
    bead_circularity:    float = 0.7
    fiber_min_length_px: int   = 30
    canny_low:           int   = 50
    canny_high:          int   = 150

    # Temporal smoothing
    temporal_alpha:      float = 0.3   # EMA smoothing factor


# ─────────────────────────────────────────────────────────────────────────────
# Taylor Cone Analyzer
# ─────────────────────────────────────────────────────────────────────────────

class TaylorConeAnalyzer:
    """
    Analyzes Taylor cone shape and jet stability from grayscale ROI.
    Uses edge detection and contour analysis to characterize cone geometry.
    """

    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg

    def analyze(
        self, roi: np.ndarray
    ) -> Tuple[float, float, float]:
        """
        Returns:
            cone_score:      0.0 (no cone) to 1.0 (ideal cone)
            jet_straightness:0.0 to 1.0
            jet_diameter_um: estimated jet diameter in micrometers
        """
        if roi is None or roi.size == 0:
            return 0.0, 0.0, 0.0

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
        edges = cv2.Canny(blurred, self.cfg.canny_low, self.cfg.canny_high)

        # Find contours
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return 0.0, 0.0, 0.0

        # Find largest contour (cone silhouette)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 100:
            return 0.1, 0.0, 0.0

        # Compute convex hull for ideality score
        hull = cv2.convexHull(largest)
        hull_area = cv2.contourArea(hull)
        solidity = float(area / (hull_area + 1e-6))

        # Fit bounding ellipse for cone shape assessment
        if len(largest) >= 5:
            ellipse = cv2.fitEllipse(largest)
            axes = ellipse[1]
            aspect_ratio = min(axes) / (max(axes) + 1e-6)
            # Ideal Taylor cone: narrow, tall → low aspect ratio is good
            cone_score = float(np.clip(solidity * (1.0 - aspect_ratio), 0.0, 1.0))
        else:
            cone_score = 0.3

        # Jet straightness: count edge pixels along vertical axis
        h, w = edges.shape
        vert_profile = edges[:, w//2 - 5 : w//2 + 5].sum(axis=1)
        straightness = float(np.clip(vert_profile.mean() / 255.0, 0.0, 1.0))

        # Jet diameter from horizontal slice near needle tip
        tip_slice = edges[int(h * 0.6), :]
        jet_px = int(tip_slice.sum() / 255)
        jet_diameter_um = float(jet_px / self.cfg.px_per_nm * 0.001)

        return (
            float(np.clip(cone_score, 0.0, 1.0)),
            float(np.clip(straightness, 0.0, 1.0)),
            float(np.clip(jet_diameter_um, 0.0, 500.0))
        )


# ─────────────────────────────────────────────────────────────────────────────
# Fiber Quality Analyzer
# ─────────────────────────────────────────────────────────────────────────────

class FiberAnalyzer:
    """
    Analyzes fiber morphology in the collector region.
    Estimates: diameter, uniformity, bead score, alignment.
    """

    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg

    def analyze(
        self, roi: np.ndarray
    ) -> Dict[str, float]:
        """Returns dict of fiber quality metrics."""
        result = {
            "uniformity":  0.5,
            "diameter_nm": 500.0,
            "bead_score":  0.0,
            "alignment":   0.5,
        }

        if roi is None or roi.size == 0:
            return result

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        blurred = cv2.GaussianBlur(gray, (3, 3), 1.0)

        # ── Fiber Diameter Estimation ─────────────────────────────────────────
        # Use skeleton width measurement via distance transform
        _, thresh = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        dist = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        fiber_widths = dist[dist > 0]
        if len(fiber_widths) > 0:
            avg_radius_px = float(np.median(fiber_widths))
            diameter_nm = avg_radius_px * 2.0 / self.cfg.px_per_nm
            result["diameter_nm"] = float(np.clip(diameter_nm, 50.0, 5000.0))

            # Uniformity from coefficient of variation
            cv_val = float(fiber_widths.std() / (fiber_widths.mean() + 1e-6))
            result["uniformity"] = float(np.clip(1.0 - cv_val, 0.0, 1.0))

        # ── Bead Detection ────────────────────────────────────────────────────
        # Beads appear as circular blobs larger than fiber width
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea       = True
        params.minArea            = self.cfg.bead_min_area_px
        params.maxArea            = 5000
        params.filterByCircularity = True
        params.minCircularity     = self.cfg.bead_circularity
        params.filterByConvexity  = True
        params.minConvexity       = 0.8

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(blurred)
        num_beads = len(keypoints)
        bead_area = sum(k.size ** 2 * math.pi / 4.0 for k in keypoints)
        total_area = float(roi.shape[0] * roi.shape[1])
        result["bead_score"] = float(np.clip(bead_area / total_area * 10.0, 0.0, 1.0))

        # ── Fiber Alignment ───────────────────────────────────────────────────
        # Use gradient orientation histogram
        gx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        angles = np.arctan2(gy, gx)
        hist, _ = np.histogram(angles.ravel(), bins=36, range=(-math.pi, math.pi))
        hist = hist.astype(float)
        hist /= hist.sum() + 1e-6
        # Peak concentration = aligned fibers
        peak_concentration = float(hist.max() * 36.0)
        result["alignment"] = float(np.clip((peak_concentration - 1.0) / 5.0, 0.0, 1.0))

        return result


# ─────────────────────────────────────────────────────────────────────────────
# Coverage Analyzer
# ─────────────────────────────────────────────────────────────────────────────

class CoverageAnalyzer:
    """
    Estimates fiber deposition density and spatial uniformity
    on the collector surface.
    """

    def __init__(self, cfg: VisionConfig, zones: int = 16):
        self.cfg   = cfg
        self.zones = zones

    def analyze(self, roi: np.ndarray) -> Tuple[float, float]:
        """
        Returns:
            density:   overall coverage density 0.0–1.0
            uniformity:spatial uniformity 0.0–1.0
        """
        if roi is None or roi.size == 0:
            return 0.0, 0.0

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        density = float(mask.sum() / 255) / float(mask.size)

        # Zone-wise density for uniformity
        h, w = mask.shape
        zone_densities = []
        zone_w = w // self.zones
        for z in range(self.zones):
            zone = mask[:, z * zone_w : (z + 1) * zone_w]
            zone_dens = float(zone.sum() / 255) / float(zone.size + 1e-6)
            zone_densities.append(zone_dens)

        if sum(zone_densities) > 0:
            cv_val = float(np.std(zone_densities) / (np.mean(zone_densities) + 1e-6))
            uniformity = float(np.clip(1.0 - cv_val, 0.0, 1.0))
        else:
            uniformity = 0.0

        return float(np.clip(density, 0.0, 1.0)), uniformity


# ─────────────────────────────────────────────────────────────────────────────
# Vision System Node
# ─────────────────────────────────────────────────────────────────────────────

class VisionSystemNode(Node):
    """
    ROS2 Vision System node for real-time nanofiber quality monitoring.

    Topics Published:
        /camera/image_raw  → sensor_msgs/Image
        /fiber_quality     → electrospin_interfaces/FiberQuality
        /vision_debug      → sensor_msgs/Image (annotated)

    Topics Subscribed:
        (none by default; system operates autonomously)
    """

    def __init__(self):
        super().__init__("vision_system")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("processing_fps", 10.0)
        self.declare_parameter("use_yolo", False)
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("temporal_smoothing", True)
        self.declare_parameter("debug_visualization", True)

        self.sim_mode    = self.get_parameter("simulation_mode").value
        cam_idx          = self.get_parameter("camera_index").value
        proc_fps         = self.get_parameter("processing_fps").value
        self.use_debug   = self.get_parameter("debug_visualization").value
        self.smooth      = self.get_parameter("temporal_smoothing").value

        # ── Components ────────────────────────────────────────────────────────
        self.cfg          = VisionConfig(camera_index=cam_idx)
        self.bridge       = CvBridge()
        self.cone_analyzer   = TaylorConeAnalyzer(self.cfg)
        self.fiber_analyzer  = FiberAnalyzer(self.cfg)
        self.coverage_analyzer = CoverageAnalyzer(self.cfg)

        # ── Camera Setup ──────────────────────────────────────────────────────
        if not self.sim_mode:
            self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cfg.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        else:
            self.cap = None

        # ── Smoothed state (EMA) ──────────────────────────────────────────────
        self._smoothed = {
            "uniformity":    0.5,
            "diameter_nm":   500.0,
            "bead_score":    0.0,
            "cone_score":    0.5,
            "jet_straight":  0.5,
            "density":       0.3,
            "cov_uniform":   0.5,
            "alignment":     0.5,
        }
        self._alpha = self.cfg.temporal_alpha

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_image   = self.create_publisher(Image, "/camera/image_raw", sensor_qos)
        self.pub_quality = self.create_publisher(FiberQuality, "/fiber_quality", sensor_qos)
        self.pub_debug   = self.create_publisher(Image, "/vision_debug", sensor_qos)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.proc_timer = self.create_timer(
            1.0 / proc_fps, self._process_frame
        )

        self.get_logger().info(
            f"[VisionSystem] Initialized. Sim={self.sim_mode}, FPS={proc_fps}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Frame Processing
    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self):
        """Acquire and analyze a camera frame."""
        frame = self._get_frame()
        if frame is None:
            return

        h, w = frame.shape[:2]

        # ── Extract ROIs ──────────────────────────────────────────────────────
        def get_roi(fx, fy, fw, fh):
            x0 = int(fx * w);  y0 = int(fy * h)
            x1 = int((fx + fw) * w); y1 = int((fy + fh) * h)
            return frame[y0:y1, x0:x1], (x0, y0, x1, y1)

        cone_roi, cone_bounds  = get_roi(
            self.cfg.cone_roi_x, self.cfg.cone_roi_y,
            self.cfg.cone_roi_w, self.cfg.cone_roi_h
        )
        fiber_roi, fiber_bounds = get_roi(
            self.cfg.fiber_roi_x, self.cfg.fiber_roi_y,
            self.cfg.fiber_roi_w, self.cfg.fiber_roi_h
        )

        # ── Analyze ───────────────────────────────────────────────────────────
        cone_score, jet_str, jet_diam = self.cone_analyzer.analyze(cone_roi)
        fiber_metrics = self.fiber_analyzer.analyze(fiber_roi)
        density, cov_uniform = self.coverage_analyzer.analyze(fiber_roi)

        # ── Temporal Smoothing (EMA) ──────────────────────────────────────────
        def ema(key, new_val):
            if self.smooth:
                self._smoothed[key] = (
                    self._alpha * new_val +
                    (1 - self._alpha) * self._smoothed[key]
                )
            else:
                self._smoothed[key] = new_val

        ema("cone_score",   cone_score)
        ema("jet_straight", jet_str)
        ema("uniformity",   fiber_metrics["uniformity"])
        ema("diameter_nm",  fiber_metrics["diameter_nm"])
        ema("bead_score",   fiber_metrics["bead_score"])
        ema("alignment",    fiber_metrics["alignment"])
        ema("density",      density)
        ema("cov_uniform",  cov_uniform)

        # ── Publish FiberQuality ──────────────────────────────────────────────
        self._publish_quality()

        # ── Publish raw image ─────────────────────────────────────────────────
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_image.publish(img_msg)
        except Exception:
            pass

        # ── Publish debug visualization ───────────────────────────────────────
        if self.use_debug:
            debug = self._draw_debug(frame.copy(), cone_bounds, fiber_bounds)
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                dbg_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_debug.publish(dbg_msg)
            except Exception:
                pass

    def _publish_quality(self):
        """Build and publish FiberQuality message."""
        s = self._smoothed
        q = FiberQuality()
        q.header.stamp = self.get_clock().now().to_msg()

        q.uniformity          = float(s["uniformity"])
        q.diameter            = float(s["diameter_nm"])
        q.bead_score          = float(s["bead_score"])
        q.deposition_density  = float(s["density"])
        q.jet_stable          = bool(s["cone_score"] > 0.4 and s["jet_straight"] > 0.3)
        q.taylor_cone_score   = float(s["cone_score"])
        q.jet_straightness    = float(s["jet_straight"])
        q.coverage_uniformity = float(s["cov_uniform"])
        q.alignment_score     = float(s["alignment"])

        # Overall quality composite
        quality = (
            0.25 * s["uniformity"] +
            0.20 * s["cone_score"] +
            0.20 * (1.0 - s["bead_score"]) +
            0.15 * s["cov_uniform"] +
            0.10 * s["jet_straight"] +
            0.10 * s["alignment"]
        )
        q.overall_quality = float(np.clip(quality, 0.0, 1.0))

        # Quality grade
        if   quality >= 0.85: q.quality_grade = 4
        elif quality >= 0.70: q.quality_grade = 3
        elif quality >= 0.50: q.quality_grade = 2
        elif quality >= 0.30: q.quality_grade = 1
        else:                 q.quality_grade = 0

        # Diagnosis
        issues = []
        if s["bead_score"]  > 0.3:  issues.append("beading")
        if s["cone_score"]  < 0.3:  issues.append("unstable cone")
        if s["uniformity"]  < 0.4:  issues.append("non-uniform")
        if s["density"]     < 0.2:  issues.append("low coverage")
        q.diagnosis = ", ".join(issues) if issues else "nominal"

        self.pub_quality.publish(q)

    def _get_frame(self) -> Optional[np.ndarray]:
        """Get a camera frame (real or simulated)."""
        if self.sim_mode:
            return self._generate_synthetic_frame()

        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            return frame if ret else None
        return None

    def _generate_synthetic_frame(self) -> np.ndarray:
        """Generate a realistic-looking synthetic electrospinning frame."""
        t = time.time()
        frame = np.zeros((self.cfg.frame_height, self.cfg.frame_width, 3), dtype=np.uint8)
        frame[:] = (15, 15, 25)  # Dark background

        # Draw collector drum
        cy = int(self.cfg.frame_height * 0.75)
        cv2.rectangle(frame, (100, cy), (self.cfg.frame_width - 100, cy + 80),
                      (40, 40, 60), -1)

        # Draw fiber deposition (random lines)
        np.random.seed(int(t * 10) % 1000)
        for _ in range(80):
            x1 = np.random.randint(100, self.cfg.frame_width - 100)
            x2 = x1 + np.random.randint(-40, 40)
            y1 = np.random.randint(cy, cy + 80)
            y2 = y1 + np.random.randint(-5, 5)
            brightness = np.random.randint(100, 200)
            cv2.line(frame, (x1, y1), (x2, y2), (brightness,) * 3, 1)

        # Draw Taylor cone
        cone_x = self.cfg.frame_width // 2
        cone_y = int(self.cfg.frame_height * 0.25)
        cone_pts = np.array([
            [cone_x, cone_y],
            [cone_x - 30, cone_y + 80],
            [cone_x + 30, cone_y + 80]
        ])
        cv2.fillPoly(frame, [cone_pts], (200, 200, 220))

        # Jet
        jet_wobble = int(10 * math.sin(t * 2.0))
        cv2.line(frame,
                 (cone_x + jet_wobble, cone_y + 80),
                 (cone_x + jet_wobble // 2, cy),
                 (220, 220, 230), 2)

        return frame

    def _draw_debug(
        self, frame: np.ndarray,
        cone_bounds: tuple, fiber_bounds: tuple
    ) -> np.ndarray:
        """Draw diagnostic overlays on debug frame."""
        s = self._smoothed

        # ROI rectangles
        cv2.rectangle(frame, (cone_bounds[0], cone_bounds[1]),
                      (cone_bounds[2], cone_bounds[3]), (0, 255, 255), 2)
        cv2.putText(frame, f"Cone:{s['cone_score']:.2f}",
                    (cone_bounds[0], cone_bounds[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.rectangle(frame, (fiber_bounds[0], fiber_bounds[1]),
                      (fiber_bounds[2], fiber_bounds[3]), (0, 200, 100), 2)
        cv2.putText(frame, f"Q:{s['uniformity']:.2f} Bead:{s['bead_score']:.2f}",
                    (fiber_bounds[0], fiber_bounds[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 100), 1)

        # HUD overlay
        h = frame.shape[0]
        lines = [
            f"Diameter: {s['diameter_nm']:.0f} nm",
            f"Uniformity: {s['uniformity']:.2f}",
            f"Density: {s['density']:.2f}",
            f"Jet stable: {s['cone_score'] > 0.4}",
        ]
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (10, 30 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 100), 1)

        return frame

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VisionSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()