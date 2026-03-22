#!/usr/bin/env python3
"""
RealSense D435 Driver Node for URC 2026 SLAM Pipeline

Publishes RGB, depth, and camera_info on topics consumed by depth_processor
and RTAB-Map (or equivalent):
  - camera/rgb/image_raw
  - camera/depth/image_raw
  - camera/depth/camera_info

Uses pyrealsense2 when available; supports parameter loading from realsense_d435.yaml.

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

logger = logging.getLogger(__name__)

try:
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    rs = None


class RealSenseDriver(Node):
    """
    ROS2 node that drives Intel RealSense D435 and publishes RGB, depth, and camera_info.

    Topics (matching depth_processor expectations):
      - camera/rgb/image_raw (sensor_msgs/Image, bgr8)
      - camera/depth/image_raw (sensor_msgs/Image, 16UC1 in mm)
      - camera/depth/camera_info (sensor_msgs/CameraInfo)
    """

    def __init__(self) -> None:
        super().__init__("realsense_driver")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 30)
        self.declare_parameter("depth_preset", "HighAccuracy")
        self.declare_parameter("align_depth_to_color", True)
        self.declare_parameter("optical_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("base_frame_id", "camera_link")

        self._width = self.get_parameter("width").value
        self._height = self.get_parameter("height").value
        self._fps = self.get_parameter("fps").value
        self._depth_width = self.get_parameter("depth_width").value
        self._depth_height = self.get_parameter("depth_height").value
        self._depth_fps = self.get_parameter("depth_fps").value
        self._depth_preset = self.get_parameter("depth_preset").value
        self._align_depth_to_color = self.get_parameter("align_depth_to_color").value
        self._optical_frame_id = self.get_parameter("optical_frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value

        self._pipeline: Optional[object] = None
        self._align: Optional[object] = None

        self._rgb_pub = self.create_publisher(Image, "camera/rgb/image_raw", 10)
        self._depth_pub = self.create_publisher(Image, "camera/depth/image_raw", 10)
        self._info_pub = self.create_publisher(
            CameraInfo, "camera/depth/camera_info", 10
        )

        if not REALSENSE_AVAILABLE:
            self.get_logger().error(
                "pyrealsense2 not installed. Install with: pip install pyrealsense2. "
                "RealSense driver will not stream."
            )
            return

        self._init_pipeline()

    def _init_pipeline(self) -> None:
        if not REALSENSE_AVAILABLE or rs is None:
            return
        try:
            self._pipeline = rs.pipeline()
            cfg = rs.config()

            cfg.enable_stream(
                rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps
            )
            cfg.enable_stream(
                rs.stream.depth,
                self._depth_width,
                self._depth_height,
                rs.format.z16,
                self._depth_fps,
            )

            profile = self._pipeline.start(cfg)
            depth_sensor = profile.get_device().first_depth_sensor()
            if depth_sensor.supports(rs.option.visual_preset):
                presets = {
                    "Default": rs.option.visual_preset,
                    "Hand": 1,
                    "HighAccuracy": 2,
                    "HighDensity": 3,
                    "MediumDensity": 4,
                }
                preset_val = presets.get(self._depth_preset, 2)
                depth_sensor.set_option(rs.option.visual_preset, preset_val)

            if self._align_depth_to_color:
                self._align = rs.align(rs.stream.color)

            self.get_logger().info(
                f"RealSense D435 started: color {self._width}x{self._height}@{self._fps}, "
                f"depth {self._depth_width}x{self._depth_height}@{self._depth_fps}, "
                f"preset={self._depth_preset}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            self._pipeline = None

    def _build_camera_info(
        self, frame_id: str, width: int, height: int, intr
    ) -> CameraInfo:
        info = CameraInfo()
        info.header = Header(frame_id=frame_id)
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [
            intr.coeffs[0],
            intr.coeffs[1],
            intr.coeffs[2],
            intr.coeffs[3],
            intr.coeffs[4],
        ]
        info.k = [
            intr.fx,
            0.0,
            intr.ppx,
            0.0,
            intr.fy,
            intr.ppy,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            intr.fx,
            0.0,
            intr.ppx,
            0.0,
            0.0,
            intr.fy,
            intr.ppy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def run_once(self) -> bool:
        """Capture and publish one frame. Returns True if successful."""
        if not REALSENSE_AVAILABLE or self._pipeline is None:
            return False
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            if self._align is not None:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return False

            stamp = self.get_clock().now().to_msg()
            header = Header(stamp=stamp, frame_id=self._optical_frame_id)

            # RGB
            color_data = np.asanyarray(color_frame.get_data())
            rgb_msg = Image()
            rgb_msg.header = header
            rgb_msg.height, rgb_msg.width = color_data.shape[:2]
            rgb_msg.encoding = "bgr8"
            rgb_msg.is_bigendian = 0
            rgb_msg.step = color_data.shape[1] * 3
            rgb_msg.data = color_data.tobytes()
            self._rgb_pub.publish(rgb_msg)

            # Depth (16UC1, mm)
            depth_data = np.asanyarray(depth_frame.get_data())
            depth_msg = Image()
            depth_msg.header = header
            depth_msg.height, depth_msg.width = depth_data.shape[:2]
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = 0
            depth_msg.step = depth_data.nbytes // depth_msg.height
            depth_msg.data = depth_data.tobytes()
            self._depth_pub.publish(depth_msg)

            # Camera info from depth intrinsics (aligned depth shares color frame geometry)
            intr = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
            info_msg = self._build_camera_info(
                self._optical_frame_id, depth_msg.width, depth_msg.height, intr
            )
            info_msg.header = header
            self._info_pub.publish(info_msg)

            return True
        except Exception as e:
            self.get_logger().warn(f"Frame capture failed: {e}")
            return False

    def destroy_node(self, *args, **kwargs) -> None:
        if REALSENSE_AVAILABLE and self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        super().destroy_node(*args, **kwargs)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseDriver()
    if not REALSENSE_AVAILABLE or node._pipeline is None:
        node.get_logger().warn("RealSense not available; node will exit.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        import time

        rate = max(1, node._fps)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if node.run_once():
                time.sleep(1.0 / rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
