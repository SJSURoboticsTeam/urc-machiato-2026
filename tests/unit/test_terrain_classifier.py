#!/usr/bin/env python3
"""
Unit tests for Terrain Intelligence - Centralized Vision Processing Integration

Tests terrain analyzer integration with centralized vision processing system.
Validates terrain map subscription and processing rather than direct image processing.
"""

import os
import sys
from typing import Any, Dict
from unittest.mock import Mock, patch

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

# Add autonomy modules to path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
AUTONOMY_CODE_ROOT = os.path.join(PROJECT_ROOT, "src", "autonomy")
sys.path.insert(0, AUTONOMY_CODE_ROOT)

try:
    from core.terrain_intelligence.terrain_analyzer import TerrainAnalyzer, TerrainType
except ImportError:
    # Fallback for testing without full ROS2 environment
    TerrainAnalyzer = None


@pytest.mark.skipif(TerrainAnalyzer is None, reason="TerrainAnalyzer not available")
class TestTerrainAnalyzerVisionIntegration:
    """Test Terrain Analyzer integration with centralized vision processing."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def terrain_analyzer(self, ros_context):
        """Create terrain analyzer instance."""
        with patch("rclpy.node.Node.__init__", return_value=None):
            analyzer = TerrainAnalyzer.__new__(TerrainAnalyzer)
            # Mock required attributes
            analyzer.terrain_map = None
            analyzer.robot_pose = None
            analyzer.map_initialized = False
            analyzer.bridge = Mock()
            analyzer.get_logger = Mock(return_value=Mock())
            analyzer.get_logger.return_value.info = Mock()
            analyzer.get_logger.return_value.warn = Mock()
            analyzer.get_logger.return_value.error = Mock()
            return analyzer

    def test_terrain_map_subscription(self, terrain_analyzer):
        """Test subscription to centralized terrain map."""
        # Mock the subscription creation
        with patch.object(terrain_analyzer, "create_subscription") as mock_sub:
            # Re-initialize to trigger subscription creation
            terrain_analyzer.__init__()

            # Verify terrain map subscription was created
            mock_sub.assert_any_call(
                OccupancyGrid,
                "/vision/terrain_map",
                terrain_analyzer.terrain_map_callback,
                pytest.any(),  # QoS profile
            )

    def test_terrain_map_callback_processing(self, terrain_analyzer):
        """Test processing of terrain map messages from vision system."""
        # Create mock terrain map message
        terrain_map = OccupancyGrid()
        terrain_map.header.stamp = rclpy.time.Time().to_msg()
        terrain_map.header.frame_id = "odom"
        terrain_map.info.resolution = 0.1
        terrain_map.info.width = 200
        terrain_map.info.height = 200
        terrain_map.info.origin.position.x = -10.0
        terrain_map.info.origin.position.y = -10.0
        terrain_map.data = [0] * 40000  # 200x200 grid

        # Add some terrain features (simulate vision processing output)
        # Sand area (traversable)
        for i in range(10000, 15000):
            terrain_map.data[i] = 10  # Low cost traversable

        # Rock area (higher cost)
        for i in range(25000, 30000):
            terrain_map.data[i] = 80  # High cost obstacle

        # Mock traversability publisher
        terrain_analyzer.traversability_pub = Mock()
        terrain_analyzer.traversability_pub.publish = Mock()

        # Call callback
        terrain_analyzer.terrain_map_callback(terrain_map)

        # Verify terrain map was stored
        assert terrain_analyzer.terrain_map is not None
        assert terrain_analyzer.terrain_map.info.width == 200
        assert terrain_analyzer.terrain_map.info.height == 200

        # Verify traversability map was published
        terrain_analyzer.traversability_pub.publish.assert_called()

    def test_pointcloud_integration(self, terrain_analyzer):
        """Test point cloud processing for slope analysis."""
        # Create mock point cloud message
        pointcloud = PointCloud2()
        pointcloud.header.stamp = rclpy.time.Time().to_msg()
        pointcloud.header.frame_id = "camera_depth_optical_frame"

        # Mock point cloud data processing
        with patch.object(
            terrain_analyzer, "process_pointcloud_for_slopes"
        ) as mock_process:
            mock_process.return_value = {"max_slope": 0.3, "hazard_zones": []}

            terrain_analyzer.pointcloud_callback(pointcloud)

            # Verify point cloud processing was called
            mock_process.assert_called_once_with(pointcloud)

    def test_traversability_cost_mapping(self, terrain_analyzer):
        """Test traversability cost calculation from terrain map."""
        # Initialize terrain map
        terrain_analyzer.initialize_terrain_map()

        # Test different terrain types produce different costs
        sand_cost = terrain_analyzer.calculate_traversability_cost(
            TerrainType.SAND, 1.0
        )
        rock_cost = terrain_analyzer.calculate_traversability_cost(
            TerrainType.ROCK, 1.0
        )
        hazard_cost = terrain_analyzer.calculate_traversability_cost(
            TerrainType.HAZARD, 1.0
        )

        # Verify costs are reasonable and differentiated
        assert sand_cost < rock_cost  # Sand should be easier than rock
        assert rock_cost < hazard_cost  # Rock should be easier than hazard
        assert all(cost >= 0 for cost in [sand_cost, rock_cost, hazard_cost])

    def test_hazard_detection_from_vision(self, terrain_analyzer):
        """Test hazard detection using vision-processed terrain data."""
        # Create terrain map with hazards
        terrain_map = OccupancyGrid()
        terrain_map.info.width = 10
        terrain_map.info.height = 10
        terrain_map.data = [0] * 100

        # Add hazard (high occupancy values)
        for i in range(50, 60):  # Row of hazards
            terrain_map.data[i] = 100  # Complete obstacle

        hazards = terrain_analyzer.detect_hazards_from_map(terrain_map)

        # Verify hazards were detected
        assert isinstance(hazards, list)
        assert len(hazards) > 0

        # Verify hazard structure
        for hazard in hazards:
            assert "x" in hazard and "y" in hazard
            assert "type" in hazard
            assert hazard["type"] == TerrainType.HAZARD

    def test_slope_analysis_from_pointcloud(self, terrain_analyzer):
        """Test slope analysis from point cloud data."""
        # Mock point cloud with slope
        pointcloud = PointCloud2()

        # Mock slope calculation
        slopes = terrain_analyzer.analyze_slopes_from_pointcloud(pointcloud)

        # Verify slope analysis structure
        assert isinstance(slopes, dict)
        assert "max_slope" in slopes
        assert "average_slope" in slopes
        assert "slope_map" in slopes

    def test_terrain_type_constants(self):
        """Test terrain type constants are properly defined."""
        assert TerrainType.SAND == "sand"
        assert TerrainType.ROCK == "rock"
        assert TerrainType.SLOPE == "slope"
        assert TerrainType.HAZARD == "hazard"
        assert TerrainType.UNKNOWN == "unknown"

    def test_robot_pose_integration(self, terrain_analyzer):
        """Test robot pose integration for local terrain analysis."""
        # Mock odometry message
        odom_msg = Mock()
        odom_msg.pose.pose.position.x = 5.0
        odom_msg.pose.pose.position.y = 3.0
        odom_msg.pose.pose.orientation.w = 1.0  # No rotation

        terrain_analyzer.odom_callback(odom_msg)

        # Verify pose was stored
        assert terrain_analyzer.robot_pose is not None
        assert terrain_analyzer.robot_pose[0] == 5.0  # x position
        assert terrain_analyzer.robot_pose[1] == 3.0  # y position
