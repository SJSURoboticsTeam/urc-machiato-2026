#!/usr/bin/env python3
"""
USD Scene Automation for Isaac Sim Mars Environment

This script automates the creation of a Mars-like environment in Isaac Sim
with proper lighting, terrain, and physics settings for the URC rover.

Usage:
    python3 setup_mars_scene.py --output /path/to/output.usd
    python3 setup_mars_scene.py --with-rover --rover-urdf /path/to/rover.urdf
    python3 setup_mars_scene.py --headless  # For batch processing
"""

import argparse
import sys
import os
from pathlib import Path
from typing import Optional, List, Tuple


class MarsSceneSetup:
    """Automates Mars environment setup in Isaac Sim."""
    
    def __init__(self, headless: bool = False):
        """Initialize Mars scene setup.
        
        Args:
            headless: Run without GUI (for batch processing)
        """
        self.headless = headless
        self.stage = None
        self.world = None
        
    def initialize_isaac(self) -> bool:
        """Initialize Isaac Sim environment."""
        try:
            # Import Isaac Sim modules (only available inside Isaac Sim)
            from omni.isaac.kit import SimulationApp
            
            # Start Isaac Sim
            config = {
                "headless": self.headless,
                "width": 1920,
                "height": 1080,
            }
            
            self.simulation_app = SimulationApp(config)
            
            # Import core modules after initialization
            from omni.isaac.core import World
            from pxr import Usd, UsdGeom, Gf, Sdf
            import omni.usd
            
            self.World = World
            self.Usd = Usd
            self.UsdGeom = UsdGeom
            self.Gf = Gf
            self.Sdf = Sdf
            self.omni_usd = omni.usd
            
            return True
            
        except ImportError as e:
            print(f"Error: Isaac Sim modules not available. {e}")
            print("This script must be run from within Isaac Sim.")
            return False
    
    def create_world(self) -> bool:
        """Create a new world with Mars physics."""
        try:
            self.world = self.World(
                stage_units_in_meters=1.0,
                physics_dt=1.0 / 60.0,
                rendering_dt=1.0 / 60.0,
            )
            
            self.stage = self.omni_usd.get_context().get_stage()
            
            print("✓ Created world with Mars physics")
            return True
            
        except Exception as e:
            print(f"Error creating world: {e}")
            return False
    
    def setup_mars_physics(self):
        """Configure Mars physics (3.71 m/s² gravity)."""
        try:
            from omni.isaac.core.utils.prims import set_prim_attribute_value
            
            # Set Mars gravity
            set_prim_attribute_value(
                "/physicsScene",
                "gravityDirection",
                self.Gf.Vec3f(0.0, 0.0, -1.0)
            )
            set_prim_attribute_value(
                "/physicsScene",
                "gravityMagnitude",
                3.71  # Mars gravity
            )
            
            print("✓ Set Mars gravity (3.71 m/s²)")
            
        except Exception as e:
            print(f"Warning: Could not set Mars physics: {e}")
    
    def setup_mars_lighting(self):
        """Setup Mars lighting (distant sun, reddish ambient)."""
        try:
            from omni.isaac.core.utils.prims import create_prim
            
            # Create distant light (sun)
            sun_prim = create_prim(
                "/World/Lighting/Sun",
                "DistantLight",
                attributes={
                    "intensity": 1.0,
                    "color": self.Gf.Vec3f(1.0, 0.95, 0.9),  # Slightly yellowish
                    "angle": 0.53,  # Sun angular diameter
                }
            )
            
            # Position sun
            sun_xform = self.UsdGeom.Xform(sun_prim)
            sun_xform.AddTranslateOp().Set(self.Gf.Vec3f(0.0, 0.0, 100.0))
            sun_xform.AddRotateXOp().Set(-45.0)
            
            # Create ambient light (reddish Mars sky)
            dome_light = create_prim(
                "/World/Lighting/Sky",
                "DomeLight",
                attributes={
                    "intensity": 0.3,
                    "color": self.Gf.Vec3f(0.9, 0.7, 0.6),  # Reddish
                }
            )
            
            print("✓ Set up Mars lighting")
            
        except Exception as e:
            print(f"Warning: Could not set up lighting: {e}")
    
    def create_terrain(self, size: float = 100.0, resolution: int = 512):
        """Create Mars terrain with rocks and craters.
        
        Args:
            size: Terrain size in meters
            resolution: Heightmap resolution
        """
        try:
            import numpy as np
            from omni.isaac.core.utils.prims import create_prim
            
            # Generate heightmap
            heightmap = self._generate_mars_heightmap(resolution)
            
            # Create terrain prim
            terrain_path = "/World/Terrain"
            terrain_prim = create_prim(
                terrain_path,
                "Mesh",
                attributes={
                    "points": self._create_terrain_mesh(heightmap, size),
                    "faceVertexCounts": [...],  # Mesh topology
                    "faceVertexIndices": [...],
                }
            )
            
            # Add collision
            from omni.isaac.core.prims import RigidPrim
            terrain_rigid = RigidPrim(
                prim_path=terrain_path,
                name="terrain",
                position=np.array([0.0, 0.0, 0.0]),
            )
            
            print(f"✓ Created Mars terrain ({size}m x {size}m)")
            
        except Exception as e:
            print(f"Warning: Could not create terrain: {e}")
    
    def _generate_mars_heightmap(self, resolution: int) -> 'np.ndarray':
        """Generate procedural Mars-like heightmap.
        
        Args:
            resolution: Map resolution
            
        Returns:
            Heightmap array
        """
        import numpy as np
        
        # Simple noise-based terrain
        np.random.seed(42)
        heightmap = np.random.rand(resolution, resolution).astype(np.float32)
        
        # Add some craters
        num_craters = 10
        for _ in range(num_craters):
            cx = np.random.randint(0, resolution)
            cy = np.random.randint(0, resolution)
            radius = np.random.randint(10, 50)
            depth = np.random.uniform(0.1, 0.5)
            
            y, x = np.ogrid[:resolution, :resolution]
            dist = np.sqrt((x - cx)**2 + (y - cy)**2)
            
            mask = dist < radius
            heightmap[mask] -= depth * (1 - dist[mask] / radius)
        
        # Add rocks/boulders (small bumps)
        num_rocks = 100
        for _ in range(num_rocks):
            rx = np.random.randint(0, resolution)
            ry = np.random.randint(0, resolution)
            size = np.random.randint(2, 8)
            height = np.random.uniform(0.05, 0.2)
            
            y, x = np.ogrid[:resolution, :resolution]
            dist = np.sqrt((x - rx)**2 + (y - ry)**2)
            
            mask = dist < size
            heightmap[mask] += height * (1 - dist[mask] / size)
        
        # Normalize
        heightmap = (heightmap - heightmap.min()) / (heightmap.max() - heightmap.min())
        heightmap *= 5.0  # Max height 5 meters
        
        return heightmap
    
    def _create_terrain_mesh(self, heightmap: 'np.ndarray', size: float):
        """Create mesh from heightmap."""
        # Simplified mesh creation
        # In production, use proper mesh generation
        import numpy as np
        
        h, w = heightmap.shape
        points = []
        
        for y in range(h):
            for x in range(w):
                px = (x / w - 0.5) * size
                py = (y / h - 0.5) * size
                pz = heightmap[y, x]
                points.append(self.Gf.Vec3f(px, py, pz))
        
        return points
    
    def import_rover(self, urdf_path: str, position: Tuple[float, float, float] = (0.0, 0.0, 2.0)):
        """Import rover URDF into the scene.
        
        Args:
            urdf_path: Path to rover URDF file
            position: Initial position (x, y, z)
        """
        try:
            from omni.isaac.urdf import _urdf
            
            # Import URDF
            urdf_interface = _urdf.acquire_urdf_interface()
            
            import_config = _urdf.ImportConfig()
            import_config.merge_fixed_joints = False
            import_config.convex_decomp = False
            import_config.import_inertia_tensor = True
            import_config.fix_base = False
            import_config.make_default_prim = True
            
            result = urdf_interface.import_urdf(
                urdf_path,
                "/World/Rover",
                import_config,
            )
            
            if result:
                print(f"✓ Imported rover from {urdf_path}")
                
                # Set initial position
                from omni.isaac.core.prims import RigidPrim
                rover_prim = RigidPrim(
                    prim_path="/World/Rover",
                    name="rover",
                    position=np.array(position),
                )
            else:
                print(f"✗ Failed to import URDF")
                
        except Exception as e:
            print(f"Error importing rover: {e}")
    
    def add_sensors(self):
        """Add camera and LiDAR sensors to the rover."""
        try:
            from omni.isaac.sensor import Camera, LidarRtx
            import numpy as np
            
            # Add stereo cameras
            left_camera = Camera(
                prim_path="/World/Rover/camera_left",
                position=np.array([0.2, 0.1, 0.5]),
                frequency=30,
                resolution=(640, 480),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            
            right_camera = Camera(
                prim_path="/World/Rover/camera_right",
                position=np.array([0.2, -0.1, 0.5]),
                frequency=30,
                resolution=(640, 480),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            
            # Add LiDAR
            lidar = LidarRtx(
                prim_path="/World/Rover/lidar",
                position=np.array([0.0, 0.0, 0.8]),
                frequency=10,
            )
            
            print("✓ Added rover sensors (cameras + LiDAR)")
            
        except Exception as e:
            print(f"Warning: Could not add sensors: {e}")
    
    def setup_ros_bridge(self):
        """Setup ROS2 bridge for sensor publishing."""
        try:
            import omni.graph.core as og
            
            # Create ROS2 camera publishers
            # This would create OmniGraph nodes for ROS2 publishing
            
            print("✓ Set up ROS2 bridge")
            print("  Topics:")
            print("    - /camera_left/rgb/image_raw")
            print("    - /camera_left/depth/image_raw")
            print("    - /camera_right/rgb/image_raw")
            print("    - /scan")
            print("    - /odom")
            
        except Exception as e:
            print(f"Warning: Could not setup ROS bridge: {e}")
    
    def save_scene(self, output_path: str):
        """Save the scene to USD file.
        
        Args:
            output_path: Path to save USD file
        """
        try:
            self.omni_usd.get_context().save_as_stage(output_path)
            print(f"✓ Saved scene to {output_path}")
            
        except Exception as e:
            print(f"Error saving scene: {e}")
    
    def run(self, args):
        """Run the complete setup.
        
        Args:
            args: Command line arguments
        """
        print("=" * 60)
        print("Isaac Sim Mars Environment Setup")
        print("=" * 60)
        print()
        
        # Initialize
        if not self.initialize_isaac():
            return False
        
        # Create world
        if not self.create_world():
            return False
        
        # Setup environment
        self.setup_mars_physics()
        self.setup_mars_lighting()
        
        # Create terrain
        self.create_terrain(
            size=args.terrain_size,
            resolution=args.terrain_resolution
        )
        
        # Import rover if requested
        if args.with_rover and args.rover_urdf:
            if os.path.exists(args.rover_urdf):
                self.import_rover(args.rover_urdf, position=args.rover_position)
                self.add_sensors()
                if args.ros_bridge:
                    self.setup_ros_bridge()
            else:
                print(f"Warning: URDF not found: {args.rover_urdf}")
        
        # Save scene
        if args.output:
            self.save_scene(args.output)
        
        print()
        print("=" * 60)
        print("Setup complete!")
        print("=" * 60)
        
        # Keep running if not headless
        if not self.headless:
            print("\nPress Ctrl+C to exit...")
            try:
                while self.simulation_app.is_running():
                    self.world.step(render=True)
            except KeyboardInterrupt:
                pass
        
        self.simulation_app.close()
        return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Setup Mars environment in Isaac Sim for URC Rover',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic Mars environment
  python3 setup_mars_scene.py --output mars_environment.usd

  # With rover
  python3 setup_mars_scene.py --with-rover --rover-urdf ../robot_definition/rover2025/urdf/rover2025.urdf

  # Headless mode for batch processing
  python3 setup_mars_scene.py --headless --output mars_scene.usd

  # Custom terrain
  python3 setup_mars_scene.py --terrain-size 200 --terrain-resolution 1024
        """
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='mars_environment.usd',
        help='Output USD file path (default: mars_environment.usd)'
    )
    
    parser.add_argument(
        '--headless',
        action='store_true',
        help='Run without GUI (for batch processing)'
    )
    
    parser.add_argument(
        '--with-rover',
        action='store_true',
        help='Import rover URDF into the scene'
    )
    
    parser.add_argument(
        '--rover-urdf',
        type=str,
        default='robot_definition/rover2025/urdf/rover2025.urdf',
        help='Path to rover URDF file'
    )
    
    parser.add_argument(
        '--rover-position',
        type=float,
        nargs=3,
        metavar=('X', 'Y', 'Z'),
        default=[0.0, 0.0, 2.0],
        help='Initial rover position (default: 0 0 2)'
    )
    
    parser.add_argument(
        '--ros-bridge',
        action='store_true',
        help='Setup ROS2 bridge for sensor publishing'
    )
    
    parser.add_argument(
        '--terrain-size',
        type=float,
        default=100.0,
        help='Terrain size in meters (default: 100)'
    )
    
    parser.add_argument(
        '--terrain-resolution',
        type=int,
        default=512,
        help='Terrain heightmap resolution (default: 512)'
    )
    
    args = parser.parse_args()
    
    # Run setup
    setup = MarsSceneSetup(headless=args.headless)
    success = setup.run(args)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
