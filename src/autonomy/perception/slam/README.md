# SLAM System - Raspberry Pi 5 + OAK-D + Pi Zero W Optimization Ideas

## Hardware Configuration

**Main Computer**: Raspberry Pi 5 (8GB RAM)
- Primary SLAM processing and navigation
- RTAB-Map RGB-D SLAM orchestration
- GPS/IMU fusion and path planning

**Primary Camera**: OAK-D (Luxonis)
- Stereo RGB-D camera with onboard processing
- 4K RGB camera + stereo depth
- Neural processing unit for edge AI
- Handles initial feature detection and depth processing

**Secondary Camera**: Raspberry Pi Zero W + AI Pi Camera
- Lightweight secondary vision system
- Backup visual odometry
- Environmental monitoring
- Minimal processing load

## Current Implementation

The URC 2026 rover uses RTAB-Map RGB-D SLAM with the following components:

- **SLAM Orchestrator** (`slam_node.py`): Manages RTAB-Map RGB-D SLAM with GPS fusion
- **Depth Processor** (`depth_processor.py`): Specialized filtering for desert environments
- **Configuration**: RTAB-Map tuned for low-feature desert terrain with 500MB memory limit

### Key Features
- RTAB-Map RGB-D SLAM with ORB features (400 max features)
- Bilateral filtering for sand/dust noise reduction
- GPS fusion with EKF for global consistency
- Desert-specific tuning (loop closure thresholds, memory management)

## Hardware-Specific Performance Analysis

### Raspberry Pi 5 (8GB) Capabilities
- **Available Memory**: ~6GB usable (after OS and ROS2 overhead)
- **CPU**: 4x A76 cores @ 2.4GHz + 4x A55 cores @ 2.0GHz
- **Neural Processing**: No dedicated NPU (unlike Jetson)
- **Power**: 27W TDP, passive cooling possible

### OAK-D Capabilities & Optimization Opportunities
- **Onboard Processing**: 4 TOPS neural compute
- **Depth Processing**: Hardware-accelerated stereo matching
- **Feature Detection**: Can offload ORB/SIFT to onboard NPU
- **Network Bandwidth**: USB 3.0 (5Gbps) to Pi 5

### Pi Zero W Constraints
- **Memory**: 512MB total, ~300MB usable
- **CPU**: Single-core ARM1176JZF-S @ 1GHz
- **Network**: WiFi + Bluetooth only
- **Use Case**: Lightweight visual odometry backup only

### Most Expensive Components (in order of impact):

1. **Dense Map Storage** - RTAB-Map point clouds exceed Pi 5 memory when aggressive
2. **Feature Detection on Pi 5** - Should be offloaded to OAK-D NPU
3. **Graph Optimization** - Heavy computation on A76 cores
4. **Dense Depth Processing** - Redundant with OAK-D hardware depth
5. **GPS Fusion** - EKF state estimation manageable on 8GB system

### Current Memory Limits (Too Conservative for Pi 5)
- RTAB-Map: 500MB max memory (can increase to 2-3GB on Pi 5)
- Should leverage OAK-D for preprocessing to reduce Pi 5 load
- Pi Zero W: Keep under 100MB total usage

## Hardware-Optimized SLAM Ideas

### 1. OAK-D + Pi 5 Distributed Processing

#### Leverage OAK-D Onboard Processing
```python
class OAKDSLAMInterface:
    def __init__(self):
        # Use OAK-D for feature detection and initial depth processing
        self.oakd_features = OAKDFeatureDetector(
            model="ORB",  # Offload to OAK-D NPU
            max_features=300,
            confidence_threshold=0.7
        )

        # Pi 5 handles high-level SLAM and fusion
        self.pi5_slam = RTABMapInterface(
            memory_limit_mb=2048,  # Leverage full Pi 5 memory
            enable_loop_closure=True
        )

    def process_frame(self, oakd_data):
        # Features already extracted by OAK-D
        features = oakd_data.features

        # Depth already computed by OAK-D stereo
        depth_map = oakd_data.depth_map

        # Pi 5 does pose estimation and mapping
        pose = self.pi5_slam.track(features, depth_map)

        return pose
```

#### Pi Zero W as Backup VO Node
```python
class PiZeroVisualOdometry:
    def __init__(self):
        # Ultra-lightweight VO for redundancy
        self.memory_limit_mb = 50  # Keep under 100MB total
        self.feature_detector = cv2.FastFeatureDetector_create(
            threshold=20,  # Conservative threshold
            nonmaxSuppression=True
        )
        self.tracker = cv2.TrackerKCF_create()  # Lightweight tracking

    def compute_odometry(self, frame):
        # Minimal processing for backup localization
        keypoints = self.feature_detector.detect(frame)
        # Simple optical flow tracking
        return self.track_movement(keypoints)
```

### 2. Multi-Camera Visual Odometry Hierarchy

#### OAK-D Primary + Pi Zero W Backup
```python
class MultiCameraVO:
    def __init__(self):
        self.primary_vo = OAKDVisualOdometry()  # High-quality VO
        self.backup_vo = PiZeroVisualOdometry()  # Lightweight fallback
        self.fusion = SensorFusion()

    def get_pose_estimate(self):
        primary_pose = self.primary_vo.get_pose()

        if primary_pose.confidence < 0.6:
            # Switch to backup if primary fails
            backup_pose = self.backup_vo.get_pose()
            return self.fusion.fuse_poses(primary_pose, backup_pose)
        else:
            return primary_pose
```

#### Pi Zero W Optimized VO
```python
class PiZeroVisualOdometry:
    def __init__(self):
        self.memory_limit_mb = 50
        self.use_fast_features = True  # FAST instead of ORB
        self.keyframe_distance_threshold = 0.5  # Meters
        self.max_keyframes = 20  # Very limited history

    def track_frame(self, frame):
        # Minimal feature detection
        features = cv2.FAST(frame, threshold=20, nonmaxSuppression=True)

        # Simple motion estimation
        motion = self.estimate_motion_2d(features)

        # Update pose with minimal state
        self.update_pose(motion)

        return self.current_pose
```

#### Feature Processing Distribution
- **OAK-D**: ORB/SIFT feature extraction (300 features)
- **Pi 5**: Feature matching and pose optimization
- **Pi Zero W**: FAST corner detection only (50-100 features)

### 3. Pi 5 Memory-Optimized Map Representations

#### Leverage 8GB RAM Effectively
```python
class Pi5SLAMMap:
    def __init__(self):
        self.memory_limit_mb = 3072  # Use 3GB for mapping on Pi 5
        self.oakd_preprocessed = True  # OAK-D handles initial processing

        # Optimized data structures for Pi 5
        self.keyframe_map = {}  # Hash map for fast keyframe lookup
        self.pose_graph = SparsePoseGraph()  # Efficient graph representation
        self.feature_cloud = CompressedFeatureCloud()  # Compressed storage

    def add_keyframe(self, keyframe_data):
        # OAK-D provides pre-filtered features and depth
        if self.should_add_keyframe(keyframe_data):
            compressed_frame = self.compress_keyframe(keyframe_data)
            self.keyframe_map[keyframe_data.id] = compressed_frame
            self.pose_graph.add_node(compressed_frame)
```

#### OAK-D + Pi 5 Hybrid Mapping
```python
class DistributedSLAMMap:
    def __init__(self):
        # OAK-D handles real-time processing
        self.oakd_processor = OAKDDepthProcessor(
            resolution="720p",  # Balance quality vs bandwidth
            framerate=30
        )

        # Pi 5 handles global mapping
        self.pi5_mapper = RTABMapPi5(
            memory_limit_mb=2048,
            input_from_oakd=True  # Expect preprocessed data
        )

    def process_sensor_data(self):
        # OAK-D: Feature detection + depth computation
        oakd_features = self.oakd_processor.get_features()
        oakd_depth = self.oakd_processor.get_depth()

        # Pi 5: Pose estimation + global mapping
        pose = self.pi5_mapper.track(oakd_features, oakd_depth)

        return pose
```

#### Memory Management for 8GB System
- **Keyframe Storage**: 1-2GB for recent keyframes
- **Pose Graph**: 500MB for optimization
- **Feature Database**: 500MB compressed storage
- **Leave 2-3GB** for ROS2, navigation, and other processes

### 4. OAK-D + Multi-Pi Computation Distribution (With Complexity Management)

#### Addressing Redundancy Complexity
**Valid Concern**: Multiple localization systems can clash and increase complexity. Here's how to manage it:

**Problems to Avoid:**
- Conflicting pose estimates causing oscillation
- Network overhead between devices
- Resource competition
- Synchronization issues
- False failure detection

**Solutions:**
- Hierarchical arbitration (GPS → OAK-D SLAM → Pi Zero VO)
- Sensor fusion with confidence weighting
- Independent operation with occasional synchronization
- Clear failure boundaries and fallbacks

#### OAK-D → Pi 5 → Pi Zero W Processing Pipeline
```python
class DistributedSLAMPipeline:
    def __init__(self):
        # OAK-D: Low-level vision processing
        self.oakd_processor = OAKDSLAMProcessor(
            neural_inference=True,  # Use onboard NPU
            stereo_depth=True,      # Hardware depth
            feature_extraction="ORB" # Offload feature detection
        )

        # Pi 5: High-level SLAM and fusion
        self.pi5_slam = RTABMapPi5(
            memory_limit_mb=2048,
            input_preprocessed=True  # From OAK-D
        )

        # Pi Zero W: Backup localization
        self.pizero_backup = PiZeroVisualOdometry(
            memory_limit_mb=50
        )

        # Inter-device communication
        self.network_bridge = PiNetworkBridge()

    def process_frame_distributed(self):
        # Step 1: OAK-D processes raw camera data
        oakd_result = self.oakd_processor.process_frame()
        # Returns: features, depth_map, imu_data

        # Step 2: Send to Pi 5 for SLAM
        slam_pose = self.pi5_slam.track(oakd_result)

        # Step 3: Pi Zero W runs backup VO if needed
        if slam_pose.confidence < 0.5:
            backup_pose = self.network_bridge.get_pi_zero_pose()
            slam_pose = self.fuse_poses(slam_pose, backup_pose)

        return slam_pose
```

#### Managing Redundancy Complexity

##### Hierarchical Arbitration System
```python
class LocalizationArbitrator:
    def __init__(self):
        # Priority hierarchy: GPS > OAK-D SLAM > Pi Zero VO
        self.systems = {
            'gps': {'priority': 1, 'confidence_threshold': 0.8},
            'oakd_slam': {'priority': 2, 'confidence_threshold': 0.6},
            'pizero_vo': {'priority': 3, 'confidence_threshold': 0.4}
        }

        self.current_primary = 'gps'
        self.last_switch_time = time.time()

    def arbitrate_pose(self, poses_dict):
        """
        Choose which localization system to trust based on:
        - GPS accuracy and availability
        - SLAM confidence and consistency
        - System health and agreement
        """
        # Check GPS first (highest priority)
        if self._is_gps_reliable(poses_dict.get('gps')):
            return self._switch_to_system('gps', poses_dict['gps'])

        # Check OAK-D SLAM
        if self._is_slam_confident(poses_dict.get('oakd_slam')):
            return self._switch_to_system('oakd_slam', poses_dict['oakd_slam'])

        # Fallback to Pi Zero VO
        if poses_dict.get('pizero_vo'):
            return self._switch_to_system('pizero_vo', poses_dict['pizero_vo'])

        # Total failure - use last known good pose
        return self._use_last_known_pose()

    def _switch_to_system(self, system_name, pose):
        """Handle system switching with hysteresis to prevent oscillation"""
        if system_name != self.current_primary:
            # Prevent rapid switching (hysteresis)
            if time.time() - self.last_switch_time > 2.0:  # 2 second minimum
                self.current_primary = system_name
                self.last_switch_time = time.time()
                logger.info(f"Switched to {system_name} localization")

        return pose
```

##### Independent Operation with Loose Coupling
```python
class LooseCouplingManager:
    def __init__(self):
        # Systems operate independently, synchronize occasionally
        self.sync_interval = 5.0  # Sync every 5 seconds
        self.last_sync = time.time()

        # Each system maintains its own state
        self.system_states = {
            'oakd': {'pose': None, 'confidence': 0.0, 'last_update': 0},
            'pizero': {'pose': None, 'confidence': 0.0, 'last_update': 0}
        }

    def update_system_pose(self, system_name, pose, confidence):
        """Update system pose without immediate global impact"""
        self.system_states[system_name] = {
            'pose': pose,
            'confidence': confidence,
            'last_update': time.time()
        }

    def should_sync_now(self):
        """Periodic sync to maintain consistency"""
        return time.time() - self.last_sync > self.sync_interval

    def perform_sync(self):
        """Synchronize systems when needed"""
        if not self.should_sync_now():
            return

        # Check for gross disagreements
        oakd_pose = self.system_states['oakd']['pose']
        pizero_pose = self.system_states['pizero']['pose']

        if oakd_pose and pizero_pose:
            distance = self._pose_distance(oakd_pose, pizero_pose)
            if distance > 5.0:  # 5 meter disagreement
                logger.warning(f"System disagreement: {distance:.2f}m")
                # Handle disagreement (reset, recalibrate, etc.)

        self.last_sync = time.time()
```

##### Resource Partitioning
```python
class ResourcePartitioner:
    def __init__(self):
        # Prevent resource competition
        self.device_limits = {
            'oakd': {'cpu_percent': 70, 'memory_mb': 256},
            'pi5': {'cpu_percent': 80, 'memory_mb': 4096},
            'pizero': {'cpu_percent': 60, 'memory_mb': 100}
        }

        self.network_limits = {
            'usb_bandwidth_mbps': 3000,  # OAK-D to Pi 5
            'wifi_bandwidth_mbps': 50    # Pi Zero to Pi 5
        }

    def enforce_limits(self):
        """Monitor and enforce resource limits"""
        for device, limits in self.device_limits.items():
            if self._check_resource_usage(device) > limits:
                self._throttle_device(device)
```

#### When Redundancy Helps vs Hurts

**Redundancy Benefits (When Done Right):**
- GPS outages don't stop navigation (Mars-like conditions)
- Hardware failures are masked
- Better accuracy through sensor fusion
- Gradual degradation instead of sudden failure

**Redundancy Costs (When Done Wrong):**
- Increased complexity and debugging difficulty
- Network reliability requirements
- Resource overhead even when not needed
- Potential for conflicting decisions

**Recommendation: Start Simple, Add Complexity Gradually**
1. **Phase 1**: Pi 5 + OAK-D only (single reliable system)
2. **Phase 2**: Add Pi Zero VO as independent backup (loose coupling)
3. **Phase 3**: Add arbitration and fusion only when needed

### **Alternative: Simpler Robustness Approaches**

If redundancy complexity is a major concern, consider these alternatives:

#### **Single System with Superior Reliability**
```python
class RobustSingleSystem:
    def __init__(self):
        # One excellent system instead of three mediocre ones
        self.oakd_slam = OAKDSLAM()  # High-quality primary
        self.memory_mb = 3072  # Use full 3GB on Pi 5
        self.backup_gps_only = True  # Simple GPS fallback

    # Focus effort on making one system bulletproof
    def robust_error_handling(self):
        # Extensive error recovery
        # Quality monitoring
        # Self-diagnosis and repair
```

#### **Quality over Quantity**
- Invest time in making OAK-D + Pi 5 system exceptionally reliable
- Use proven algorithms (RTAB-Map) with extensive testing
- Add monitoring and self-healing capabilities
- Focus on preventing failures rather than recovering from them

#### **Progressive Enhancement**
Start with: **Pi 5 + OAK-D only**
- If that's reliable enough → Stop there
- If not → Add simple Pi Zero backup
- Only add fusion/arbitration if independent systems aren't sufficient

#### Device-Specific Task Allocation
```python
class HardwareTaskAllocator:
    def __init__(self):
        self.device_capabilities = {
            'oakd': {
                'cpu': '4 TOPS NPU',
                'memory': '512MB onboard',
                'tasks': ['feature_detection', 'depth_computation', 'stereo_matching']
            },
            'pi5': {
                'cpu': 'Cortex-A76/A55',
                'memory': '8GB',
                'tasks': ['slam_tracking', 'pose_optimization', 'map_management']
            },
            'pizero': {
                'cpu': 'ARM1176JZF-S',
                'memory': '512MB',
                'tasks': ['backup_vo', 'simple_tracking']
            }
        }

    def allocate_task(self, task_type, data):
        if task_type in ['feature_detection', 'depth']:
            return self.send_to_oakd(data)
        elif task_type in ['slam', 'optimization']:
            return self.send_to_pi5(data)
        elif task_type == 'backup_localization':
            return self.send_to_pi_zero(data)
```

### 5. Hardware-Specific Optimizations

#### Raspberry Pi 5 Optimizations (8GB RAM)
```python
class Pi5SLAMOptimizations:
    def __init__(self):
        # Leverage Pi 5's multi-core architecture
        self.thread_pool = ThreadPoolExecutor(max_workers=4)

        # NEON SIMD acceleration for ARM64
        import cv2
        cv2.setUseOptimized(True)
        cv2.setNumThreads(4)  # Pi 5 has 4 performance cores

        # Memory management for 8GB system
        self.memory_manager = Pi5MemoryManager(
            total_memory_gb=8,
            slam_allocation_gb=3,  # 3GB for SLAM
            safety_buffer_gb=1     # 1GB safety margin
        )

    def optimize_computation(self):
        # Use A76 cores for heavy SLAM computation
        # Use A55 cores for I/O and networking
        self.set_cpu_affinity()

        # Enable large page support for better memory performance
        self.configure_large_pages()
```

#### OAK-D Hardware Acceleration + IMU for VIO
```python
class OAKDVIOOptimizations:
    def __init__(self):
        # Leverage OAK-D's 4 TOPS neural compute + onboard IMU
        self.depthai_pipeline = depthai.Pipeline()

        # Configure for SLAM preprocessing + IMU
        self.configure_neural_network()
        self.configure_imu_for_vio()
        self.optimize_usb_bandwidth()

    def configure_imu_for_vio(self):
        # Enable OAK-D's onboard IMU for VIO SLAM
        imu = self.depthai_pipeline.createIMU()
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)  # 400Hz accel
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)      # 400Hz gyro
        imu.setBatchReportThreshold(1)  # Low latency
        imu.setMaxBatchReports(10)      # Small batches

        # Link IMU to XLink output
        imu_xlink = self.depthai_pipeline.createXLinkOut()
        imu_xlink.setStreamName("imu")
        imu.out.link(imu_xlink.input)

    def get_vio_data(self):
        """Get synchronized visual + inertial data for VIO SLAM"""
        # OAK-D provides:
        # - Features (ORB/SIFT via neural processing)
        # - Depth map (hardware stereo)
        # - IMU data (accelerometer + gyroscope @ 400Hz)
        # All timestamped and synchronized
        return {
            'features': self.oakd_features,
            'depth': self.depth_map,
            'imu_data': self.imu_queue.get()  # Accel + gyro readings
        }

    def configure_neural_network(self):
        # Use OAK-D's neural processing for feature detection
        nn = self.depthai_pipeline.createNeuralNetwork()
        nn.setBlobPath("models/orb_features.blob")
        nn.setNumInferenceThreads(2)  # Balance with depth processing

    def optimize_usb_bandwidth(self):
        # Maximize USB 3.0 throughput for depth/features/IMU
        config = dai.Device.Config()
        config.board.usb.maxSpeed = dai.UsbSpeed.SUPER_PLUS
        # 5Gbps theoretical, ~3.5Gbps practical
```

##### Getting IMU Data from OAK-D for VIO SLAM

**Hardware Setup:**
```python
import depthai as dai

# Create pipeline with IMU
pipeline = dai.Pipeline()

# Configure IMU sensor
imu = pipeline.createIMU()
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)  # 400Hz
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)     # 400Hz

# Create output link
imu_xlink = pipeline.createXLinkOut()
imu_xlink.setStreamName("imu")
imu.out.link(imu_xlink.input)

# Start device and get queue
with dai.Device(pipeline) as device:
    imu_queue = device.getOutputQueue("imu", maxSize=50, blocking=False)

    while True:
        imu_data = imu_queue.get()  # Get IMU packets
        for imu_packet in imu_data.packets:
            accel = imu_packet.acceleroMeter
            gyro = imu_packet.gyroscope

            # Process accel/gyro for VIO
            process_vio_data(accel, gyro, timestamp)
```

**ROS2 Integration:**
```python
# Publish IMU data as ROS2 sensor_msgs/Imu
def publish_oakd_imu(imu_queue, imu_publisher):
    imu_data = imu_queue.get()
    for packet in imu_data.packets:
        imu_msg = Imu()
        imu_msg.header.stamp = get_timestamp(packet)

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = packet.acceleroMeter.x
        imu_msg.linear_acceleration.y = packet.acceleroMeter.y
        imu_msg.linear_acceleration.z = packet.acceleroMeter.z

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = packet.gyroscope.x
        imu_msg.angular_velocity.y = packet.gyroscope.y
        imu_msg.angular_velocity.z = packet.gyroscope.z

        imu_publisher.publish(imu_msg)
```

**VIO SLAM Integration:**
```python
class OAKDVIO:
    def __init__(self):
        self.visual_frontend = OAKDVisualFrontend()  # Features + depth
        self.imu_processor = IMUProcessor()          # IMU preintegration
        self.vio_backend = VIOBackend()              # Optimization

    def process_vio_frame(self):
        # Get synchronized visual + inertial data
        vio_data = self.visual_frontend.get_vio_data()

        # Preintegrate IMU measurements between frames
        imu_preintegration = self.imu_processor.preintegrate(vio_data.imu_data)

        # Update VIO state
        pose_estimate = self.vio_backend.update(
            vio_data.features,
            vio_data.depth,
            imu_preintegration
        )

        return pose_estimate
```

**Key Benefits for VIO:**
- **Hardware synchronization**: IMU and camera timestamps are perfectly aligned
- **High frequency**: 400Hz IMU vs 30Hz camera = smooth motion estimates
- **Low latency**: IMU data available immediately, not dependent on image processing
- **Power efficient**: IMU runs continuously with minimal power draw
- **Robust initialization**: IMU provides scale and gravity direction for monocular VIO

### Alternative VIO SLAM Algorithms for OAK-D + Pi 5

#### VINS-Fusion (Recommended for Edge Devices)
```python
# Lightweight VIO optimized for edge hardware
class VINSFusionOAKD:
    def __init__(self):
        self.config = {
            'imu_freq': 400,      # Match OAK-D IMU frequency
            'cam_freq': 30,       # OAK-D camera rate
            'image_width': 1280,  # OAK-D resolution
            'image_height': 720,
            'max_solver_time': 0.04,  # Fast convergence for real-time
            'max_num_iterations': 8,  # Limited iterations for speed
        }

    def process_oakd_data(self, oakd_data):
        # VINS-Fusion expects:
        # - IMU measurements (accel + gyro)
        # - Camera images (mono or stereo)
        # - Camera intrinsics and extrinsics

        vio_result = self.vins_estimator.processIMU(oakd_data.imu_data)
        vio_result = self.vins_estimator.processImage(
            oakd_data.left_image,
            oakd_data.right_image,  # Optional stereo
            oakd_data.timestamp
        )

        return vio_result.pose
```

**VINS-Fusion Advantages:**
- **Memory efficient**: ~50-100MB RAM usage
- **Real-time performance**: Processes at camera frame rate
- **Robust initialization**: Works well with OAK-D's synchronized IMU+camera
- **GPS fusion ready**: Can integrate with GNSS for global consistency

#### OKVIS (Alternative Lightweight VIO)
```python
class OKVISOAKD:
    def __init__(self):
        self.okvis_estimator = okvis::Estimator()

        # Configure for OAK-D hardware
        self.camera_params = {
            'fu': 800, 'fv': 800,  # Focal length from OAK-D calibration
            'cu': 640, 'cv': 360,  # Principal point
            'T_SC': np.eye(4)      # IMU-camera transformation
        }

    def add_imu_measurement(self, imu_data):
        # OKVIS IMU integration
        imu_measurement = okvis::ImuMeasurement(
            imu_data.timestamp,
            okvis::ImuSensorReadings(
                imu_data.accel, imu_data.gyro
            )
        )
        self.okvis_estimator.addImuMeasurement(imu_measurement)

    def add_image_measurement(self, image, timestamp):
        # OKVIS camera integration
        self.okvis_estimator.addImageMeasurement(
            timestamp, image, self.camera_index
        )
```

#### ORB-SLAM3 VIO Mode
```python
# ORB-SLAM3 with IMU integration
class ORBSLAM3VIO:
    def __init__(self):
        self.slam = ORB_SLAM3::System(
            vocabulary_file,
            settings_file,
            ORB_SLAM3::System::IMU_MONOCULAR  # Use IMU mode
        )

    def track_with_imu(self, image, imu_data, timestamp):
        # ORB-SLAM3 IMU integration
        pose = self.slam.TrackMonocular(image, imu_data, timestamp)
        return pose
```

### VIO SLAM vs Visual-Only SLAM Comparison

| Aspect | Visual-Only SLAM | VIO SLAM (with OAK-D IMU) |
|--------|------------------|---------------------------|
| **Memory Usage** | 500MB-2GB | 50-200MB |
| **CPU Usage** | High (feature tracking) | Moderate (IMU preintegration) |
| **Motion Blur Robustness** | Poor | Excellent (IMU provides motion) |
| **Scale Drift** | High (monocular) | Low (IMU provides scale) |
| **Initialization** | Requires motion | Robust with IMU gravity vector |
| **GPS Integration** | Complex | Natural (complementary sensors) |
| **Power Efficiency** | High | Very high (IMU low power) |

### Recommended VIO Setup for Your Hardware

**Primary System: VINS-Fusion on Pi 5 with OAK-D IMU**
- Use OAK-D IMU (400Hz) + stereo camera (30Hz)
- VINS-Fusion backend on Pi 5 (optimized for ARM64)
- Memory usage: ~100MB
- Real-time performance on Pi 5

**Backup System: OKVIS on Pi Zero W**
- Lightweight VIO for redundancy
- Minimal memory footprint (<50MB)
- Independent operation
- WiFi synchronization with Pi 5

**Development Priority:**
1. Get VINS-Fusion working with OAK-D IMU
2. Add OKVIS backup on Pi Zero W
3. Integrate with RTAB-Map for mapping when GPS available

#### Pi Zero W Lightweight Optimizations
```python
class PiZeroOptimizations:
    def __init__(self):
        # Extreme memory constraints
        self.memory_limit_mb = 100
        self.cpu_limit_percent = 50  # Don't saturate the single core

        # Minimal computer vision
        self.use_fastest_algorithms = True

    def lightweight_vo_pipeline(self):
        # FAST feature detection (very fast)
        self.feature_detector = cv2.FastFeatureDetector_create(
            threshold=25,  # Higher threshold = fewer features
            nonmaxSuppression=True
        )

        # Lucas-Kanade optical flow (efficient tracking)
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
```

#### Multi-Device Memory Pool Management
```python
class DistributedMemoryManager:
    def __init__(self):
        self.device_memory = {
            'oakd': {'total_mb': 512, 'slam_mb': 256},
            'pi5': {'total_mb': 8192, 'slam_mb': 3072},
            'pizero': {'total_mb': 512, 'slam_mb': 50}
        }

        # Network-aware memory allocation
        self.network_bandwidth_mbps = 1000  # USB 3.0 theoretical

    def allocate_memory_distributed(self, task_requirements):
        # Allocate memory across devices based on task needs
        allocations = {}
        for device, memory in self.device_memory.items():
            if memory['slam_mb'] > task_requirements[device]:
                allocations[device] = task_requirements[device]
            else:
                allocations[device] = memory['slam_mb']

        return allocations
```

### 6. Alternative SLAM Algorithms

#### LSD-SLAM (Direct Method)
- More memory efficient than feature-based methods
- Better for low-texture environments like deserts
- Direct optimization of photometric error

#### OKVIS (Optimized Kalman Filter Visual Inertial SLAM)
- Tightly coupled visual-inertial odometry
- Lower memory footprint than graph-based SLAM
- Good for GPS-denied environments

#### VINS-Mono/Fusion
- Lightweight visual-inertial SLAM
- Minimal memory usage
- Excellent for edge devices

### 7. Navigation Integration Optimizations

#### Hierarchical Path Planning
```python
class EdgeNavigation:
    def __init__(self):
        self.global_planner = SparseAStar()  # Coarse waypoints only
        self.local_planner = VectorField()   # Reactive obstacle avoidance
        self.terrain_cache = LRUCache(maxsize=100)  # Limited terrain data

    def plan_path(self, start, goal):
        # Phase 1: Coarse global planning (memory efficient)
        waypoints = self.global_planner.plan_sparse(start, goal, step_size=5.0)

        # Phase 2: Local refinement (on-demand)
        refined_path = self.local_planner.refine_locally(waypoints)

        return refined_path
```

#### Reactive Navigation Fallback
- Vector Field Histogram (VFH) for local obstacle avoidance
- Dynamic Window Approach (DWA) for motion planning
- No global map required, works with local sensor data only

### 8. Real-time Resource Monitoring

#### Adaptive Quality Scaling
```python
class ResourceMonitor:
    def __init__(self):
        self.memory_thresholds = [0.5, 0.7, 0.9]  # 50%, 70%, 90%
        self.adaptation_strategies = [
            self.reduce_map_resolution,
            self.disable_loop_closure,
            self.use_visual_odom_only
        ]

    def monitor_and_adapt(self):
        memory_percent = psutil.virtual_memory().percent
        for i, threshold in enumerate(self.memory_thresholds):
            if memory_percent > threshold * 100:
                self.adaptation_strategies[i]()
                break
```

## Implementation Roadmap: Start Simple, Add Redundancy Gradually

### Phase 1: Single Reliable System (Week 1-2) - **AVOID COMPLEXITY**
- [ ] Set up OAK-D depthai pipeline for feature detection
- [ ] Implement OAK-D → Pi 5 data streaming (USB only)
- [ ] Modify depth_processor.py for OAK-D input
- [ ] Test RTAB-Map with OAK-D preprocessed data
- [ ] Benchmark Pi 5 SLAM performance with 8GB RAM
- **Goal**: Get one robust SLAM system working reliably

### Phase 2: Basic Redundancy Without Complexity (Week 3-4) - **LOOSE COUPLING**
- [ ] Implement independent Pi Zero W VO (no fusion yet)
- [ ] Set up one-way WiFi communication (Pi Zero → Pi 5 status only)
- [ ] Pi Zero W runs completely independently
- [ ] Monitor both systems but use only primary (OAK-D + Pi 5)
- [ ] Test independent operation without interference
- **Goal**: Two systems running without interaction

### Phase 3: Simple Arbitration (Week 5-6) - **CONTROLLED COMPLEXITY**
- [ ] Add basic primary/backup switching logic
- [ ] Implement confidence-based arbitration (no complex fusion)
- [ ] Test failover scenarios with manual verification
- [ ] Add resource monitoring to prevent conflicts
- [ ] Validate that systems don't interfere when both running
- **Goal**: Simple switching without sensor fusion complexity

### Phase 4: Advanced Fusion (Week 7-8) - **ADD COMPLEXITY ONLY WHEN NEEDED**
- [ ] Implement sensor fusion with confidence weighting
- [ ] Add loose coupling with periodic synchronization
- [ ] Test in GPS-denied scenarios
- [ ] Performance profiling with all systems active
- [ ] Validate that complexity provides real benefits
- **Goal**: Full redundancy with controlled complexity

### Phase 5: Competition Hardening (Week 9-10)
- [ ] Desert environment testing with all systems
- [ ] GPS-denied operation validation
- [ ] Recovery mechanisms for multiple failure modes
- [ ] Power consumption optimization across devices
- [ ] Thermal management for sustained operation
- **Goal**: Reliable multi-modal localization for competition

## Hardware-Specific Performance Targets

### Raspberry Pi 5 (8GB)
- **Memory Usage**: 3-4GB peak (SLAM + navigation)
- **CPU Usage**: <60% sustained (with OAK-D offload)
- **Frame Rate**: 15-20 FPS for SLAM processing
- **Power**: <25W under load

### OAK-D Camera
- **Feature Detection**: 300 features @ 30 FPS
- **Depth Quality**: 720p stereo depth
- **Neural Processing**: <50% NPU utilization
- **USB Bandwidth**: <2Gbps sustained

### Pi Zero W + AI Pi Camera
- **Memory Usage**: <100MB total
- **CPU Usage**: <40% for VO processing
- **Frame Rate**: 10-15 FPS backup
- **Network**: Reliable WiFi connection to Pi 5

## Expected Performance Gains vs Current System

- **Total System Memory**: 200% increase (500MB → 3-4GB usable)
- **Processing Distribution**: 70% offload from Pi 5 to OAK-D
- **Redundancy**: 3x reliability with backup systems
- **Power Efficiency**: Better distribution across devices
- **Localization Quality**: 95% of desktop performance in ideal conditions

## When Redundancy is Worth the Complexity

### **Situations Where Multiple Systems Pay Off:**
- **GPS-denied environments** (Mars, underground, urban canyons)
- **Critical safety systems** where single failure = mission failure
- **Extended autonomous operation** (hours/days)
- **Unknown terrain** requiring robust perception
- **Competition scenarios** with high reliability requirements

### **Situations Where Single System is Better:**
- **Constrained development timeline** (< 4 weeks)
- **Limited testing resources** (can't validate complex interactions)
- **GPS-available environments** (Earth-based navigation)
- **Simple, well-mapped courses**
- **When primary system is already highly reliable**

### **Decision Framework:**
```
If (mission_critical_failure_risk > 0.1) AND (development_time > 6_weeks):
    Implement redundant systems with careful complexity management
Else:
    Focus on making single system exceptionally reliable
```

## Competition-Ready Features

- **Multi-modal Localization**: GPS + OAK-D SLAM + Pi Zero VO
- **Graceful Degradation**: Automatic fallback on sensor failure
- **Network Resilience**: Pi Zero W operates independently
- **Real-time Monitoring**: Resource usage tracking across devices
- **Desert Optimization**: Sand/dust filtering for Mars-like terrain

## Testing & Validation for Distributed Hardware

### Multi-Device Integration Testing
- **OAK-D ↔ Pi 5 Communication**: USB 3.0 bandwidth and latency
- **Pi 5 ↔ Pi Zero W Network**: WiFi reliability and failover
- **Sensor Synchronization**: Timestamp alignment across devices
- **Resource Allocation**: Memory/CPU distribution under load

### Hardware-Specific Benchmarks
#### OAK-D Performance
- Feature detection accuracy vs computational load
- Depth map quality at different resolutions
- Neural inference latency (ORB features)
- USB bandwidth utilization

#### Raspberry Pi 5 Benchmarks
- SLAM processing with preprocessed OAK-D input
- Memory usage scaling with map size
- CPU utilization across A76/A55 cores
- Thermal performance under sustained load

#### Pi Zero W Benchmarks
- Visual odometry accuracy with limited resources
- Memory usage stability (<100MB)
- Network reliability for pose transmission
- CPU temperature under continuous operation

### Competition Scenario Testing
- **Desert Navigation**: Sand/dust environment simulation
- **GPS-Denied Operation**: Extended periods without GPS
- **Sensor Failure Recovery**: OAK-D failure → Pi Zero backup
- **Multi-hour Autonomy**: Sustained operation testing
- **Thermal Management**: Performance in hot conditions

### Performance Metrics Dashboard
```python
class DistributedSLAMMonitor:
    def __init__(self):
        self.metrics = {
            'oakd': ['feature_count', 'depth_fps', 'neural_load'],
            'pi5': ['memory_usage', 'cpu_percent', 'slam_fps'],
            'pizero': ['vo_memory', 'network_latency', 'backup_fps']
        }

    def generate_competition_report(self):
        # Generate comprehensive performance report
        # for URC competition requirements
        pass
```

## Related Files
- `slam_node.py`: Current RTAB-Map orchestrator
- `depth_processor.py`: Depth filtering for desert environments
- `config/rtabmap_desert.yaml`: Current RTAB-Map configuration
- `config/gps_fusion.yaml`: GPS-SLAM fusion parameters

## Hardware-Specific References

### OAK-D (Luxonis) Resources
- [DepthAI Documentation](https://docs.luxonis.com/)
- [OAK-D SLAM Examples](https://github.com/luxonis/depthai-experiments)
- [Neural Inference Models](https://github.com/luxonis/depthai-model-zoo)
- [USB Optimization Guide](https://docs.luxonis.com/projects/api/en/latest/tutorials/usb_bandwidth/)

### Raspberry Pi Resources
- [Pi 5 Performance Guide](https://www.raspberrypi.com/news/raspberry-pi-5-performance/)
- [64-bit ARM Optimization](https://developer.arm.com/documentation/102374/latest/)
- [NEON SIMD Programming](https://developer.arm.com/architectures/instruction-sets/intrinsic-functions)
- [Pi Zero W Networking](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#raspberry-pi-zero-w)

### SLAM Algorithm References
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [LSD-SLAM](https://github.com/tum-vision/lsd_slam)
- [OKVIS](https://github.com/ethz-asl/okvis)

### Distributed Systems
- [ROS2 Multi-Machine Setup](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Introducing-ros2-security.html)
- [Real-time Linux on Pi](https://docs.ros.org/en/humble/Installation/Alternatives/RMW-Implementation.html)
- [Network-Aware ROS2](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)
