# Pillar 1: Perception Systems - Educational Presentation

## What is Perception in Robotics?

### The Analogy: A Human's Eyes & Brain

Think about how YOU navigate the world:
1. Your **eyes capture visual information** (light, colors, shapes)
2. Your **brain processes** this information (recognizes objects, understands distance)
3. Your **brain makes decisions** based on what you see (avoid obstacles, find targets)

A robot's **perception system** does the same thing - but with cameras, sensors, and computer algorithms instead of eyes and a biological brain.

### Core Perception Tasks

```
SENSE → PROCESS → UNDERSTAND → ACT

Real World          Camera/Sensors          Algorithms          Robot Decision
Lunar rocks    →    RGB Image        →    Computer Vision   →    "That's a sample!"
Distance to        LiDAR/Depth Data   →    SLAM Algorithm    →    "I'm at position X,Y"
objects ahead
Satellite position GPS Data           →    Sensor Fusion     →    "My true location is..."
```

---

## 1. Computer Vision: Seeing the World

### What is Computer Vision?

Computer vision is the process of having a computer **"understand" images**.

### The Workflow

```
1. CAPTURE IMAGE
   - Camera takes picture
   - Image is digital data (pixels with colors)
   
2. PREPROCESSING
   - Convert to useful format (RGB → HSV for color detection)
   - Remove noise, enhance features
   
3. DETECTION / RECOGNITION
   - Find specific objects (ArUco markers, samples)
   - Determine their position and size
   
4. OUTPUT INFORMATION
   - "I found a red marker at position (x, y) in the image"
   - "The object is 50 pixels from the left, 100 pixels from top"
```

### In the URC Mars Rover

**Real-World Challenge**: Find and collect samples, identify equipment on Mars

**Computer Vision Solution**:

```python
# Pseudo-code for vision pipeline
def find_samples_in_image(image):
    # 1. Convert image to HSV color space
    hsv_image = convert_to_hsv(image)
    
    # 2. Find red/orange colored regions (samples)
    # Look for pixels where: hue is red, saturation is high, value is bright
    sample_mask = (hsv_image.H in [0-10, 170-180]) & (hsv_image.S > 100) & (hsv_image.V > 100)
    
    # 3. Find connected regions (objects)
    objects = find_contours(sample_mask)
    
    # 4. Filter by size (only real samples, not noise)
    real_samples = [obj for obj in objects if obj.area > MIN_SIZE]
    
    # 5. Calculate position of each sample in image coordinates
    positions = [get_center(obj) for obj in real_samples]
    
    return positions
```

### Two Approaches: Light vs. Smart

**Lightweight Approach (Color Detection)**:
- How: Look for specific colors (red, blue, etc.)
- Speed: Very fast (20-30 ms per frame)
- Accuracy: ~70-85% (depends on lighting)
- Power: Minimal CPU usage
- Use case: Simple objects with distinct colors (colored markers, samples)

**Smart Approach (Machine Learning)**:
- How: Neural network trained on thousands of images
- Speed: Slower (100-200 ms per frame)
- Accuracy: 90%+ (handles variations, lighting, angles)
- Power: High CPU/GPU usage
- Use case: Complex objects (equipment, tools, terrain hazards)

**URC Strategy**: Use lightweight detection during normal operation (fast), fall back to ML when lightweight fails.

### Key Technologies in URC

| Technology | What It Does | Where It's Used |
|-----------|-------------|-----------------|
| **ArUco Markers** | Fiducial markers (like QR codes) that robots can reliably find | Localization, equipment identification |
| **YOLO (You Only Look Once)** | Real-time object detection neural network | Detecting samples, equipment, obstacles |
| **OpenCV** | Library of computer vision algorithms | Image processing, contour detection, ArUco |
| **HSV Color Space** | Better for color detection than RGB | Finding colored objects |

---

## 2. SLAM: Understanding "Where Am I?"

### What is SLAM?

**SLAM = Simultaneous Localization And Mapping**

It's like navigating a new city without GPS:
- **Localization**: "Where am I right now?"
- **Mapping**: "What does this area look like?"
- **Simultaneous**: Do both at the same time without prior knowledge

### The Challenge

Imagine exploring a cave:
- You enter with no map
- Your GPS doesn't work underground
- You need to figure out where you are AND draw a map as you move

```
START                         SLAM RESULT
  ↓                               ↓
┌─────────┐         →       ┌──────────────────┐
│ I'm lost│              │ Map:              │
│ Where?  │              │ ┌──────────────┐  │
└─────────┘              │ │              │  │
                         │ │  ← You here  │  │
                         │ │              │  │
                         │ └──────────────┘  │
                         └──────────────────┘
```

### How SLAM Works

```
STEP 1: CAPTURE SENSOR DATA
└─ Camera images
└─ LiDAR (laser distance measurements)
└─ Depth sensors

STEP 2: EXTRACT FEATURES
└─ Identify stable features in images (corners, edges)
└─ Keep track of how these features move between frames

STEP 3: MOTION ESTIMATION
└─ "I saw feature X at position A, then at position B"
└─ Calculate how far I moved and in what direction

STEP 4: MAP BUILDING
└─ Add new features to my map
└─ Update their 3D positions based on my movement

STEP 5: LOOP CLOSURE
└─ "Wait, I recognize this place! I've been here before!"
└─ Correct accumulated errors in the map
```

### Real-World Example

```
Frame 1: Robot sees tree at (0, 0)
Frame 2: Tree is now at (-5, -5) → Robot moved forward 5m
Frame 3: Robot sees new rock at (10, 0)
...
Frame 100: Robot sees that same tree again at (-5, 5)
           But based on odometry it should be at (-5, 7)
           LOOP CLOSURE: Correct the error in the map!
```

### In the URC Mars Rover

**Real-World Challenge**: Navigate Mars terrain without real-time GPS, build a map of obstacles

**SLAM Solution**:

```python
# Pseudo-code for SLAM
class SLAMNode:
    def __init__(self):
        self.map = {}  # Features and their 3D positions
        self.pose = (0, 0, 0)  # (x, y, yaw)
        
    def process_frame(self, camera_image, depth_data, lidar_data):
        # 1. Extract features
        features = extract_features(camera_image)
        
        # 2. Track features between frames
        matched_features = match_features_with_previous_frame(features)
        
        # 3. Estimate motion
        estimated_motion = estimate_motion_from_features(matched_features)
        self.pose = update_pose(self.pose, estimated_motion)
        
        # 4. Update map
        for feature in features:
            feature_3d_position = convert_to_3d(feature, depth_data)
            self.map[feature.id] = feature_3d_position
            
        # 5. Detect loop closures
        if loop_closure_detected(features, self.map):
            correct_accumulated_error()
        
        return self.pose, self.map
```

### Sensor Types Used

| Sensor | Data Type | Pros | Cons |
|--------|-----------|------|------|
| **Camera (RGB)** | Image | Rich visual info | Affected by lighting |
| **Depth Camera** | 3D points | Precise distances | Limited range |
| **LiDAR** | 360° distance scan | Works day/night, wide range | Expensive, noisy |
| **IMU** | Acceleration, rotation | Fast, no moving parts | Drifts over time |
| **GPS** | Global position | No local error accumulation | Doesn't work indoors/underground |

### Sensor Fusion: Combining Data

SLAM doesn't just use one sensor - it **combines multiple sensors** to be more accurate:

```
         GPS           IMU           LIDAR         CAMERA
          ↓             ↓              ↓              ↓
          └─────────────┬──────────────┬──────────────┘
                        ↓
                 Sensor Fusion
                 (Kalman Filter)
                        ↓
            Best estimate of true position
```

**Kalman Filter**: Mathematical algorithm that combines uncertain measurements to get a better estimate.

---

## 3. Sensor Processing & Filtering

### The Problem: Noisy Data

Real sensors are imperfect:

```
IDEAL READING:     REAL-WORLD READING:
50mm distance      48mm, 51mm, 49mm, 52mm, 49mm, 50mm, ...
(constant)         (noisy, varies each time)
```

### Solution: Filtering

#### Simple Moving Average
```python
readings = [48, 51, 49, 52, 49]
average = sum(readings) / len(readings)  # 49.8mm
```

#### Kalman Filter (Advanced)
```python
# Estimates true value considering:
# - Measurement uncertainty
# - Expected motion model
# - Previous estimate
estimated_value = calculate_kalman(measurement, sensor_noise)
```

### In URC: Real-Time Processing

The rover constantly processes sensor data:

```
Sensor Data Stream:
IMU (100 Hz) → | \
GPS (10 Hz)  → |  SENSOR FUSION → STATE ESTIMATE
LiDAR (30 Hz)→ | /

Every 10 ms:
1. Collect new readings
2. Fuse data from multiple sources
3. Output best estimate of position, velocity, orientation
4. Use for navigation and collision avoidance
```

---

## 4. Perception in the URC 2026 Codebase

### File Structure

```
src/autonomy/autonomy_core/
├── perception/              # All perception code
│   ├── computer_vision_node.py      # Vision processing
│   ├── slam_node.py                 # SLAM
│   ├── sensor_fusion.py             # Sensor fusion
│   └── depth_processor.py           # Depth data filtering
```

### Configuration

```python
from src.infrastructure.config import get_urc_config

config = get_urc_config()

# Computer Vision Settings
config.perception.detection_model  # "yolov8" or "color_detection"
config.perception.confidence_threshold  # 0.7 for ML
config.perception.frame_rate_hz  # 30 Hz target

# SLAM Settings
config.perception.slam_enabled  # True/False
config.perception.loop_closure_threshold  # When to trigger

# Sensor Settings
config.perception.imu_update_rate  # 100 Hz
config.perception.gps_update_rate  # 10 Hz
```

### ROS2 Integration

```python
# Perception publishes these topics for other systems to use:

/vision/detections          # Found objects: (x, y, type, confidence)
/slam/pose                  # Estimated robot position
/slam/map                   # 3D map of environment
/sensor_data/processed      # Fused sensor readings
/perception/status          # Health and diagnostics
```

### Testing Perception

```bash
# Run all perception tests
python -m pytest tests/unit/test_perception.py -v

# Test vision pipeline
python -m pytest tests/unit/test_computer_vision.py -v

# Test SLAM
python -m pytest tests/unit/test_slam.py -v

# Performance tests
python -m pytest tests/performance/ -v --tb=short
```

---

## 5. How Perception Feeds Into Mission Success

### Example: Sample Collection Mission

```
GOAL: Find, approach, and collect a red sample

┌─────────────────────────────────────────────────────┐
│ 1. VISION LOOKS FOR RED                            │
│    "I see something red at (100px, 150px)"         │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ 2. SLAM DETERMINES DISTANCE TO OBJECT              │
│    "That object is 2 meters away, 30 degrees left" │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ 3. COGNITION PLANS A PATH                          │
│    "Navigate to (x, y) coordinates of the sample"  │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ 4. MOTION EXECUTES THE PATH                        │
│    "Drive left, move forward, stop at sample"      │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ 5. VISION CONFIRMS GRIP                            │
│    "Sample is in the collection arm's center"      │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ 6. PERCEPTION CONTINUOUSLY MONITORS                │
│    "Position updated, obstacle ahead, stop!"       │
└─────────────────────────────────────────────────────┘
```

---

## 6. Key Concepts for Beginners

### Accuracy vs. Speed Tradeoff

```
Speed               Accuracy
├─────────────────────────────┤
FAST ├─ Color detection (80% accurate, 30ms)
     ├─ LiDAR only (85% accurate, 50ms)
     ├─ Multi-sensor fusion (92% accurate, 100ms)
     └─ ML + all sensors (95% accurate, 200ms)      ← SLOW
```

### Real-Time Constraints

Mars rovers have **milliseconds to make decisions**:

- Frame capture: 33ms (at 30 Hz)
- Vision processing: <20ms
- SLAM update: <50ms
- Decision making: <100ms
- Total latency: <300ms

If perception is too slow, the rover crashes into obstacles!

### Why Filtering Matters

```
Raw sensor:    48mm  52mm  49mm  51mm  50mm  49mm
               Noisy! Can't trust individual readings

Filtered:      50mm  50mm  50mm  50mm  50mm  50mm
               Smooth! Can use for navigation
```

---

## 7. Common Challenges & Solutions

| Challenge | Why It's Hard | URC Solution |
|-----------|---------------|------------|
| **Lighting Changes** | Camera performance varies | Normalize images, use color spaces |
| **Terrain Variations** | Objects look different in sand vs. rocks | Train ML on diverse terrain |
| **Dust/Sand in Air** | Obscures vision and LiDAR | Use depth filtering, ignore distant points |
| **GPS Unreliability** | Underground caves don't have GPS | Use SLAM for local position |
| **Sensor Lag** | Network delays in communication | Fuse multiple sensors, predict motion |

---

## 8. Quick Reference: Perception APIs

### Get Current Robot Pose (Position)

```python
from src.autonomy.autonomy_core.perception import slam_node

slam = slam_node.SLAMNode()
pose = slam.get_current_pose()  # Returns (x, y, yaw)
print(f"Robot is at ({pose.x}, {pose.y})")
```

### Detect Objects in Image

```python
from src.autonomy.autonomy_core.perception import computer_vision_node

vision = computer_vision_node.VisionProcessor()
detections = vision.detect_objects(image)

for detection in detections:
    print(f"Found {detection.object_class} at position {detection.position}")
```

### Get Sensor Status

```python
from src.infrastructure.monitoring import HealthMonitor

monitor = HealthMonitor()
status = monitor.get_sensor_status()
print(f"GPS: {status.gps}")      # "HEALTHY", "DEGRADED", "FAILED"
print(f"IMU: {status.imu}")      # Accelerometer and gyro status
print(f"Camera: {status.camera}")  # Vision system status
```

---

## 9. Knowledge Check: Test Your Understanding

1. **What's the difference between Computer Vision and SLAM?**
   - Vision: Detects objects in images
   - SLAM: Determines robot position and maps the environment

2. **Why does a robot need multiple sensors?**
   - Each sensor has different strengths/weaknesses
   - Combined data is more accurate and reliable

3. **What happens if the vision system fails?**
   - Robot can still navigate using SLAM and sensor fusion
   - Mission might continue at reduced capability

4. **How fast must perception process data?**
   - <300ms total latency to avoid collisions
   - Depends on rover speed and terrain

---

## 10. Next Steps

- **Read the code**: Check `src/autonomy/autonomy_core/perception/`
- **Run tests**: `python -m pytest tests/unit/test_perception.py -v`
- **Understand SLAM**: Read about Kalman filters and loop closure
- **Experiment**: Try modifying detection parameters in config
- **Contribute**: Improve perception accuracy!

---

## Key Takeaways

1. **Perception is the rover's senses** - Without it, the rover is blind
2. **Multiple sensors are better** - Redundancy and accuracy through fusion
3. **Real-time is critical** - Must process data in milliseconds
4. **Filtering improves reliability** - Remove noise from sensor data
5. **Perception drives decisions** - All downstream decisions depend on it

---

*Next Pillar: [Cognition & Decision Making](PILLAR_2_COGNITION.md)*
