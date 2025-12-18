# DetectAruco.srv
# Service for ArUco tag detection requests

# Request
int32[] target_tag_ids  # List of ArUco tag IDs to detect (empty = detect any)
float32 detection_timeout  # Timeout in seconds (0.0 = no timeout)
bool require_distance_estimate  # Whether to require distance estimation
float32 max_detection_distance  # Maximum detection distance in meters (0.0 = no limit)
bool calculate_alignment  # Whether to calculate alignment for multi-tag scenarios
float32 target_depth  # Target depth for alignment (meters from tag plane)
string mission_type  # Mission type: "TYPING", "USB", "FOLLOW_ME", "GENERAL"
