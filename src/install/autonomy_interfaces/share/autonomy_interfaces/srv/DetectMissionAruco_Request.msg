# DetectMissionAruco.srv
# Service for mission-specific ArUco tag detection with auto-alignment

# Request
string mission_type  # Mission type: "AUTONOMOUS_TYPING", "USB_CONNECTION", "PANEL_OPERATIONS"
int32[] required_tag_ids  # Required ArUco tag IDs for this mission
int32[] optional_tag_ids  # Optional ArUco tag IDs (improves alignment quality)
float32 detection_timeout  # Timeout in seconds (0.0 = no timeout)
float32 target_depth  # Target depth for arm positioning (meters from tag plane)
float32 max_detection_distance  # Maximum detection distance in meters
bool require_all_tags  # Whether all required tags must be detected
float32 min_alignment_quality  # Minimum alignment quality threshold (0.0-1.0)

