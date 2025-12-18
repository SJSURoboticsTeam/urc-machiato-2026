# FollowMeControl.srv
# Service for controlling follow me behavior

# Request
int32 target_tag_id  # ArUco tag ID to follow
float32 safety_distance  # Safe following distance in meters
float32 max_speed  # Maximum following speed in m/s
bool enable_following  # Start/stop following
string operator_id  # ID of operator requesting control
