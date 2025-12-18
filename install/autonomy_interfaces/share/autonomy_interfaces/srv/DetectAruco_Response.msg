
# Response
bool success  # Whether detection was successful
string message  # Status message
int32[] detected_tag_ids  # IDs of detected tags
geometry_msgs/Point[] tag_positions  # 3D positions of detected tags
float32[] tag_distances  # Estimated distances to tags
float32[] tag_angles  # Angles to tags (radians)
builtin_interfaces/Time detection_time  # When detection occurred

# Alignment information (only if calculate_alignment=true)
bool alignment_available  # Whether alignment calculation is available
geometry_msgs/Point alignment_center  # Calculated center point of detected tags
geometry_msgs/Quaternion alignment_orientation  # Calculated orientation for coplanar alignment
geometry_msgs/Point arm_target_position  # Recommended arm position for alignment
float32 alignment_quality  # Quality score of alignment (0.0-1.0)
string[] alignment_warnings  # Warnings about alignment quality or missing tags
