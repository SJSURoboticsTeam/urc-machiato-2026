// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/detect_mission_aruco.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mission_type'
#include "rosidl_runtime_c/string.h"
// Member 'required_tag_ids'
// Member 'optional_tag_ids'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/DetectMissionAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Request
{
  /// Request
  /// Mission type: "AUTONOMOUS_TYPING", "USB_CONNECTION", "PANEL_OPERATIONS"
  rosidl_runtime_c__String mission_type;
  /// Required ArUco tag IDs for this mission
  rosidl_runtime_c__int32__Sequence required_tag_ids;
  /// Optional ArUco tag IDs (improves alignment quality)
  rosidl_runtime_c__int32__Sequence optional_tag_ids;
  /// Timeout in seconds (0.0 = no timeout)
  float detection_timeout;
  /// Target depth for arm positioning (meters from tag plane)
  float target_depth;
  /// Maximum detection distance in meters
  float max_detection_distance;
  /// Whether all required tags must be detected
  bool require_all_tags;
  /// Minimum alignment quality threshold (0.0-1.0)
  float min_alignment_quality;
} autonomy_interfaces__srv__DetectMissionAruco_Request;

// Struct for a sequence of autonomy_interfaces__srv__DetectMissionAruco_Request.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence
{
  autonomy_interfaces__srv__DetectMissionAruco_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'mission_type'
// Member 'missing_required_tags'
// Member 'detected_optional_tags'
// Member 'alignment_warnings'
// Member 'mission_recommendations'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'detected_tag_ids'
// Member 'tag_distances'
// Member 'tag_angles'
// Member 'alignment_errors'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"
// Member 'tag_positions'
// Member 'alignment_center'
// Member 'arm_target_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'detection_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'alignment_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in srv/DetectMissionAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Response
{
  /// Whether detection and alignment calculation was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Mission type that was processed
  rosidl_runtime_c__String mission_type;
  /// Detection results
  /// IDs of successfully detected tags
  rosidl_runtime_c__int32__Sequence detected_tag_ids;
  /// 3D positions of detected tags
  geometry_msgs__msg__Point__Sequence tag_positions;
  /// Estimated distances to tags
  rosidl_runtime_c__float__Sequence tag_distances;
  /// Angles to tags (radians)
  rosidl_runtime_c__float__Sequence tag_angles;
  /// When detection occurred
  builtin_interfaces__msg__Time detection_time;
  /// Alignment calculation results
  /// Whether alignment calculation is available
  bool alignment_available;
  /// Calculated center point of detected tags
  geometry_msgs__msg__Point alignment_center;
  /// Calculated orientation for coplanar alignment
  geometry_msgs__msg__Quaternion alignment_orientation;
  /// Recommended arm position for alignment
  geometry_msgs__msg__Point arm_target_position;
  /// Quality score of alignment (0.0-1.0)
  float alignment_quality;
  /// Individual alignment errors for each tag
  rosidl_runtime_c__float__Sequence alignment_errors;
  /// Mission-specific information
  /// Whether mission can proceed based on detection results
  bool mission_ready;
  /// Required tags that were not detected
  rosidl_runtime_c__String__Sequence missing_required_tags;
  /// Optional tags that were detected
  rosidl_runtime_c__String__Sequence detected_optional_tags;
  /// Warnings about alignment quality or missing tags
  rosidl_runtime_c__String__Sequence alignment_warnings;
  /// Recommendations for improving mission success
  rosidl_runtime_c__String__Sequence mission_recommendations;
} autonomy_interfaces__srv__DetectMissionAruco_Response;

// Struct for a sequence of autonomy_interfaces__srv__DetectMissionAruco_Response.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence
{
  autonomy_interfaces__srv__DetectMissionAruco_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__DetectMissionAruco_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__DetectMissionAruco_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/DetectMissionAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence request;
  autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence response;
} autonomy_interfaces__srv__DetectMissionAruco_Event;

// Struct for a sequence of autonomy_interfaces__srv__DetectMissionAruco_Event.
typedef struct autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence
{
  autonomy_interfaces__srv__DetectMissionAruco_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_H_
