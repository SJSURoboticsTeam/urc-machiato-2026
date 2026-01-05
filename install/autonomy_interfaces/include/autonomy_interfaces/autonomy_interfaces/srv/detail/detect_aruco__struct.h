// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/detect_aruco.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_tag_ids'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'mission_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DetectAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectAruco_Request
{
  /// Request
  /// List of ArUco tag IDs to detect (empty = detect any)
  rosidl_runtime_c__int32__Sequence target_tag_ids;
  /// Timeout in seconds (0.0 = no timeout)
  float detection_timeout;
  /// Whether to require distance estimation
  bool require_distance_estimate;
  /// Maximum detection distance in meters (0.0 = no limit)
  float max_detection_distance;
  /// Whether to calculate alignment for multi-tag scenarios
  bool calculate_alignment;
  /// Target depth for alignment (meters from tag plane)
  float target_depth;
  /// Mission type: "TYPING", "USB", "FOLLOW_ME", "GENERAL"
  rosidl_runtime_c__String mission_type;
} autonomy_interfaces__srv__DetectAruco_Request;

// Struct for a sequence of autonomy_interfaces__srv__DetectAruco_Request.
typedef struct autonomy_interfaces__srv__DetectAruco_Request__Sequence
{
  autonomy_interfaces__srv__DetectAruco_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectAruco_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'alignment_warnings'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'detected_tag_ids'
// Member 'tag_distances'
// Member 'tag_angles'
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

/// Struct defined in srv/DetectAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectAruco_Response
{
  /// Whether detection was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// IDs of detected tags
  rosidl_runtime_c__int32__Sequence detected_tag_ids;
  /// 3D positions of detected tags
  geometry_msgs__msg__Point__Sequence tag_positions;
  /// Estimated distances to tags
  rosidl_runtime_c__float__Sequence tag_distances;
  /// Angles to tags (radians)
  rosidl_runtime_c__float__Sequence tag_angles;
  /// When detection occurred
  builtin_interfaces__msg__Time detection_time;
  /// Alignment information (only if calculate_alignment=true)
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
  /// Warnings about alignment quality or missing tags
  rosidl_runtime_c__String__Sequence alignment_warnings;
} autonomy_interfaces__srv__DetectAruco_Response;

// Struct for a sequence of autonomy_interfaces__srv__DetectAruco_Response.
typedef struct autonomy_interfaces__srv__DetectAruco_Response__Sequence
{
  autonomy_interfaces__srv__DetectAruco_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectAruco_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__DetectAruco_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__DetectAruco_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/DetectAruco in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__DetectAruco_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__DetectAruco_Request__Sequence request;
  autonomy_interfaces__srv__DetectAruco_Response__Sequence response;
} autonomy_interfaces__srv__DetectAruco_Event;

// Struct for a sequence of autonomy_interfaces__srv__DetectAruco_Event.
typedef struct autonomy_interfaces__srv__DetectAruco_Event__Sequence
{
  autonomy_interfaces__srv__DetectAruco_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__DetectAruco_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_H_
