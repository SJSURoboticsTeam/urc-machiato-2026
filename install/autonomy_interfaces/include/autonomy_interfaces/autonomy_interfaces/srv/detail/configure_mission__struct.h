// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/configure_mission.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mission_name'
// Member 'objectives'
// Member 'waypoint_names'
// Member 'typing_text'
// Member 'terrain_type'
#include "rosidl_runtime_c/string.h"
// Member 'waypoints'
// Member 'typing_location'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'precision_required'
// Member 'waypoint_tolerances'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ConfigureMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ConfigureMission_Request
{
  /// Request: Mission specification
  rosidl_runtime_c__String mission_name;
  /// ["navigate", "typing", "sample_collection", etc.]
  rosidl_runtime_c__String__Sequence objectives;
  /// Waypoint configuration
  geometry_msgs__msg__PoseStamped__Sequence waypoints;
  rosidl_runtime_c__String__Sequence waypoint_names;
  /// Whether precision approach needed
  rosidl_runtime_c__boolean__Sequence precision_required;
  /// Acceptable position error (meters)
  rosidl_runtime_c__float__Sequence waypoint_tolerances;
  /// Timing constraints
  /// Maximum mission duration (seconds)
  float time_limit;
  /// Timeout per waypoint (seconds)
  float waypoint_timeout;
  /// Navigation parameters
  /// m/s
  float max_linear_velocity;
  /// rad/s
  float max_angular_velocity;
  /// meters
  float waypoint_approach_tolerance;
  /// Mission-specific parameters
  /// Text to type (if typing objective)
  rosidl_runtime_c__String typing_text;
  /// Where to perform typing
  geometry_msgs__msg__PoseStamped typing_location;
  /// Environmental parameters
  /// "rough", "smooth", "mixed"
  rosidl_runtime_c__String terrain_type;
  /// Maximum allowed terrain incline (degrees)
  float max_incline;
} autonomy_interfaces__srv__ConfigureMission_Request;

// Struct for a sequence of autonomy_interfaces__srv__ConfigureMission_Request.
typedef struct autonomy_interfaces__srv__ConfigureMission_Request__Sequence
{
  autonomy_interfaces__srv__ConfigureMission_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ConfigureMission_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'mission_id'
// Member 'configured_objectives'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConfigureMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ConfigureMission_Response
{
  bool success;
  rosidl_runtime_c__String message;
  /// Unique identifier for configured mission
  rosidl_runtime_c__String mission_id;
  /// Expected mission time (seconds)
  float estimated_duration;
  /// Number of waypoints configured
  int32_t total_waypoints;
  /// Objectives that were successfully configured
  rosidl_runtime_c__String__Sequence configured_objectives;
} autonomy_interfaces__srv__ConfigureMission_Response;

// Struct for a sequence of autonomy_interfaces__srv__ConfigureMission_Response.
typedef struct autonomy_interfaces__srv__ConfigureMission_Response__Sequence
{
  autonomy_interfaces__srv__ConfigureMission_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ConfigureMission_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__ConfigureMission_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__ConfigureMission_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ConfigureMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ConfigureMission_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence request;
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence response;
} autonomy_interfaces__srv__ConfigureMission_Event;

// Struct for a sequence of autonomy_interfaces__srv__ConfigureMission_Event.
typedef struct autonomy_interfaces__srv__ConfigureMission_Event__Sequence
{
  autonomy_interfaces__srv__ConfigureMission_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ConfigureMission_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_H_
