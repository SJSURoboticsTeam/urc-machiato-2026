// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'operator_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FollowMeControl in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__FollowMeControl_Request
{
  /// Request
  /// ArUco tag ID to follow
  int32_t target_tag_id;
  /// Safe following distance in meters
  float safety_distance;
  /// Maximum following speed in m/s
  float max_speed;
  /// Start/stop following
  bool enable_following;
  /// ID of operator requesting control
  rosidl_runtime_c__String operator_id;
} autonomy_interfaces__srv__FollowMeControl_Request;

// Struct for a sequence of autonomy_interfaces__srv__FollowMeControl_Request.
typedef struct autonomy_interfaces__srv__FollowMeControl_Request__Sequence
{
  autonomy_interfaces__srv__FollowMeControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__FollowMeControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/FollowMeControl in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__FollowMeControl_Response
{
  /// Whether command was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Current following status
  bool is_following;
  /// Currently tracked tag ID
  int32_t current_target_tag;
  /// Current distance to target
  float current_distance;
  /// Current target position
  geometry_msgs__msg__Point target_position;
} autonomy_interfaces__srv__FollowMeControl_Response;

// Struct for a sequence of autonomy_interfaces__srv__FollowMeControl_Response.
typedef struct autonomy_interfaces__srv__FollowMeControl_Response__Sequence
{
  autonomy_interfaces__srv__FollowMeControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__FollowMeControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_H_
