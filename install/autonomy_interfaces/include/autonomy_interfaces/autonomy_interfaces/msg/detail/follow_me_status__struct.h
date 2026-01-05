// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/FollowMeStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/follow_me_status.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'operator_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FollowMeStatus in the package autonomy_interfaces.
/**
  * FollowMeStatus.msg
  * Status information for follow me mode
 */
typedef struct autonomy_interfaces__msg__FollowMeStatus
{
  std_msgs__msg__Header header;
  /// Follow me state
  /// Whether currently following
  bool is_following;
  /// ID of tag being followed
  int32_t target_tag_id;
  /// Current distance to target
  float target_distance;
  /// Current angle to target (radians)
  float target_angle;
  /// Safety information
  /// Configured safety distance
  float safety_distance;
  /// Whether safety distance is violated
  bool safety_violation;
  /// Current following speed
  float current_speed;
  /// Target tracking
  /// 3D position of target
  geometry_msgs__msg__Point target_position;
  /// Whether target is currently visible
  bool target_visible;
  /// Time since last detection (seconds)
  float last_detection_time;
  /// Control parameters
  /// Maximum allowed speed
  float max_speed;
  /// ID of controlling operator
  rosidl_runtime_c__String operator_id;
} autonomy_interfaces__msg__FollowMeStatus;

// Struct for a sequence of autonomy_interfaces__msg__FollowMeStatus.
typedef struct autonomy_interfaces__msg__FollowMeStatus__Sequence
{
  autonomy_interfaces__msg__FollowMeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__FollowMeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_H_
