// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/NavigationStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__STRUCT_H_

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
// Member 'state'
// Member 'status_message'
#include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/NavigationStatus in the package autonomy_interfaces.
/**
  * Navigation subsystem status
  * Provides current navigation state and progress
 */
typedef struct autonomy_interfaces__msg__NavigationStatus
{
  std_msgs__msg__Header header;
  /// Navigation state machine
  /// "idle", "planning", "navigating", "avoiding_obstacle", "precision_approach", "arrived", "error"
  rosidl_runtime_c__String state;
  /// Progress indicators
  /// 0.0 to 1.0, overall mission completion
  float mission_progress;
  /// Current waypoint index (0-based)
  int32_t current_waypoint;
  /// Total waypoints in mission
  int32_t total_waypoints;
  /// Current position and goal
  geometry_msgs__msg__PoseStamped current_pose;
  geometry_msgs__msg__PoseStamped goal_pose;
  /// Performance metrics
  /// meters, negative if no active goal
  float distance_to_goal;
  /// m/s, current linear speed
  float speed;
  /// radians, difference between desired and actual heading
  float heading_error;
  /// Status message for operators
  rosidl_runtime_c__String status_message;
} autonomy_interfaces__msg__NavigationStatus;

// Struct for a sequence of autonomy_interfaces__msg__NavigationStatus.
typedef struct autonomy_interfaces__msg__NavigationStatus__Sequence
{
  autonomy_interfaces__msg__NavigationStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__NavigationStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__STRUCT_H_
