// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/typing_goal.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_H_

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
// Member 'target_text'
// Member 'keyboard_layout'
#include "rosidl_runtime_c/string.h"
// Member 'keyboard_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'key_dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TypingGoal in the package autonomy_interfaces.
/**
  * Goal specification for autonomous typing task
 */
typedef struct autonomy_interfaces__msg__TypingGoal
{
  std_msgs__msg__Header header;
  /// Target specification
  /// Text to type on keyboard
  rosidl_runtime_c__String target_text;
  /// Location of keyboard in world frame
  geometry_msgs__msg__PoseStamped keyboard_pose;
  /// Performance requirements
  /// Minimum accuracy (0.0 to 1.0)
  float accuracy_requirement;
  /// Minimum typing speed (chars/min)
  float speed_requirement;
  /// Maximum time allowed (seconds)
  float timeout;
  /// Keyboard layout information
  /// "qwerty", "azerty", etc.
  rosidl_runtime_c__String keyboard_layout;
  /// Distance between key centers (meters)
  float key_spacing_m;
  /// Width, height, depth of keys
  geometry_msgs__msg__Vector3 key_dimensions;
  /// Approach parameters
  /// Distance to approach keyboard (meters)
  float standoff_distance;
  /// Desired contact force (Newtons)
  float contact_force;
} autonomy_interfaces__msg__TypingGoal;

// Struct for a sequence of autonomy_interfaces__msg__TypingGoal.
typedef struct autonomy_interfaces__msg__TypingGoal__Sequence
{
  autonomy_interfaces__msg__TypingGoal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__TypingGoal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_H_
