// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/arm_alignment_command.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_H_

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
// Member 'mission_type'
// Member 'alignment_id'
// Member 'safety_zones'
// Member 'required_aruco_tags'
#include "rosidl_runtime_c/string.h"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'target_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/ArmAlignmentCommand in the package autonomy_interfaces.
/**
  * ArmAlignmentCommand.msg
  * Command for arm alignment based on ArUco tag detection
 */
typedef struct autonomy_interfaces__msg__ArmAlignmentCommand
{
  std_msgs__msg__Header header;
  /// Mission information
  /// Mission type: "AUTONOMOUS_TYPING", "USB_CONNECTION", "PANEL_OPERATIONS"
  rosidl_runtime_c__String mission_type;
  /// Unique identifier for this alignment operation
  rosidl_runtime_c__String alignment_id;
  /// Target information
  /// Target position for arm end-effector
  geometry_msgs__msg__Point target_position;
  /// Target orientation for arm end-effector
  geometry_msgs__msg__Quaternion target_orientation;
  /// Distance to maintain from target during approach
  float approach_distance;
  /// Final distance from target for operation
  float final_distance;
  /// Alignment parameters
  /// Quality score of alignment (0.0-1.0)
  float alignment_quality;
  /// Maximum allowed position error (meters)
  float max_position_error;
  /// Maximum allowed orientation error (radians)
  float max_orientation_error;
  /// Timeout for alignment operation (seconds)
  float alignment_timeout;
  /// Safety parameters
  /// Maximum approach speed (m/s)
  float max_approach_speed;
  /// Maximum rotation speed (rad/s)
  float max_rotation_speed;
  /// Whether to enable collision avoidance
  bool enable_collision_avoidance;
  /// Safety zones to avoid during movement
  rosidl_runtime_c__String__Sequence safety_zones;
  /// Feedback requirements
  /// Whether to require position feedback during alignment
  bool require_position_feedback;
  /// Whether to require force feedback during alignment
  bool require_force_feedback;
  /// Rate for feedback updates (Hz)
  float feedback_rate;
  /// Mission-specific parameters
  /// ArUco tags that must remain visible during operation
  rosidl_runtime_c__String__Sequence required_aruco_tags;
  /// Timeout for tag visibility loss (seconds)
  float tag_visibility_timeout;
  /// Whether to allow realignment if tags are lost
  bool allow_realignment;
} autonomy_interfaces__msg__ArmAlignmentCommand;

// Struct for a sequence of autonomy_interfaces__msg__ArmAlignmentCommand.
typedef struct autonomy_interfaces__msg__ArmAlignmentCommand__Sequence
{
  autonomy_interfaces__msg__ArmAlignmentCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__ArmAlignmentCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_H_
