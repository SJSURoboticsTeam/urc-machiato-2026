// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/camera_command.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_H_

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
// Member 'scan_pattern'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CameraCommand in the package autonomy_interfaces.
/**
  * Camera control commands for mast-mounted camera
  * Supports pan-tilt-zoom operations for computer vision tasks
 */
typedef struct autonomy_interfaces__msg__CameraCommand
{
  std_msgs__msg__Header header;
  /// Command type
  /// 0=absolute, 1=relative, 2=track_target, 3=scan_pattern
  uint8_t command_type;
  /// Pan-tilt control (radians)
  /// Desired pan angle (-pi to pi)
  float pan_angle;
  /// Desired tilt angle (-pi/2 to pi/2)
  float tilt_angle;
  /// Pan angular velocity (rad/s)
  float pan_speed;
  /// Tilt angular velocity (rad/s)
  float tilt_speed;
  /// Zoom control
  /// 1.0 = normal, higher values = zoomed in
  float zoom_level;
  /// Enable/disable autofocus
  bool autofocus;
  /// Tracking control (for command_type=2)
  /// Target position in camera frame
  geometry_msgs__msg__Point target_position;
  /// Seconds to track before giving up
  float tracking_timeout;
  /// Scan pattern control (for command_type=3)
  /// "horizontal", "vertical", "spiral", "raster"
  rosidl_runtime_c__String scan_pattern;
  /// Scan angular velocity (rad/s)
  float scan_speed;
  /// Total scan angle (radians)
  float scan_range;
  /// Safety limits
  /// Maximum allowed pan speed (rad/s)
  float max_pan_speed;
  /// Maximum allowed tilt speed (rad/s)
  float max_tilt_speed;
  /// Command priority and timing
  /// 0=normal, 1=high, 2=critical
  uint8_t priority;
  /// Command timeout in seconds
  float timeout;
} autonomy_interfaces__msg__CameraCommand;

// Struct for a sequence of autonomy_interfaces__msg__CameraCommand.
typedef struct autonomy_interfaces__msg__CameraCommand__Sequence
{
  autonomy_interfaces__msg__CameraCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__CameraCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_H_
