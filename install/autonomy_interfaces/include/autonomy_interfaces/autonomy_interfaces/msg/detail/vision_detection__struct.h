// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/VisionDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/vision_detection.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__STRUCT_H_

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
// Member 'class_name'
// Member 'detector_type'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'size'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'keypoints'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/VisionDetection in the package autonomy_interfaces.
/**
  * Single object detection result
  * Used in DetectionArray for multiple detections
 */
typedef struct autonomy_interfaces__msg__VisionDetection
{
  std_msgs__msg__Header header;
  /// Detection metadata
  /// Object class ("aruco_marker", "keyboard", "target", etc.)
  rosidl_runtime_c__String class_name;
  /// Class identifier
  int32_t class_id;
  /// Detection confidence (0.0 to 1.0)
  float confidence;
  /// Object pose and size
  /// Object position and orientation
  geometry_msgs__msg__PoseStamped pose;
  /// Object dimensions (width, height, depth)
  geometry_msgs__msg__Vector3 size;
  /// Vision-specific data
  /// Feature points (x,y coordinates)
  rosidl_runtime_c__float__Sequence keypoints;
  /// "aruco", "yolo", "template_matching", etc.
  rosidl_runtime_c__String detector_type;
  /// Tracking information
  /// Object track ID for multi-frame tracking
  int32_t track_id;
  /// Seconds since first detection
  float age;
} autonomy_interfaces__msg__VisionDetection;

// Struct for a sequence of autonomy_interfaces__msg__VisionDetection.
typedef struct autonomy_interfaces__msg__VisionDetection__Sequence
{
  autonomy_interfaces__msg__VisionDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__VisionDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__STRUCT_H_
