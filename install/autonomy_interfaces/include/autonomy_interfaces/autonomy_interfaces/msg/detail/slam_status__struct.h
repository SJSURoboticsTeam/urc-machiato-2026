// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/SlamStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/slam_status.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.h"
// Member 'local_map'
#include "nav_msgs/msg/detail/occupancy_grid__struct.h"

/// Struct defined in msg/SlamStatus in the package autonomy_interfaces.
/**
  * SLAM subsystem status and pose estimate
 */
typedef struct autonomy_interfaces__msg__SlamStatus
{
  std_msgs__msg__Header header;
  /// SLAM state
  /// "initializing", "tracking", "lost", "relocalizing", "mapping"
  rosidl_runtime_c__String state;
  /// Pose estimate with covariance
  geometry_msgs__msg__PoseWithCovarianceStamped pose;
  /// Map information
  nav_msgs__msg__OccupancyGrid local_map;
  /// Map dimensions
  int32_t map_width;
  int32_t map_height;
  /// meters per pixel
  float map_resolution;
  /// Performance metrics
  /// 0.0 to 1.0
  float loop_closure_confidence;
  /// Number of keyframes in map
  int32_t keyframes_tracked;
  /// Number of landmarks/features tracked
  int32_t landmarks_tracked;
  /// Quality indicators
  /// 0.0 (lost) to 1.0 (excellent tracking)
  float tracking_quality;
  /// True if recent loop closure
  bool loop_closure_detected;
  /// Estimated position drift in meters
  float drift_estimate;
} autonomy_interfaces__msg__SlamStatus;

// Struct for a sequence of autonomy_interfaces__msg__SlamStatus.
typedef struct autonomy_interfaces__msg__SlamStatus__Sequence
{
  autonomy_interfaces__msg__SlamStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__SlamStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_H_
