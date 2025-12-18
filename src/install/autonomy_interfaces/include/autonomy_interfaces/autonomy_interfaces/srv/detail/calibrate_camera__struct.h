// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image_directory'
// Member 'board_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CalibrateCamera in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__CalibrateCamera_Request
{
  rosidl_runtime_c__String image_directory;
  /// "charuco" or "chessboard"
  rosidl_runtime_c__String board_type;
  int32_t squares_x;
  int32_t squares_y;
  float square_size;
  float marker_size;
} autonomy_interfaces__srv__CalibrateCamera_Request;

// Struct for a sequence of autonomy_interfaces__srv__CalibrateCamera_Request.
typedef struct autonomy_interfaces__srv__CalibrateCamera_Request__Sequence
{
  autonomy_interfaces__srv__CalibrateCamera_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__CalibrateCamera_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result_file'
// Member 'calibration_summary'
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CalibrateCamera in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__CalibrateCamera_Response
{
  bool success;
  rosidl_runtime_c__String result_file;
  rosidl_runtime_c__String calibration_summary;
  rosidl_runtime_c__String error_message;
} autonomy_interfaces__srv__CalibrateCamera_Response;

// Struct for a sequence of autonomy_interfaces__srv__CalibrateCamera_Response.
typedef struct autonomy_interfaces__srv__CalibrateCamera_Response__Sequence
{
  autonomy_interfaces__srv__CalibrateCamera_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__CalibrateCamera_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_
