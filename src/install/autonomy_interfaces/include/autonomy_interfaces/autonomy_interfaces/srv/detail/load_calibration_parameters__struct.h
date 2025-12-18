// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/LoadCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'calibration_file'
// Member 'parameter_namespace'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadCalibrationParameters in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__LoadCalibrationParameters_Request
{
  rosidl_runtime_c__String calibration_file;
  rosidl_runtime_c__String parameter_namespace;
} autonomy_interfaces__srv__LoadCalibrationParameters_Request;

// Struct for a sequence of autonomy_interfaces__srv__LoadCalibrationParameters_Request.
typedef struct autonomy_interfaces__srv__LoadCalibrationParameters_Request__Sequence
{
  autonomy_interfaces__srv__LoadCalibrationParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__LoadCalibrationParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadCalibrationParameters in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__LoadCalibrationParameters_Response
{
  bool success;
  rosidl_runtime_c__String error_message;
} autonomy_interfaces__srv__LoadCalibrationParameters_Response;

// Struct for a sequence of autonomy_interfaces__srv__LoadCalibrationParameters_Response.
typedef struct autonomy_interfaces__srv__LoadCalibrationParameters_Response__Sequence
{
  autonomy_interfaces__srv__LoadCalibrationParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__LoadCalibrationParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__STRUCT_H_
