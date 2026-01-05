// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_alert.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'property'
// Member 'severity'
// Member 'details'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SafetyAlert in the package autonomy_interfaces.
typedef struct autonomy_interfaces__msg__SafetyAlert
{
  rosidl_runtime_c__String property;
  rosidl_runtime_c__String severity;
  rosidl_runtime_c__String details;
  double timestamp;
  bool acknowledged;
} autonomy_interfaces__msg__SafetyAlert;

// Struct for a sequence of autonomy_interfaces__msg__SafetyAlert.
typedef struct autonomy_interfaces__msg__SafetyAlert__Sequence
{
  autonomy_interfaces__msg__SafetyAlert * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__SafetyAlert__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_H_
