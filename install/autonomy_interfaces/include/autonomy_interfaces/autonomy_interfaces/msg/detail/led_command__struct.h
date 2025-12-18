// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_H_

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
// Member 'pattern'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LedCommand in the package autonomy_interfaces.
/**
  * LED status signaling command
  * Based on URC 2026 competition requirements
 */
typedef struct autonomy_interfaces__msg__LedCommand
{
  std_msgs__msg__Header header;
  /// Status codes (judge-visible signals)
  /// 0=off, 1=ready, 2=running, 3=error, 4=emergency, 5=success
  int32_t status_code;
  /// Color specification
  /// 0.0 to 1.0
  float red;
  /// 0.0 to 1.0
  float green;
  /// 0.0 to 1.0
  float blue;
  /// Pattern specification
  /// "solid", "blinking", "pulsing", "chasing"
  rosidl_runtime_c__String pattern;
  /// Hz, for blinking/pulsing patterns
  float frequency;
  /// 0=normal, 1=warning, 2=critical (affects brightness)
  int32_t priority;
  /// Timing
  /// seconds, 0.0 = indefinite
  float duration;
  /// true to override lower priority commands
  bool override;
} autonomy_interfaces__msg__LedCommand;

// Struct for a sequence of autonomy_interfaces__msg__LedCommand.
typedef struct autonomy_interfaces__msg__LedCommand__Sequence
{
  autonomy_interfaces__msg__LedCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__LedCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_H_
