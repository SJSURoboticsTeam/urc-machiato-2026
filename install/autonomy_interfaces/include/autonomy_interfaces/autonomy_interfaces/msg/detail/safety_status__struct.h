// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_

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
// Member 'safety_level'
// Member 'active_triggers'
// Member 'trigger_type'
// Member 'trigger_source'
// Member 'trigger_description'
// Member 'recovery_steps'
// Member 'context_state'
// Member 'mission_phase'
// Member 'degraded_capabilities'
#include "rosidl_runtime_c/string.h"
// Member 'trigger_time'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SafetyStatus in the package autonomy_interfaces.
/**
  * Safety Status Message
  * Comprehensive safety system status with trigger types and recovery requirements
 */
typedef struct autonomy_interfaces__msg__SafetyStatus
{
  std_msgs__msg__Header header;
  /// Safety state
  /// Overall safety status
  bool is_safe;
  /// "NORMAL", "WARNING", "CRITICAL", "EMERGENCY"
  rosidl_runtime_c__String safety_level;
  /// List of currently active safety triggers
  rosidl_runtime_c__String__Sequence active_triggers;
  /// Trigger information
  /// Type of safety trigger
  rosidl_runtime_c__String trigger_type;
  /// Subsystem that triggered safety response
  rosidl_runtime_c__String trigger_source;
  /// When safety was triggered
  builtin_interfaces__msg__Time trigger_time;
  /// Human-readable description
  rosidl_runtime_c__String trigger_description;
  /// Recovery information
  /// True if manual recovery required
  bool requires_manual_intervention;
  /// True if automatic recovery possible
  bool can_auto_recover;
  /// List of steps needed for recovery
  rosidl_runtime_c__String__Sequence recovery_steps;
  /// Estimated time to recover (seconds)
  double estimated_recovery_time;
  /// Context-specific information
  /// State when safety was triggered
  rosidl_runtime_c__String context_state;
  /// Mission context when triggered
  rosidl_runtime_c__String mission_phase;
  /// True if safe to retry after recovery
  bool safe_to_retry;
  /// System health
  /// Current battery percentage
  double battery_level;
  /// System temperature (Celsius)
  double temperature;
  /// Communication link status
  bool communication_ok;
  /// Sensor system status
  bool sensors_ok;
  /// List of capabilities that are degraded
  rosidl_runtime_c__String__Sequence degraded_capabilities;
} autonomy_interfaces__msg__SafetyStatus;

// Struct for a sequence of autonomy_interfaces__msg__SafetyStatus.
typedef struct autonomy_interfaces__msg__SafetyStatus__Sequence
{
  autonomy_interfaces__msg__SafetyStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__SafetyStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_
