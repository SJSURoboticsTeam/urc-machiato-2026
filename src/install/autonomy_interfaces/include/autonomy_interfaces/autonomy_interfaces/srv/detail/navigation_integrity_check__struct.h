// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'check_components'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NavigationIntegrityCheck in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__NavigationIntegrityCheck_Request
{
  /// Request
  /// Request detailed integrity analysis
  bool detailed_check;
  /// Specific components to check (empty = all)
  rosidl_runtime_c__String__Sequence check_components;
} autonomy_interfaces__srv__NavigationIntegrityCheck_Request;

// Struct for a sequence of autonomy_interfaces__srv__NavigationIntegrityCheck_Request.
typedef struct autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'integrity_level'
// Member 'checked_components'
// Member 'component_details'
// Member 'recommendations'
// Member 'timestamp'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'component_status'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/NavigationIntegrityCheck in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__NavigationIntegrityCheck_Response
{
  /// Overall navigation integrity status
  bool integrity_ok;
  /// Integrity score (0.0-1.0)
  double integrity_score;
  /// "NOMINAL", "DEGRADED", "CRITICAL", "FAILED"
  rosidl_runtime_c__String integrity_level;
  /// Component status
  /// Components that were checked
  rosidl_runtime_c__String__Sequence checked_components;
  /// Status of each component
  rosidl_runtime_c__boolean__Sequence component_status;
  /// Detailed status for each component
  rosidl_runtime_c__String__Sequence component_details;
  /// Metrics
  /// Current position accuracy (meters)
  double position_accuracy;
  /// Current heading accuracy (degrees)
  double heading_accuracy;
  /// Velocity consistency score (0.0-1.0)
  double velocity_consistency;
  /// Number of satellites used
  int32_t satellite_count;
  /// Horizontal dilution of precision
  double hdop;
  /// Recommendations
  /// Recommended actions if integrity is compromised
  rosidl_runtime_c__String__Sequence recommendations;
  /// When check was performed
  rosidl_runtime_c__String timestamp;
} autonomy_interfaces__srv__NavigationIntegrityCheck_Response;

// Struct for a sequence of autonomy_interfaces__srv__NavigationIntegrityCheck_Response.
typedef struct autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_H_
