// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetAdaptationHistory.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetAdaptationHistory in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetAdaptationHistory_Request
{
  /// Maximum number of entries to return (0 = all)
  int32_t limit;
  /// Include full context with each action
  bool include_context;
} autonomy_interfaces__srv__GetAdaptationHistory_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetAdaptationHistory_Request.
typedef struct autonomy_interfaces__srv__GetAdaptationHistory_Request__Sequence
{
  autonomy_interfaces__srv__GetAdaptationHistory_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetAdaptationHistory_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'actions'
#include "autonomy_interfaces/msg/detail/adaptive_action__struct.h"
// Member 'contexts'
#include "autonomy_interfaces/msg/detail/context_state__struct.h"

/// Struct defined in srv/GetAdaptationHistory in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetAdaptationHistory_Response
{
  autonomy_interfaces__msg__AdaptiveAction__Sequence actions;
  autonomy_interfaces__msg__ContextState__Sequence contexts;
} autonomy_interfaces__srv__GetAdaptationHistory_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetAdaptationHistory_Response.
typedef struct autonomy_interfaces__srv__GetAdaptationHistory_Response__Sequence
{
  autonomy_interfaces__srv__GetAdaptationHistory_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetAdaptationHistory_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__STRUCT_H_
