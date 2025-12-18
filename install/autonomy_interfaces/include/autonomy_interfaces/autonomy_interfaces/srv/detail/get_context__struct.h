// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetContext.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetContext in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetContext_Request
{
  uint8_t structure_needs_at_least_one_member;
} autonomy_interfaces__srv__GetContext_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetContext_Request.
typedef struct autonomy_interfaces__srv__GetContext_Request__Sequence
{
  autonomy_interfaces__srv__GetContext_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetContext_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'context'
#include "autonomy_interfaces/msg/detail/context_state__struct.h"

/// Struct defined in srv/GetContext in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetContext_Response
{
  autonomy_interfaces__msg__ContextState context;
} autonomy_interfaces__srv__GetContext_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetContext_Response.
typedef struct autonomy_interfaces__srv__GetContext_Response__Sequence
{
  autonomy_interfaces__srv__GetContext_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetContext_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__STRUCT_H_
