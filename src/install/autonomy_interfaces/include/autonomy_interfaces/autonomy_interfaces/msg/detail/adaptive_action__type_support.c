// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/adaptive_action__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/adaptive_action__functions.h"
#include "autonomy_interfaces/msg/detail/adaptive_action__struct.h"


// Include directives for member types
// Member `action_type`
// Member `parameters`
// Member `success_criteria`
#include "rosidl_runtime_c/string_functions.h"
// Member `trigger_context`
#include "autonomy_interfaces/msg/context_state.h"
// Member `trigger_context`
#include "autonomy_interfaces/msg/detail/context_state__rosidl_typesupport_introspection_c.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__AdaptiveAction__init(message_memory);
}

void autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__AdaptiveAction__fini(message_memory);
}

size_t autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__size_function__AdaptiveAction__parameters(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_const_function__AdaptiveAction__parameters(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_function__AdaptiveAction__parameters(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__fetch_function__AdaptiveAction__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_const_function__AdaptiveAction__parameters(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__assign_function__AdaptiveAction__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_function__AdaptiveAction__parameters(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__resize_function__AdaptiveAction__parameters(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_member_array[7] = {
  {
    "action_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, action_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, parameters),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__size_function__AdaptiveAction__parameters,  // size() function pointer
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_const_function__AdaptiveAction__parameters,  // get_const(index) function pointer
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__get_function__AdaptiveAction__parameters,  // get(index) function pointer
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__fetch_function__AdaptiveAction__parameters,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__assign_function__AdaptiveAction__parameters,  // assign(index, value) function pointer
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__resize_function__AdaptiveAction__parameters  // resize(index) function pointer
  },
  {
    "trigger_context",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, trigger_context),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "priority",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, priority),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "expected_duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, expected_duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success_criteria",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, success_criteria),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__AdaptiveAction, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "AdaptiveAction",  // message name
  7,  // number of fields
  sizeof(autonomy_interfaces__msg__AdaptiveAction),
  autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_member_array,  // message members
  autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, AdaptiveAction)() {
  autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, ContextState)();
  autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__AdaptiveAction__rosidl_typesupport_introspection_c__AdaptiveAction_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
