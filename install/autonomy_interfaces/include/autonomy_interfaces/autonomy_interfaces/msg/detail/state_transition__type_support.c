// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/state_transition__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/state_transition__functions.h"
#include "autonomy_interfaces/msg/detail/state_transition__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `from_state`
// Member `to_state`
// Member `reason`
// Member `initiated_by`
// Member `failure_reason`
// Member `preconditions_checked`
// Member `entry_actions_executed`
// Member `exit_actions_executed`
#include "rosidl_runtime_c/string_functions.h"
// Member `start_time`
// Member `end_time`
#include "builtin_interfaces/msg/time.h"
// Member `start_time`
// Member `end_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__StateTransition__init(message_memory);
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__StateTransition__fini(message_memory);
}

size_t autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__preconditions_checked(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__preconditions_checked(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__preconditions_checked(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__preconditions_checked(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__preconditions_checked(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__preconditions_checked(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__preconditions_checked(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__preconditions_checked(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__entry_actions_executed(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__entry_actions_executed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__entry_actions_executed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__entry_actions_executed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__entry_actions_executed(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__entry_actions_executed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__entry_actions_executed(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__entry_actions_executed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__exit_actions_executed(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__exit_actions_executed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__exit_actions_executed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__exit_actions_executed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__exit_actions_executed(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__exit_actions_executed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__exit_actions_executed(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__exit_actions_executed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_member_array[13] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "from_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, from_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "to_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, to_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, start_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, end_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transition_duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, transition_duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initiated_by",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, initiated_by),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failure_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, failure_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "preconditions_checked",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, preconditions_checked),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__preconditions_checked,  // size() function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__preconditions_checked,  // get_const(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__preconditions_checked,  // get(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__preconditions_checked,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__preconditions_checked,  // assign(index, value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__preconditions_checked  // resize(index) function pointer
  },
  {
    "entry_actions_executed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, entry_actions_executed),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__entry_actions_executed,  // size() function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__entry_actions_executed,  // get_const(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__entry_actions_executed,  // get(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__entry_actions_executed,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__entry_actions_executed,  // assign(index, value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__entry_actions_executed  // resize(index) function pointer
  },
  {
    "exit_actions_executed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__StateTransition, exit_actions_executed),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__size_function__StateTransition__exit_actions_executed,  // size() function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_const_function__StateTransition__exit_actions_executed,  // get_const(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__get_function__StateTransition__exit_actions_executed,  // get(index) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__fetch_function__StateTransition__exit_actions_executed,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__assign_function__StateTransition__exit_actions_executed,  // assign(index, value) function pointer
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__resize_function__StateTransition__exit_actions_executed  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "StateTransition",  // message name
  13,  // number of fields
  sizeof(autonomy_interfaces__msg__StateTransition),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_member_array,  // message members
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__StateTransition__get_type_hash,
  &autonomy_interfaces__msg__StateTransition__get_type_description,
  &autonomy_interfaces__msg__StateTransition__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, StateTransition)() {
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__StateTransition__rosidl_typesupport_introspection_c__StateTransition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
