// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/context_update__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/context_update__functions.h"
#include "autonomy_interfaces/msg/detail/context_update__struct.h"


// Include directives for member types
// Member `mission_status`
// Member `active_adaptations`
// Member `alert_level`
// Member `available_actions`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__ContextUpdate__init(message_memory);
}

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__ContextUpdate__fini(message_memory);
}

size_t autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__size_function__ContextUpdate__active_adaptations(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__active_adaptations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__active_adaptations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__fetch_function__ContextUpdate__active_adaptations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__active_adaptations(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__assign_function__ContextUpdate__active_adaptations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__active_adaptations(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__resize_function__ContextUpdate__active_adaptations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__size_function__ContextUpdate__available_actions(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__available_actions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__available_actions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__fetch_function__ContextUpdate__available_actions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__available_actions(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__assign_function__ContextUpdate__available_actions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__available_actions(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__resize_function__ContextUpdate__available_actions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_member_array[9] = {
  {
    "battery_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, battery_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, mission_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, mission_progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "communication_active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, communication_active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, safety_active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "active_adaptations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, active_adaptations),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__size_function__ContextUpdate__active_adaptations,  // size() function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__active_adaptations,  // get_const(index) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__active_adaptations,  // get(index) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__fetch_function__ContextUpdate__active_adaptations,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__assign_function__ContextUpdate__active_adaptations,  // assign(index, value) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__resize_function__ContextUpdate__active_adaptations  // resize(index) function pointer
  },
  {
    "alert_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, alert_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "available_actions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, available_actions),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__size_function__ContextUpdate__available_actions,  // size() function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_const_function__ContextUpdate__available_actions,  // get_const(index) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__get_function__ContextUpdate__available_actions,  // get(index) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__fetch_function__ContextUpdate__available_actions,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__assign_function__ContextUpdate__available_actions,  // assign(index, value) function pointer
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__resize_function__ContextUpdate__available_actions  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__ContextUpdate, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "ContextUpdate",  // message name
  9,  // number of fields
  sizeof(autonomy_interfaces__msg__ContextUpdate),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_member_array,  // message members
  autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__ContextUpdate__get_type_hash,
  &autonomy_interfaces__msg__ContextUpdate__get_type_description,
  &autonomy_interfaces__msg__ContextUpdate__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, ContextUpdate)() {
  autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__ContextUpdate__rosidl_typesupport_introspection_c__ContextUpdate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
