// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/safety_status__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/safety_status__functions.h"
#include "autonomy_interfaces/msg/detail/safety_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `safety_level`
// Member `active_triggers`
// Member `trigger_type`
// Member `trigger_source`
// Member `trigger_description`
// Member `recovery_steps`
// Member `context_state`
// Member `mission_phase`
// Member `degraded_capabilities`
#include "rosidl_runtime_c/string_functions.h"
// Member `trigger_time`
#include "builtin_interfaces/msg/time.h"
// Member `trigger_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__SafetyStatus__init(message_memory);
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__SafetyStatus__fini(message_memory);
}

size_t autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__active_triggers(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__active_triggers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__active_triggers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__active_triggers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__active_triggers(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__active_triggers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__active_triggers(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__active_triggers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__recovery_steps(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__recovery_steps(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__recovery_steps(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__recovery_steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__recovery_steps(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__recovery_steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__recovery_steps(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__recovery_steps(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__degraded_capabilities(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__degraded_capabilities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__degraded_capabilities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__degraded_capabilities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__degraded_capabilities(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__degraded_capabilities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__degraded_capabilities(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__degraded_capabilities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_member_array[20] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_safe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, is_safe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, safety_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "active_triggers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, active_triggers),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__active_triggers,  // size() function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__active_triggers,  // get_const(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__active_triggers,  // get(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__active_triggers,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__active_triggers,  // assign(index, value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__active_triggers  // resize(index) function pointer
  },
  {
    "trigger_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, trigger_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trigger_source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, trigger_source),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trigger_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, trigger_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trigger_description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, trigger_description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "requires_manual_intervention",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, requires_manual_intervention),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "can_auto_recover",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, can_auto_recover),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recovery_steps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, recovery_steps),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__recovery_steps,  // size() function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__recovery_steps,  // get_const(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__recovery_steps,  // get(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__recovery_steps,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__recovery_steps,  // assign(index, value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__recovery_steps  // resize(index) function pointer
  },
  {
    "estimated_recovery_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, estimated_recovery_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "context_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, context_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, mission_phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safe_to_retry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, safe_to_retry),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, battery_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "communication_ok",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, communication_ok),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sensors_ok",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, sensors_ok),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "degraded_capabilities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__SafetyStatus, degraded_capabilities),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__size_function__SafetyStatus__degraded_capabilities,  // size() function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_const_function__SafetyStatus__degraded_capabilities,  // get_const(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__get_function__SafetyStatus__degraded_capabilities,  // get(index) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__fetch_function__SafetyStatus__degraded_capabilities,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__assign_function__SafetyStatus__degraded_capabilities,  // assign(index, value) function pointer
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__resize_function__SafetyStatus__degraded_capabilities  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "SafetyStatus",  // message name
  20,  // number of fields
  sizeof(autonomy_interfaces__msg__SafetyStatus),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_member_array,  // message members
  autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__SafetyStatus__get_type_hash,
  &autonomy_interfaces__msg__SafetyStatus__get_type_description,
  &autonomy_interfaces__msg__SafetyStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, SafetyStatus)() {
  autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__SafetyStatus__rosidl_typesupport_introspection_c__SafetyStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
