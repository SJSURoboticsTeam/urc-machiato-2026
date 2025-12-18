// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/get_system_state__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/get_system_state__functions.h"
#include "autonomy_interfaces/srv/detail/get_system_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetSystemState_Request__init(message_memory);
}

void autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetSystemState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_member_array[3] = {
  {
    "include_history",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Request, include_history),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "include_subsystems",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Request, include_subsystems),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "history_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Request, history_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetSystemState_Request",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__GetSystemState_Request),
  autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_member_array,  // message members
  autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Request)() {
  if (!autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetSystemState_Request__rosidl_typesupport_introspection_c__GetSystemState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/get_system_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_system_state__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_system_state__struct.h"


// Include directives for member types
// Member `message`
// Member `current_state`
// Member `substate`
// Member `sub_substate`
// Member `recent_states`
// Member `transition_reasons`
// Member `active_subsystems`
// Member `inactive_subsystems`
// Member `failed_subsystems`
#include "rosidl_runtime_c/string_functions.h"
// Member `state_entered`
// Member `state_timestamps`
#include "builtin_interfaces/msg/time.h"
// Member `state_entered`
// Member `state_timestamps`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetSystemState_Response__init(message_memory);
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetSystemState_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__recent_states(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__recent_states(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__recent_states(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__recent_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__recent_states(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__recent_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__recent_states(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__recent_states(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__state_timestamps(
  const void * untyped_member)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__state_timestamps(
  const void * untyped_member, size_t index)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__state_timestamps(
  void * untyped_member, size_t index)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__state_timestamps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const builtin_interfaces__msg__Time * item =
    ((const builtin_interfaces__msg__Time *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__state_timestamps(untyped_member, index));
  builtin_interfaces__msg__Time * value =
    (builtin_interfaces__msg__Time *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__state_timestamps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  builtin_interfaces__msg__Time * item =
    ((builtin_interfaces__msg__Time *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__state_timestamps(untyped_member, index));
  const builtin_interfaces__msg__Time * value =
    (const builtin_interfaces__msg__Time *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__state_timestamps(
  void * untyped_member, size_t size)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  builtin_interfaces__msg__Time__Sequence__fini(member);
  return builtin_interfaces__msg__Time__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__transition_reasons(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__transition_reasons(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__transition_reasons(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__transition_reasons(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__transition_reasons(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__transition_reasons(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__transition_reasons(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__transition_reasons(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__active_subsystems(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__active_subsystems(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__active_subsystems(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__active_subsystems(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__active_subsystems(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__active_subsystems(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__active_subsystems(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__active_subsystems(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__inactive_subsystems(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__inactive_subsystems(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__inactive_subsystems(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__inactive_subsystems(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__inactive_subsystems(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__inactive_subsystems(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__inactive_subsystems(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__inactive_subsystems(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__failed_subsystems(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__failed_subsystems(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__failed_subsystems(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__failed_subsystems(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__failed_subsystems(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__failed_subsystems(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__failed_subsystems(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__failed_subsystems(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_member_array[13] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, current_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sub_substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, sub_substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_in_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, time_in_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state_entered",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, state_entered),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recent_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, recent_states),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__recent_states,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__recent_states,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__recent_states,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__recent_states,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__recent_states,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__recent_states  // resize(index) function pointer
  },
  {
    "state_timestamps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, state_timestamps),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__state_timestamps,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__state_timestamps,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__state_timestamps,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__state_timestamps,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__state_timestamps,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__state_timestamps  // resize(index) function pointer
  },
  {
    "transition_reasons",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, transition_reasons),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__transition_reasons,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__transition_reasons,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__transition_reasons,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__transition_reasons,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__transition_reasons,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__transition_reasons  // resize(index) function pointer
  },
  {
    "active_subsystems",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, active_subsystems),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__active_subsystems,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__active_subsystems,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__active_subsystems,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__active_subsystems,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__active_subsystems,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__active_subsystems  // resize(index) function pointer
  },
  {
    "inactive_subsystems",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, inactive_subsystems),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__inactive_subsystems,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__inactive_subsystems,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__inactive_subsystems,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__inactive_subsystems,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__inactive_subsystems,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__inactive_subsystems  // resize(index) function pointer
  },
  {
    "failed_subsystems",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetSystemState_Response, failed_subsystems),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__size_function__GetSystemState_Response__failed_subsystems,  // size() function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_const_function__GetSystemState_Response__failed_subsystems,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__get_function__GetSystemState_Response__failed_subsystems,  // get(index) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__fetch_function__GetSystemState_Response__failed_subsystems,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__assign_function__GetSystemState_Response__failed_subsystems,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__resize_function__GetSystemState_Response__failed_subsystems  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetSystemState_Response",  // message name
  13,  // number of fields
  sizeof(autonomy_interfaces__srv__GetSystemState_Response),
  autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_member_array,  // message members
  autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Response)() {
  autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetSystemState_Response__rosidl_typesupport_introspection_c__GetSystemState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_system_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "GetSystemState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState)() {
  if (!autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetSystemState_Response)()->data;
  }

  return &autonomy_interfaces__srv__detail__get_system_state__rosidl_typesupport_introspection_c__GetSystemState_service_type_support_handle;
}
