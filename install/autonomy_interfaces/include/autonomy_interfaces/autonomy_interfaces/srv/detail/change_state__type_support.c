// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/change_state__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/change_state__functions.h"
#include "autonomy_interfaces/srv/detail/change_state__struct.h"


// Include directives for member types
// Member `desired_state`
// Member `desired_substate`
// Member `desired_calibration_substate`
// Member `reason`
// Member `operator_id`
// Member `metadata`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ChangeState_Request__init(message_memory);
}

void autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ChangeState_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__size_function__ChangeState_Request__metadata(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Request__metadata(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_function__ChangeState_Request__metadata(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Request__metadata(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Request__metadata(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__assign_function__ChangeState_Request__metadata(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_function__ChangeState_Request__metadata(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__resize_function__ChangeState_Request__metadata(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_member_array[7] = {
  {
    "desired_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, desired_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "desired_substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, desired_substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "desired_calibration_substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, desired_calibration_substate),  // bytes offset in struct
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
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operator_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, operator_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "metadata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Request, metadata),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__size_function__ChangeState_Request__metadata,  // size() function pointer
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Request__metadata,  // get_const(index) function pointer
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__get_function__ChangeState_Request__metadata,  // get(index) function pointer
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Request__metadata,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__assign_function__ChangeState_Request__metadata,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__resize_function__ChangeState_Request__metadata  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ChangeState_Request",  // message name
  7,  // number of fields
  sizeof(autonomy_interfaces__srv__ChangeState_Request),
  autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_member_array,  // message members
  autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Request)() {
  if (!autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ChangeState_Request__rosidl_typesupport_introspection_c__ChangeState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/change_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/change_state__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/change_state__struct.h"


// Include directives for member types
// Member `actual_state`
// Member `actual_substate`
// Member `actual_calibration_substate`
// Member `message`
// Member `failed_preconditions`
// Member `warnings`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ChangeState_Response__init(message_memory);
}

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ChangeState_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__size_function__ChangeState_Response__failed_preconditions(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__failed_preconditions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__failed_preconditions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Response__failed_preconditions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__failed_preconditions(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__assign_function__ChangeState_Response__failed_preconditions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__failed_preconditions(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__resize_function__ChangeState_Response__failed_preconditions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__size_function__ChangeState_Response__warnings(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__warnings(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__warnings(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Response__warnings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__warnings(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__assign_function__ChangeState_Response__warnings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__warnings(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__resize_function__ChangeState_Response__warnings(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_member_array[9] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, actual_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual_substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, actual_substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual_calibration_substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, actual_calibration_substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transition_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, transition_time),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__ChangeState_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "preconditions_met",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, preconditions_met),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failed_preconditions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, failed_preconditions),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__size_function__ChangeState_Response__failed_preconditions,  // size() function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__failed_preconditions,  // get_const(index) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__failed_preconditions,  // get(index) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Response__failed_preconditions,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__assign_function__ChangeState_Response__failed_preconditions,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__resize_function__ChangeState_Response__failed_preconditions  // resize(index) function pointer
  },
  {
    "warnings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ChangeState_Response, warnings),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__size_function__ChangeState_Response__warnings,  // size() function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_const_function__ChangeState_Response__warnings,  // get_const(index) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__get_function__ChangeState_Response__warnings,  // get(index) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__fetch_function__ChangeState_Response__warnings,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__assign_function__ChangeState_Response__warnings,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__resize_function__ChangeState_Response__warnings  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ChangeState_Response",  // message name
  9,  // number of fields
  sizeof(autonomy_interfaces__srv__ChangeState_Response),
  autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_member_array,  // message members
  autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Response)() {
  if (!autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ChangeState_Response__rosidl_typesupport_introspection_c__ChangeState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/change_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "ChangeState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState)() {
  if (!autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ChangeState_Response)()->data;
  }

  return &autonomy_interfaces__srv__detail__change_state__rosidl_typesupport_introspection_c__ChangeState_service_type_support_handle;
}
