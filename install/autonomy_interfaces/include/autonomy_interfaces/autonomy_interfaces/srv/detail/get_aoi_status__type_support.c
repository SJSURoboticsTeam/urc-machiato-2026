// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/get_aoi_status__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/get_aoi_status__functions.h"
#include "autonomy_interfaces/srv/detail/get_aoi_status__struct.h"


// Include directives for member types
// Member `sensor_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetAOIStatus_Request__init(message_memory);
}

void autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetAOIStatus_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_member_array[3] = {
  {
    "sensor_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Request, sensor_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "include_history",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Request, include_history),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "history_samples",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Request, history_samples),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetAOIStatus_Request",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__GetAOIStatus_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_member_array,  // message members
  autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetAOIStatus_Request__get_type_hash,
  &autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description,
  &autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Request)() {
  if (!autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `sensor_status`
#include "autonomy_interfaces/msg/aoi_status.h"
// Member `sensor_status`
#include "autonomy_interfaces/msg/detail/aoi_status__rosidl_typesupport_introspection_c.h"
// Member `aoi_history`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `timestamp_history`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp_history`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `system_metrics`
#include "autonomy_interfaces/msg/aoi_metrics.h"
// Member `system_metrics`
#include "autonomy_interfaces/msg/detail/aoi_metrics__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetAOIStatus_Response__init(message_memory);
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetAOIStatus_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__sensor_status(
  const void * untyped_member)
{
  const autonomy_interfaces__msg__AOIStatus__Sequence * member =
    (const autonomy_interfaces__msg__AOIStatus__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__sensor_status(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__msg__AOIStatus__Sequence * member =
    (const autonomy_interfaces__msg__AOIStatus__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__sensor_status(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__msg__AOIStatus__Sequence * member =
    (autonomy_interfaces__msg__AOIStatus__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__sensor_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__msg__AOIStatus * item =
    ((const autonomy_interfaces__msg__AOIStatus *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__sensor_status(untyped_member, index));
  autonomy_interfaces__msg__AOIStatus * value =
    (autonomy_interfaces__msg__AOIStatus *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__sensor_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__msg__AOIStatus * item =
    ((autonomy_interfaces__msg__AOIStatus *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__sensor_status(untyped_member, index));
  const autonomy_interfaces__msg__AOIStatus * value =
    (const autonomy_interfaces__msg__AOIStatus *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__sensor_status(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__msg__AOIStatus__Sequence * member =
    (autonomy_interfaces__msg__AOIStatus__Sequence *)(untyped_member);
  autonomy_interfaces__msg__AOIStatus__Sequence__fini(member);
  return autonomy_interfaces__msg__AOIStatus__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__aoi_history(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__aoi_history(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__aoi_history(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__aoi_history(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__aoi_history(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__aoi_history(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__aoi_history(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__aoi_history(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__timestamp_history(
  const void * untyped_member)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__timestamp_history(
  const void * untyped_member, size_t index)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__timestamp_history(
  void * untyped_member, size_t index)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__timestamp_history(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const builtin_interfaces__msg__Time * item =
    ((const builtin_interfaces__msg__Time *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__timestamp_history(untyped_member, index));
  builtin_interfaces__msg__Time * value =
    (builtin_interfaces__msg__Time *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__timestamp_history(
  void * untyped_member, size_t index, const void * untyped_value)
{
  builtin_interfaces__msg__Time * item =
    ((builtin_interfaces__msg__Time *)
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__timestamp_history(untyped_member, index));
  const builtin_interfaces__msg__Time * value =
    (const builtin_interfaces__msg__Time *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__timestamp_history(
  void * untyped_member, size_t size)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  builtin_interfaces__msg__Time__Sequence__fini(member);
  return builtin_interfaces__msg__Time__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, success),  // bytes offset in struct
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
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sensor_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, sensor_status),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__sensor_status,  // size() function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__sensor_status,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__sensor_status,  // get(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__sensor_status,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__sensor_status,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__sensor_status  // resize(index) function pointer
  },
  {
    "aoi_history",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, aoi_history),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__aoi_history,  // size() function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__aoi_history,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__aoi_history,  // get(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__aoi_history,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__aoi_history,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__aoi_history  // resize(index) function pointer
  },
  {
    "timestamp_history",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, timestamp_history),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Response__timestamp_history,  // size() function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Response__timestamp_history,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Response__timestamp_history,  // get(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Response__timestamp_history,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Response__timestamp_history,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Response__timestamp_history  // resize(index) function pointer
  },
  {
    "system_metrics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Response, system_metrics),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetAOIStatus_Response",  // message name
  6,  // number of fields
  sizeof(autonomy_interfaces__srv__GetAOIStatus_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_member_array,  // message members
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetAOIStatus_Response__get_type_hash,
  &autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description,
  &autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Response)() {
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, AOIStatus)();
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, AOIMetrics)();
  if (!autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/get_aoi_status.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetAOIStatus_Event__init(message_memory);
}

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetAOIStatus_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * member =
    (const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * member =
    (const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * member =
    (autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__GetAOIStatus_Request * item =
    ((const autonomy_interfaces__srv__GetAOIStatus_Request *)
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__request(untyped_member, index));
  autonomy_interfaces__srv__GetAOIStatus_Request * value =
    (autonomy_interfaces__srv__GetAOIStatus_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__GetAOIStatus_Request * item =
    ((autonomy_interfaces__srv__GetAOIStatus_Request *)
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__GetAOIStatus_Request * value =
    (const autonomy_interfaces__srv__GetAOIStatus_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * member =
    (autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * member =
    (const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * member =
    (const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * member =
    (autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__GetAOIStatus_Response * item =
    ((const autonomy_interfaces__srv__GetAOIStatus_Response *)
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__response(untyped_member, index));
  autonomy_interfaces__srv__GetAOIStatus_Response * value =
    (autonomy_interfaces__srv__GetAOIStatus_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__GetAOIStatus_Response * item =
    ((autonomy_interfaces__srv__GetAOIStatus_Response *)
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__GetAOIStatus_Response * value =
    (const autonomy_interfaces__srv__GetAOIStatus_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * member =
    (autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Event__request,  // size() function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAOIStatus_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__size_function__GetAOIStatus_Event__response,  // size() function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetAOIStatus_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__get_function__GetAOIStatus_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetAOIStatus_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetAOIStatus_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetAOIStatus_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetAOIStatus_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__GetAOIStatus_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_member_array,  // message members
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetAOIStatus_Event__get_type_hash,
  &autonomy_interfaces__srv__GetAOIStatus_Event__get_type_description,
  &autonomy_interfaces__srv__GetAOIStatus_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Event)() {
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Request)();
  autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Response)();
  if (!autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_aoi_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "GetAOIStatus",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__GetAOIStatus_Request__rosidl_typesupport_introspection_c__GetAOIStatus_Request_message_type_support_handle,
  &autonomy_interfaces__srv__GetAOIStatus_Response__rosidl_typesupport_introspection_c__GetAOIStatus_Response_message_type_support_handle,
  &autonomy_interfaces__srv__GetAOIStatus_Event__rosidl_typesupport_introspection_c__GetAOIStatus_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    GetAOIStatus
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    GetAOIStatus
  ),
  &autonomy_interfaces__srv__GetAOIStatus__get_type_hash,
  &autonomy_interfaces__srv__GetAOIStatus__get_type_description,
  &autonomy_interfaces__srv__GetAOIStatus__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus)(void) {
  if (!autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAOIStatus_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__get_aoi_status__rosidl_typesupport_introspection_c__GetAOIStatus_service_type_support_handle;
}
