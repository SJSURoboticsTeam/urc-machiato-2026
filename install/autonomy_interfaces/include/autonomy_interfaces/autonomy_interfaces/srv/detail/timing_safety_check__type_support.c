// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/timing_safety_check__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"
#include "autonomy_interfaces/srv/detail/timing_safety_check__struct.h"


// Include directives for member types
// Member `monitored_components`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__TimingSafetyCheck_Request__init(message_memory);
}

void autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Request__monitored_components(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Request__monitored_components(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Request__monitored_components(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Request__monitored_components(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Request__monitored_components(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Request__monitored_components(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Request__monitored_components(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Request__monitored_components(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_member_array[3] = {
  {
    "real_time_check",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Request, real_time_check),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_window",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Request, time_window),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "monitored_components",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Request, monitored_components),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Request__monitored_components,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Request__monitored_components,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Request__monitored_components,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Request__monitored_components,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Request__monitored_components,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Request__monitored_components  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "TimingSafetyCheck_Request",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_member_array,  // message members
  autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_hash,
  &autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description,
  &autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Request)() {
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__struct.h"


// Include directives for member types
// Member `timing_status`
// Member `components_checked`
// Member `timing_recommendations`
// Member `timestamp`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `component_avg_times`
// Member `component_deadlines_missed`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__TimingSafetyCheck_Response__init(message_memory);
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__components_checked(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__components_checked(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__components_checked(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__components_checked(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__components_checked(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__components_checked(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__components_checked(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__components_checked(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__component_avg_times(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_avg_times(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_avg_times(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__component_avg_times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_avg_times(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__component_avg_times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_avg_times(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__component_avg_times(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__component_deadlines_missed(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_deadlines_missed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_deadlines_missed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__component_deadlines_missed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_deadlines_missed(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__component_deadlines_missed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_deadlines_missed(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__component_deadlines_missed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__timing_recommendations(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__timing_recommendations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__timing_recommendations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__timing_recommendations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__timing_recommendations(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__timing_recommendations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__timing_recommendations(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__timing_recommendations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_member_array[17] = {
  {
    "timing_safe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, timing_safe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timing_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, timing_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "avg_response_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, avg_response_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_response_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, max_response_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_response_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, min_response_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "jitter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, jitter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deadline_misses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, deadline_misses),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deadline_miss_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, deadline_miss_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "components_checked",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, components_checked),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__components_checked,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__components_checked,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__components_checked,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__components_checked,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__components_checked,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__components_checked  // resize(index) function pointer
  },
  {
    "component_avg_times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, component_avg_times),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__component_avg_times,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_avg_times,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_avg_times,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__component_avg_times,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__component_avg_times,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__component_avg_times  // resize(index) function pointer
  },
  {
    "component_deadlines_missed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, component_deadlines_missed),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__component_deadlines_missed,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__component_deadlines_missed,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__component_deadlines_missed,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__component_deadlines_missed,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__component_deadlines_missed,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__component_deadlines_missed  // resize(index) function pointer
  },
  {
    "cpu_utilization",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, cpu_utilization),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "memory_utilization",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, memory_utilization),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "thread_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, thread_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "real_time_scheduling",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, real_time_scheduling),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timing_recommendations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, timing_recommendations),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Response__timing_recommendations,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Response__timing_recommendations,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Response__timing_recommendations,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Response__timing_recommendations,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Response__timing_recommendations,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Response__timing_recommendations  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Response, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "TimingSafetyCheck_Response",  // message name
  17,  // number of fields
  sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_member_array,  // message members
  autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_hash,
  &autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description,
  &autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Response)() {
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/timing_safety_check.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__TimingSafetyCheck_Event__init(message_memory);
}

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * member =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * member =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * member =
    (autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Request * item =
    ((const autonomy_interfaces__srv__TimingSafetyCheck_Request *)
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__request(untyped_member, index));
  autonomy_interfaces__srv__TimingSafetyCheck_Request * value =
    (autonomy_interfaces__srv__TimingSafetyCheck_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Request * item =
    ((autonomy_interfaces__srv__TimingSafetyCheck_Request *)
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__TimingSafetyCheck_Request * value =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * member =
    (autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * member =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * member =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * member =
    (autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__TimingSafetyCheck_Response * item =
    ((const autonomy_interfaces__srv__TimingSafetyCheck_Response *)
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__response(untyped_member, index));
  autonomy_interfaces__srv__TimingSafetyCheck_Response * value =
    (autonomy_interfaces__srv__TimingSafetyCheck_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Response * item =
    ((autonomy_interfaces__srv__TimingSafetyCheck_Response *)
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__TimingSafetyCheck_Response * value =
    (const autonomy_interfaces__srv__TimingSafetyCheck_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * member =
    (autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Event__request,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__srv__TimingSafetyCheck_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__size_function__TimingSafetyCheck_Event__response,  // size() function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_const_function__TimingSafetyCheck_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__get_function__TimingSafetyCheck_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__fetch_function__TimingSafetyCheck_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__assign_function__TimingSafetyCheck_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__resize_function__TimingSafetyCheck_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "TimingSafetyCheck_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_member_array,  // message members
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_hash,
  &autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_description,
  &autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Event)() {
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Request)();
  autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Response)();
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/timing_safety_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "TimingSafetyCheck",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__TimingSafetyCheck_Request__rosidl_typesupport_introspection_c__TimingSafetyCheck_Request_message_type_support_handle,
  &autonomy_interfaces__srv__TimingSafetyCheck_Response__rosidl_typesupport_introspection_c__TimingSafetyCheck_Response_message_type_support_handle,
  &autonomy_interfaces__srv__TimingSafetyCheck_Event__rosidl_typesupport_introspection_c__TimingSafetyCheck_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    TimingSafetyCheck
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    TimingSafetyCheck
  ),
  &autonomy_interfaces__srv__TimingSafetyCheck__get_type_hash,
  &autonomy_interfaces__srv__TimingSafetyCheck__get_type_description,
  &autonomy_interfaces__srv__TimingSafetyCheck__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck)(void) {
  if (!autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, TimingSafetyCheck_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__timing_safety_check__rosidl_typesupport_introspection_c__TimingSafetyCheck_service_type_support_handle;
}
