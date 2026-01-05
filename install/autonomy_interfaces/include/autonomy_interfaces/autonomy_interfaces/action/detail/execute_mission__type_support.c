// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:action/ExecuteMission.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/action/detail/execute_mission__functions.h"
#include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `mission_type`
// Member `mission_id`
// Member `waypoints`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_Goal__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_Goal__fini(message_memory);
}

size_t autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__size_function__ExecuteMission_Goal__waypoints(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Goal__waypoints(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Goal__waypoints(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_Goal__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Goal__waypoints(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_Goal__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Goal__waypoints(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_Goal__waypoints(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_member_array[4] = {
  {
    "mission_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Goal, mission_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Goal, mission_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Goal, waypoints),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__size_function__ExecuteMission_Goal__waypoints,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Goal__waypoints,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Goal__waypoints,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_Goal__waypoints,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_Goal__waypoints,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_Goal__waypoints  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Goal, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_Goal",  // message name
  4,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_Goal),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_Goal__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_Goal__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Goal)() {
  if (!autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_Goal__rosidl_typesupport_introspection_c__ExecuteMission_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `current_phase`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_Result__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_member_array[5] = {
  {
    "current_phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Result, current_phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Result, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Result, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints_completed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Result, waypoints_completed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_time_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Result, estimated_time_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_Result",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_Result),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_Result__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_Result__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_Result__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Result)() {
  if (!autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_Result__rosidl_typesupport_introspection_c__ExecuteMission_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `completion_status`
// Member `completed_tasks`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_Feedback__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_Feedback__fini(message_memory);
}

size_t autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__size_function__ExecuteMission_Feedback__completed_tasks(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Feedback__completed_tasks(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Feedback__completed_tasks(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_Feedback__completed_tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Feedback__completed_tasks(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_Feedback__completed_tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Feedback__completed_tasks(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_Feedback__completed_tasks(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Feedback, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "completion_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Feedback, completion_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "completed_tasks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Feedback, completed_tasks),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__size_function__ExecuteMission_Feedback__completed_tasks,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_Feedback__completed_tasks,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__get_function__ExecuteMission_Feedback__completed_tasks,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_Feedback__completed_tasks,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_Feedback__completed_tasks,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_Feedback__completed_tasks  // resize(index) function pointer
  },
  {
    "total_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Feedback, total_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints_visited",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_Feedback, waypoints_visited),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_Feedback",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_Feedback),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_Feedback__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_Feedback__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_Feedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Feedback)() {
  if (!autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_Feedback__rosidl_typesupport_introspection_c__ExecuteMission_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "autonomy_interfaces/action/execute_mission.h"
// Member `goal`
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_SendGoal_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Request)() {
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Goal)();
  if (!autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_SendGoal_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Response)() {
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/action/execute_mission.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__fini(message_memory);
}

size_t autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_SendGoal_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_SendGoal_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Request * item =
    ((const autonomy_interfaces__action__ExecuteMission_SendGoal_Request *)
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__request(untyped_member, index));
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request * value =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_SendGoal_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request * item =
    ((autonomy_interfaces__action__ExecuteMission_SendGoal_Request *)
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__request(untyped_member, index));
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Request * value =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_SendGoal_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence *)(untyped_member);
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence__fini(member);
  return autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_SendGoal_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_SendGoal_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Response * item =
    ((const autonomy_interfaces__action__ExecuteMission_SendGoal_Response *)
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__response(untyped_member, index));
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response * value =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_SendGoal_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response * item =
    ((autonomy_interfaces__action__ExecuteMission_SendGoal_Response *)
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__response(untyped_member, index));
  const autonomy_interfaces__action__ExecuteMission_SendGoal_Response * value =
    (const autonomy_interfaces__action__ExecuteMission_SendGoal_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_SendGoal_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence *)(untyped_member);
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence__fini(member);
  return autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_SendGoal_Event__request,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__request,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_SendGoal_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_SendGoal_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_SendGoal_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__action__ExecuteMission_SendGoal_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_SendGoal_Event__response,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_SendGoal_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_SendGoal_Event__response,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_SendGoal_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_SendGoal_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_SendGoal_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_SendGoal_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_SendGoal_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Event)() {
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Request)();
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Response)();
  if (!autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_members = {
  "autonomy_interfaces__action",  // service namespace
  "ExecuteMission_SendGoal",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_type_support_handle = {
  0,
  &autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Request__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Request_message_type_support_handle,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Response__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Response_message_type_support_handle,
  &autonomy_interfaces__action__ExecuteMission_SendGoal_Event__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    action,
    ExecuteMission_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    action,
    ExecuteMission_SendGoal
  ),
  &autonomy_interfaces__action__ExecuteMission_SendGoal__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_SendGoal__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_SendGoal__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal)(void) {
  if (!autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_SendGoal_Event)()->data;
  }

  return &autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_GetResult_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Request__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Request__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Request)() {
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "autonomy_interfaces/action/execute_mission.h"
// Member `result`
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_GetResult_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Response__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Response__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Response)() {
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Result)();
  if (!autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/service_event_info.h"
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/action/execute_mission.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__fini(message_memory);
}

size_t autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_GetResult_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_GetResult_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Request * item =
    ((const autonomy_interfaces__action__ExecuteMission_GetResult_Request *)
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__request(untyped_member, index));
  autonomy_interfaces__action__ExecuteMission_GetResult_Request * value =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_GetResult_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Request * item =
    ((autonomy_interfaces__action__ExecuteMission_GetResult_Request *)
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__request(untyped_member, index));
  const autonomy_interfaces__action__ExecuteMission_GetResult_Request * value =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_GetResult_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence *)(untyped_member);
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence__fini(member);
  return autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_GetResult_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence * member =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_GetResult_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__action__ExecuteMission_GetResult_Response * item =
    ((const autonomy_interfaces__action__ExecuteMission_GetResult_Response *)
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__response(untyped_member, index));
  autonomy_interfaces__action__ExecuteMission_GetResult_Response * value =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_GetResult_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Response * item =
    ((autonomy_interfaces__action__ExecuteMission_GetResult_Response *)
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__response(untyped_member, index));
  const autonomy_interfaces__action__ExecuteMission_GetResult_Response * value =
    (const autonomy_interfaces__action__ExecuteMission_GetResult_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_GetResult_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence * member =
    (autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence *)(untyped_member);
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence__fini(member);
  return autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_GetResult_Event__request,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__request,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_GetResult_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_GetResult_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_GetResult_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__action__ExecuteMission_GetResult_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExecuteMission_GetResult_Event__response,  // size() function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExecuteMission_GetResult_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExecuteMission_GetResult_Event__response,  // get(index) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExecuteMission_GetResult_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExecuteMission_GetResult_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExecuteMission_GetResult_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_GetResult_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_GetResult_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Event__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Event__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Event)() {
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Request)();
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Response)();
  if (!autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_members = {
  "autonomy_interfaces__action",  // service namespace
  "ExecuteMission_GetResult",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_type_support_handle = {
  0,
  &autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Request__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Request_message_type_support_handle,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Response__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Response_message_type_support_handle,
  &autonomy_interfaces__action__ExecuteMission_GetResult_Event__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    action,
    ExecuteMission_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    action,
    ExecuteMission_GetResult
  ),
  &autonomy_interfaces__action__ExecuteMission_GetResult__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_GetResult__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_GetResult__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult)(void) {
  if (!autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_GetResult_Event)()->data;
  }

  return &autonomy_interfaces__action__detail__execute_mission__rosidl_typesupport_introspection_c__ExecuteMission_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "autonomy_interfaces/action/execute_mission.h"
// Member `feedback`
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__init(message_memory);
}

void autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_fini_function(void * message_memory)
{
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__ExecuteMission_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "ExecuteMission_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__ExecuteMission_FeedbackMessage),
  false,  // has_any_key_member_
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_member_array,  // message members
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__action__ExecuteMission_FeedbackMessage__get_type_hash,
  &autonomy_interfaces__action__ExecuteMission_FeedbackMessage__get_type_description,
  &autonomy_interfaces__action__ExecuteMission_FeedbackMessage__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_FeedbackMessage)() {
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, ExecuteMission_Feedback)();
  if (!autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__ExecuteMission_FeedbackMessage__rosidl_typesupport_introspection_c__ExecuteMission_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
