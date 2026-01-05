// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/follow_me_control__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"
#include "autonomy_interfaces/srv/detail/follow_me_control__struct.h"


// Include directives for member types
// Member `operator_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__FollowMeControl_Request__init(message_memory);
}

void autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__FollowMeControl_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_member_array[5] = {
  {
    "target_tag_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Request, target_tag_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Request, safety_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Request, max_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enable_following",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Request, enable_following),  // bytes offset in struct
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
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Request, operator_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "FollowMeControl_Request",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces__srv__FollowMeControl_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_member_array,  // message members
  autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__FollowMeControl_Request__get_type_hash,
  &autonomy_interfaces__srv__FollowMeControl_Request__get_type_description,
  &autonomy_interfaces__srv__FollowMeControl_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Request)() {
  if (!autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `target_position`
#include "geometry_msgs/msg/point.h"
// Member `target_position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__FollowMeControl_Response__init(message_memory);
}

void autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__FollowMeControl_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, success),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_following",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, is_following),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_target_tag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, current_target_tag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, current_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Response, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "FollowMeControl_Response",  // message name
  6,  // number of fields
  sizeof(autonomy_interfaces__srv__FollowMeControl_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_member_array,  // message members
  autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__FollowMeControl_Response__get_type_hash,
  &autonomy_interfaces__srv__FollowMeControl_Response__get_type_description,
  &autonomy_interfaces__srv__FollowMeControl_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Response)() {
  autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/follow_me_control.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__FollowMeControl_Event__init(message_memory);
}

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__FollowMeControl_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__size_function__FollowMeControl_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__FollowMeControl_Request__Sequence * member =
    (const autonomy_interfaces__srv__FollowMeControl_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__FollowMeControl_Request__Sequence * member =
    (const autonomy_interfaces__srv__FollowMeControl_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence * member =
    (autonomy_interfaces__srv__FollowMeControl_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__fetch_function__FollowMeControl_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__FollowMeControl_Request * item =
    ((const autonomy_interfaces__srv__FollowMeControl_Request *)
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__request(untyped_member, index));
  autonomy_interfaces__srv__FollowMeControl_Request * value =
    (autonomy_interfaces__srv__FollowMeControl_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__assign_function__FollowMeControl_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__FollowMeControl_Request * item =
    ((autonomy_interfaces__srv__FollowMeControl_Request *)
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__FollowMeControl_Request * value =
    (const autonomy_interfaces__srv__FollowMeControl_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__resize_function__FollowMeControl_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence * member =
    (autonomy_interfaces__srv__FollowMeControl_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__FollowMeControl_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__size_function__FollowMeControl_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__FollowMeControl_Response__Sequence * member =
    (const autonomy_interfaces__srv__FollowMeControl_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__FollowMeControl_Response__Sequence * member =
    (const autonomy_interfaces__srv__FollowMeControl_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence * member =
    (autonomy_interfaces__srv__FollowMeControl_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__fetch_function__FollowMeControl_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__FollowMeControl_Response * item =
    ((const autonomy_interfaces__srv__FollowMeControl_Response *)
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__response(untyped_member, index));
  autonomy_interfaces__srv__FollowMeControl_Response * value =
    (autonomy_interfaces__srv__FollowMeControl_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__assign_function__FollowMeControl_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__FollowMeControl_Response * item =
    ((autonomy_interfaces__srv__FollowMeControl_Response *)
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__FollowMeControl_Response * value =
    (const autonomy_interfaces__srv__FollowMeControl_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__resize_function__FollowMeControl_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence * member =
    (autonomy_interfaces__srv__FollowMeControl_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__FollowMeControl_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__FollowMeControl_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__FollowMeControl_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__size_function__FollowMeControl_Event__request,  // size() function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__fetch_function__FollowMeControl_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__assign_function__FollowMeControl_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__resize_function__FollowMeControl_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__srv__FollowMeControl_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__size_function__FollowMeControl_Event__response,  // size() function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_const_function__FollowMeControl_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__get_function__FollowMeControl_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__fetch_function__FollowMeControl_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__assign_function__FollowMeControl_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__resize_function__FollowMeControl_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "FollowMeControl_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__FollowMeControl_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_member_array,  // message members
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__FollowMeControl_Event__get_type_hash,
  &autonomy_interfaces__srv__FollowMeControl_Event__get_type_description,
  &autonomy_interfaces__srv__FollowMeControl_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Event)() {
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Request)();
  autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Response)();
  if (!autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/follow_me_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "FollowMeControl",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__FollowMeControl_Request__rosidl_typesupport_introspection_c__FollowMeControl_Request_message_type_support_handle,
  &autonomy_interfaces__srv__FollowMeControl_Response__rosidl_typesupport_introspection_c__FollowMeControl_Response_message_type_support_handle,
  &autonomy_interfaces__srv__FollowMeControl_Event__rosidl_typesupport_introspection_c__FollowMeControl_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    FollowMeControl
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    FollowMeControl
  ),
  &autonomy_interfaces__srv__FollowMeControl__get_type_hash,
  &autonomy_interfaces__srv__FollowMeControl__get_type_description,
  &autonomy_interfaces__srv__FollowMeControl__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl)(void) {
  if (!autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, FollowMeControl_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__follow_me_control__rosidl_typesupport_introspection_c__FollowMeControl_service_type_support_handle;
}
