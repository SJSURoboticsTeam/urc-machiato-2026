// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__functions.h"
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__struct.h"


// Include directives for member types
// Member `topic_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__QoSTopicProfile__init(message_memory);
}

void autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__QoSTopicProfile__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_member_array[5] = {
  {
    "topic_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSTopicProfile, topic_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "avg_latency_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSTopicProfile, avg_latency_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "jitter_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSTopicProfile, jitter_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "packet_loss_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSTopicProfile, packet_loss_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "samples_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSTopicProfile, samples_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "QoSTopicProfile",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces__msg__QoSTopicProfile),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_member_array,  // message members
  autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__QoSTopicProfile__get_type_hash,
  &autonomy_interfaces__msg__QoSTopicProfile__get_type_description,
  &autonomy_interfaces__msg__QoSTopicProfile__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, QoSTopicProfile)() {
  if (!autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__QoSTopicProfile__rosidl_typesupport_introspection_c__QoSTopicProfile_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
