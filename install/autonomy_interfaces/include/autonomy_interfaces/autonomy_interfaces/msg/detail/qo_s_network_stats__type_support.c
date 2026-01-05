// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__functions.h"
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__struct.h"


// Include directives for member types
// Member `current_band`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__QoSNetworkStats__init(message_memory);
}

void autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__QoSNetworkStats__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_member_array[6] = {
  {
    "bandwidth_up_mbps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, bandwidth_up_mbps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bandwidth_down_mbps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, bandwidth_down_mbps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "latency_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, latency_ms),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, packet_loss_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_band",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, current_band),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "signal_strength",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__QoSNetworkStats, signal_strength),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "QoSNetworkStats",  // message name
  6,  // number of fields
  sizeof(autonomy_interfaces__msg__QoSNetworkStats),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_member_array,  // message members
  autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__QoSNetworkStats__get_type_hash,
  &autonomy_interfaces__msg__QoSNetworkStats__get_type_description,
  &autonomy_interfaces__msg__QoSNetworkStats__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, QoSNetworkStats)() {
  if (!autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__QoSNetworkStats__rosidl_typesupport_introspection_c__QoSNetworkStats_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
