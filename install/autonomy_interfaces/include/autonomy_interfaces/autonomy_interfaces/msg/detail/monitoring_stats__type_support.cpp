// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/msg/detail/monitoring_stats__functions.h"
#include "autonomy_interfaces/msg/detail/monitoring_stats__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace autonomy_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MonitoringStats_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::msg::MonitoringStats(_init);
}

void MonitoringStats_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::msg::MonitoringStats *>(message_memory);
  typed_message->~MonitoringStats();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MonitoringStats_message_member_array[3] = {
  {
    "total_evaluations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::MonitoringStats, total_evaluations),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "total_violations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::MonitoringStats, total_violations),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "evaluation_rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::MonitoringStats, evaluation_rate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MonitoringStats_message_members = {
  "autonomy_interfaces::msg",  // message namespace
  "MonitoringStats",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces::msg::MonitoringStats),
  false,  // has_any_key_member_
  MonitoringStats_message_member_array,  // message members
  MonitoringStats_init_function,  // function to initialize message memory (memory has to be allocated)
  MonitoringStats_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MonitoringStats_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MonitoringStats_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__MonitoringStats__get_type_hash,
  &autonomy_interfaces__msg__MonitoringStats__get_type_description,
  &autonomy_interfaces__msg__MonitoringStats__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::msg::MonitoringStats>()
{
  return &::autonomy_interfaces::msg::rosidl_typesupport_introspection_cpp::MonitoringStats_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, msg, MonitoringStats)() {
  return &::autonomy_interfaces::msg::rosidl_typesupport_introspection_cpp::MonitoringStats_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
