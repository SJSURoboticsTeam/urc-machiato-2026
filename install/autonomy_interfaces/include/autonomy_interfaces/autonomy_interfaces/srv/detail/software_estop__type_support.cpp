// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/srv/detail/software_estop__functions.h"
#include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace autonomy_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SoftwareEstop_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::SoftwareEstop_Request(_init);
}

void SoftwareEstop_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::SoftwareEstop_Request *>(message_memory);
  typed_message->~SoftwareEstop_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SoftwareEstop_Request_message_member_array[4] = {
  {
    "operator_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Request, operator_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reason",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Request, reason),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acknowledge_criticality",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Request, acknowledge_criticality),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "force_immediate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Request, force_immediate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SoftwareEstop_Request_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "SoftwareEstop_Request",  // message name
  4,  // number of fields
  sizeof(autonomy_interfaces::srv::SoftwareEstop_Request),
  false,  // has_any_key_member_
  SoftwareEstop_Request_message_member_array,  // message members
  SoftwareEstop_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SoftwareEstop_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SoftwareEstop_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SoftwareEstop_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__SoftwareEstop_Request__get_type_hash,
  &autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description,
  &autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Request>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, SoftwareEstop_Request)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace autonomy_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SoftwareEstop_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::SoftwareEstop_Response(_init);
}

void SoftwareEstop_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::SoftwareEstop_Response *>(message_memory);
  typed_message->~SoftwareEstop_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SoftwareEstop_Response_message_member_array[6] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "estop_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, estop_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "triggered_by",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, triggered_by),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "coordination_started",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Response, coordination_started),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SoftwareEstop_Response_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "SoftwareEstop_Response",  // message name
  6,  // number of fields
  sizeof(autonomy_interfaces::srv::SoftwareEstop_Response),
  false,  // has_any_key_member_
  SoftwareEstop_Response_message_member_array,  // message members
  SoftwareEstop_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SoftwareEstop_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SoftwareEstop_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SoftwareEstop_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__SoftwareEstop_Response__get_type_hash,
  &autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description,
  &autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Response>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, SoftwareEstop_Response)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace autonomy_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SoftwareEstop_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::SoftwareEstop_Event(_init);
}

void SoftwareEstop_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::SoftwareEstop_Event *>(message_memory);
  typed_message->~SoftwareEstop_Event();
}

size_t size_function__SoftwareEstop_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::SoftwareEstop_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SoftwareEstop_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::SoftwareEstop_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__SoftwareEstop_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::SoftwareEstop_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__SoftwareEstop_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::SoftwareEstop_Request *>(
    get_const_function__SoftwareEstop_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::SoftwareEstop_Request *>(untyped_value);
  value = item;
}

void assign_function__SoftwareEstop_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::SoftwareEstop_Request *>(
    get_function__SoftwareEstop_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::SoftwareEstop_Request *>(untyped_value);
  item = value;
}

void resize_function__SoftwareEstop_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::SoftwareEstop_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SoftwareEstop_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::SoftwareEstop_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SoftwareEstop_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::SoftwareEstop_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__SoftwareEstop_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::SoftwareEstop_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__SoftwareEstop_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::SoftwareEstop_Response *>(
    get_const_function__SoftwareEstop_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::SoftwareEstop_Response *>(untyped_value);
  value = item;
}

void assign_function__SoftwareEstop_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::SoftwareEstop_Response *>(
    get_function__SoftwareEstop_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::SoftwareEstop_Response *>(untyped_value);
  item = value;
}

void resize_function__SoftwareEstop_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::SoftwareEstop_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SoftwareEstop_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__SoftwareEstop_Event__request,  // size() function pointer
    get_const_function__SoftwareEstop_Event__request,  // get_const(index) function pointer
    get_function__SoftwareEstop_Event__request,  // get(index) function pointer
    fetch_function__SoftwareEstop_Event__request,  // fetch(index, &value) function pointer
    assign_function__SoftwareEstop_Event__request,  // assign(index, value) function pointer
    resize_function__SoftwareEstop_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::SoftwareEstop_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__SoftwareEstop_Event__response,  // size() function pointer
    get_const_function__SoftwareEstop_Event__response,  // get_const(index) function pointer
    get_function__SoftwareEstop_Event__response,  // get(index) function pointer
    fetch_function__SoftwareEstop_Event__response,  // fetch(index, &value) function pointer
    assign_function__SoftwareEstop_Event__response,  // assign(index, value) function pointer
    resize_function__SoftwareEstop_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SoftwareEstop_Event_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "SoftwareEstop_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces::srv::SoftwareEstop_Event),
  false,  // has_any_key_member_
  SoftwareEstop_Event_message_member_array,  // message members
  SoftwareEstop_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  SoftwareEstop_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SoftwareEstop_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SoftwareEstop_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__SoftwareEstop_Event__get_type_hash,
  &autonomy_interfaces__srv__SoftwareEstop_Event__get_type_description,
  &autonomy_interfaces__srv__SoftwareEstop_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Event>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, SoftwareEstop_Event)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace autonomy_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SoftwareEstop_service_members = {
  "autonomy_interfaces::srv",  // service namespace
  "SoftwareEstop",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<autonomy_interfaces::srv::SoftwareEstop>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t SoftwareEstop_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SoftwareEstop_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::SoftwareEstop_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<autonomy_interfaces::srv::SoftwareEstop>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<autonomy_interfaces::srv::SoftwareEstop>,
  &autonomy_interfaces__srv__SoftwareEstop__get_type_hash,
  &autonomy_interfaces__srv__SoftwareEstop__get_type_description,
  &autonomy_interfaces__srv__SoftwareEstop__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<autonomy_interfaces::srv::SoftwareEstop>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::SoftwareEstop_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::SoftwareEstop_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::SoftwareEstop_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::SoftwareEstop_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, SoftwareEstop)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<autonomy_interfaces::srv::SoftwareEstop>();
}

#ifdef __cplusplus
}
#endif
