// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/srv/detail/get_safety_status__functions.h"
#include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
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

void GetSafetyStatus_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::GetSafetyStatus_Request(_init);
}

void GetSafetyStatus_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::GetSafetyStatus_Request *>(message_memory);
  typed_message->~GetSafetyStatus_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetSafetyStatus_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetSafetyStatus_Request_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "GetSafetyStatus_Request",  // message name
  1,  // number of fields
  sizeof(autonomy_interfaces::srv::GetSafetyStatus_Request),
  false,  // has_any_key_member_
  GetSafetyStatus_Request_message_member_array,  // message members
  GetSafetyStatus_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetSafetyStatus_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetSafetyStatus_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetSafetyStatus_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_hash,
  &autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description,
  &autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Request>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, GetSafetyStatus_Request)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Request_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/get_safety_status__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
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

void GetSafetyStatus_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::GetSafetyStatus_Response(_init);
}

void GetSafetyStatus_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::GetSafetyStatus_Response *>(message_memory);
  typed_message->~GetSafetyStatus_Response();
}

size_t size_function__GetSafetyStatus_Response__active_alerts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::msg::SafetyAlert> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetSafetyStatus_Response__active_alerts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::msg::SafetyAlert> *>(untyped_member);
  return &member[index];
}

void * get_function__GetSafetyStatus_Response__active_alerts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::msg::SafetyAlert> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetSafetyStatus_Response__active_alerts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::msg::SafetyAlert *>(
    get_const_function__GetSafetyStatus_Response__active_alerts(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::msg::SafetyAlert *>(untyped_value);
  value = item;
}

void assign_function__GetSafetyStatus_Response__active_alerts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::msg::SafetyAlert *>(
    get_function__GetSafetyStatus_Response__active_alerts(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::msg::SafetyAlert *>(untyped_value);
  item = value;
}

void resize_function__GetSafetyStatus_Response__active_alerts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::msg::SafetyAlert> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetSafetyStatus_Response_message_member_array[3] = {
  {
    "overall_safety",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Response, overall_safety),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "active_alerts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::msg::SafetyAlert>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Response, active_alerts),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetSafetyStatus_Response__active_alerts,  // size() function pointer
    get_const_function__GetSafetyStatus_Response__active_alerts,  // get_const(index) function pointer
    get_function__GetSafetyStatus_Response__active_alerts,  // get(index) function pointer
    fetch_function__GetSafetyStatus_Response__active_alerts,  // fetch(index, &value) function pointer
    assign_function__GetSafetyStatus_Response__active_alerts,  // assign(index, value) function pointer
    resize_function__GetSafetyStatus_Response__active_alerts  // resize(index) function pointer
  },
  {
    "monitoring_stats",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::msg::MonitoringStats>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Response, monitoring_stats),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetSafetyStatus_Response_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "GetSafetyStatus_Response",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces::srv::GetSafetyStatus_Response),
  false,  // has_any_key_member_
  GetSafetyStatus_Response_message_member_array,  // message members
  GetSafetyStatus_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetSafetyStatus_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetSafetyStatus_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetSafetyStatus_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_hash,
  &autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description,
  &autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Response>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, GetSafetyStatus_Response)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Response_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/get_safety_status__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
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

void GetSafetyStatus_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::GetSafetyStatus_Event(_init);
}

void GetSafetyStatus_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::GetSafetyStatus_Event *>(message_memory);
  typed_message->~GetSafetyStatus_Event();
}

size_t size_function__GetSafetyStatus_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::GetSafetyStatus_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetSafetyStatus_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::GetSafetyStatus_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__GetSafetyStatus_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::GetSafetyStatus_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetSafetyStatus_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::GetSafetyStatus_Request *>(
    get_const_function__GetSafetyStatus_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::GetSafetyStatus_Request *>(untyped_value);
  value = item;
}

void assign_function__GetSafetyStatus_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::GetSafetyStatus_Request *>(
    get_function__GetSafetyStatus_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::GetSafetyStatus_Request *>(untyped_value);
  item = value;
}

void resize_function__GetSafetyStatus_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::GetSafetyStatus_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetSafetyStatus_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::GetSafetyStatus_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetSafetyStatus_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::GetSafetyStatus_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__GetSafetyStatus_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::GetSafetyStatus_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetSafetyStatus_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::GetSafetyStatus_Response *>(
    get_const_function__GetSafetyStatus_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::GetSafetyStatus_Response *>(untyped_value);
  value = item;
}

void assign_function__GetSafetyStatus_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::GetSafetyStatus_Response *>(
    get_function__GetSafetyStatus_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::GetSafetyStatus_Response *>(untyped_value);
  item = value;
}

void resize_function__GetSafetyStatus_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::GetSafetyStatus_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetSafetyStatus_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Event, info),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetSafetyStatus_Event__request,  // size() function pointer
    get_const_function__GetSafetyStatus_Event__request,  // get_const(index) function pointer
    get_function__GetSafetyStatus_Event__request,  // get(index) function pointer
    fetch_function__GetSafetyStatus_Event__request,  // fetch(index, &value) function pointer
    assign_function__GetSafetyStatus_Event__request,  // assign(index, value) function pointer
    resize_function__GetSafetyStatus_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::GetSafetyStatus_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetSafetyStatus_Event__response,  // size() function pointer
    get_const_function__GetSafetyStatus_Event__response,  // get_const(index) function pointer
    get_function__GetSafetyStatus_Event__response,  // get(index) function pointer
    fetch_function__GetSafetyStatus_Event__response,  // fetch(index, &value) function pointer
    assign_function__GetSafetyStatus_Event__response,  // assign(index, value) function pointer
    resize_function__GetSafetyStatus_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetSafetyStatus_Event_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "GetSafetyStatus_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces::srv::GetSafetyStatus_Event),
  false,  // has_any_key_member_
  GetSafetyStatus_Event_message_member_array,  // message members
  GetSafetyStatus_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  GetSafetyStatus_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetSafetyStatus_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetSafetyStatus_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_hash,
  &autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_description,
  &autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Event>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, GetSafetyStatus_Event)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_Event_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/get_safety_status__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetSafetyStatus_service_members = {
  "autonomy_interfaces::srv",  // service namespace
  "GetSafetyStatus",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t GetSafetyStatus_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetSafetyStatus_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<autonomy_interfaces::srv::GetSafetyStatus>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<autonomy_interfaces::srv::GetSafetyStatus>,
  &autonomy_interfaces__srv__GetSafetyStatus__get_type_hash,
  &autonomy_interfaces__srv__GetSafetyStatus__get_type_description,
  &autonomy_interfaces__srv__GetSafetyStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::GetSafetyStatus_service_type_support_handle;
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
        ::autonomy_interfaces::srv::GetSafetyStatus_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::GetSafetyStatus_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::GetSafetyStatus_Event
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, GetSafetyStatus)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<autonomy_interfaces::srv::GetSafetyStatus>();
}

#ifdef __cplusplus
}
#endif
