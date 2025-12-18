// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/srv/detail/recover_from_safety__struct.hpp"
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

void RecoverFromSafety_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::RecoverFromSafety_Request(_init);
}

void RecoverFromSafety_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::RecoverFromSafety_Request *>(message_memory);
  typed_message->~RecoverFromSafety_Request();
}

size_t size_function__RecoverFromSafety_Request__completed_steps(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoverFromSafety_Request__completed_steps(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoverFromSafety_Request__completed_steps(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoverFromSafety_Request__completed_steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RecoverFromSafety_Request__completed_steps(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RecoverFromSafety_Request__completed_steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RecoverFromSafety_Request__completed_steps(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RecoverFromSafety_Request__completed_steps(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RecoverFromSafety_Request_message_member_array[5] = {
  {
    "recovery_method",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Request, recovery_method),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "operator_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Request, operator_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acknowledge_risks",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Request, acknowledge_risks),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "completed_steps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Request, completed_steps),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoverFromSafety_Request__completed_steps,  // size() function pointer
    get_const_function__RecoverFromSafety_Request__completed_steps,  // get_const(index) function pointer
    get_function__RecoverFromSafety_Request__completed_steps,  // get(index) function pointer
    fetch_function__RecoverFromSafety_Request__completed_steps,  // fetch(index, &value) function pointer
    assign_function__RecoverFromSafety_Request__completed_steps,  // assign(index, value) function pointer
    resize_function__RecoverFromSafety_Request__completed_steps  // resize(index) function pointer
  },
  {
    "notes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Request, notes),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RecoverFromSafety_Request_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "RecoverFromSafety_Request",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces::srv::RecoverFromSafety_Request),
  RecoverFromSafety_Request_message_member_array,  // message members
  RecoverFromSafety_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  RecoverFromSafety_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RecoverFromSafety_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RecoverFromSafety_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::RecoverFromSafety_Request>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::RecoverFromSafety_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, RecoverFromSafety_Request)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::RecoverFromSafety_Request_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/recover_from_safety__struct.hpp"
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

void RecoverFromSafety_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::RecoverFromSafety_Response(_init);
}

void RecoverFromSafety_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::RecoverFromSafety_Response *>(message_memory);
  typed_message->~RecoverFromSafety_Response();
}

size_t size_function__RecoverFromSafety_Response__remaining_steps(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoverFromSafety_Response__remaining_steps(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoverFromSafety_Response__remaining_steps(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoverFromSafety_Response__remaining_steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RecoverFromSafety_Response__remaining_steps(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RecoverFromSafety_Response__remaining_steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RecoverFromSafety_Response__remaining_steps(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RecoverFromSafety_Response__remaining_steps(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RecoverFromSafety_Response__verified_systems(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoverFromSafety_Response__verified_systems(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoverFromSafety_Response__verified_systems(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoverFromSafety_Response__verified_systems(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RecoverFromSafety_Response__verified_systems(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RecoverFromSafety_Response__verified_systems(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RecoverFromSafety_Response__verified_systems(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RecoverFromSafety_Response__verified_systems(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RecoverFromSafety_Response__failed_systems(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoverFromSafety_Response__failed_systems(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoverFromSafety_Response__failed_systems(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoverFromSafety_Response__failed_systems(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RecoverFromSafety_Response__failed_systems(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RecoverFromSafety_Response__failed_systems(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RecoverFromSafety_Response__failed_systems(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RecoverFromSafety_Response__failed_systems(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RecoverFromSafety_Response__restrictions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoverFromSafety_Response__restrictions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoverFromSafety_Response__restrictions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoverFromSafety_Response__restrictions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RecoverFromSafety_Response__restrictions(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RecoverFromSafety_Response__restrictions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RecoverFromSafety_Response__restrictions(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RecoverFromSafety_Response__restrictions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RecoverFromSafety_Response_message_member_array[10] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, success),  // bytes offset in struct
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
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "recovery_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, recovery_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_safe_to_proceed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, is_safe_to_proceed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "remaining_steps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, remaining_steps),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoverFromSafety_Response__remaining_steps,  // size() function pointer
    get_const_function__RecoverFromSafety_Response__remaining_steps,  // get_const(index) function pointer
    get_function__RecoverFromSafety_Response__remaining_steps,  // get(index) function pointer
    fetch_function__RecoverFromSafety_Response__remaining_steps,  // fetch(index, &value) function pointer
    assign_function__RecoverFromSafety_Response__remaining_steps,  // assign(index, value) function pointer
    resize_function__RecoverFromSafety_Response__remaining_steps  // resize(index) function pointer
  },
  {
    "verified_systems",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, verified_systems),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoverFromSafety_Response__verified_systems,  // size() function pointer
    get_const_function__RecoverFromSafety_Response__verified_systems,  // get_const(index) function pointer
    get_function__RecoverFromSafety_Response__verified_systems,  // get(index) function pointer
    fetch_function__RecoverFromSafety_Response__verified_systems,  // fetch(index, &value) function pointer
    assign_function__RecoverFromSafety_Response__verified_systems,  // assign(index, value) function pointer
    resize_function__RecoverFromSafety_Response__verified_systems  // resize(index) function pointer
  },
  {
    "failed_systems",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, failed_systems),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoverFromSafety_Response__failed_systems,  // size() function pointer
    get_const_function__RecoverFromSafety_Response__failed_systems,  // get_const(index) function pointer
    get_function__RecoverFromSafety_Response__failed_systems,  // get(index) function pointer
    fetch_function__RecoverFromSafety_Response__failed_systems,  // fetch(index, &value) function pointer
    assign_function__RecoverFromSafety_Response__failed_systems,  // assign(index, value) function pointer
    resize_function__RecoverFromSafety_Response__failed_systems  // resize(index) function pointer
  },
  {
    "estimated_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, estimated_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "recommended_next_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, recommended_next_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "restrictions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::RecoverFromSafety_Response, restrictions),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoverFromSafety_Response__restrictions,  // size() function pointer
    get_const_function__RecoverFromSafety_Response__restrictions,  // get_const(index) function pointer
    get_function__RecoverFromSafety_Response__restrictions,  // get(index) function pointer
    fetch_function__RecoverFromSafety_Response__restrictions,  // fetch(index, &value) function pointer
    assign_function__RecoverFromSafety_Response__restrictions,  // assign(index, value) function pointer
    resize_function__RecoverFromSafety_Response__restrictions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RecoverFromSafety_Response_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "RecoverFromSafety_Response",  // message name
  10,  // number of fields
  sizeof(autonomy_interfaces::srv::RecoverFromSafety_Response),
  RecoverFromSafety_Response_message_member_array,  // message members
  RecoverFromSafety_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  RecoverFromSafety_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RecoverFromSafety_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RecoverFromSafety_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::RecoverFromSafety_Response>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::RecoverFromSafety_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, RecoverFromSafety_Response)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::RecoverFromSafety_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/recover_from_safety__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers RecoverFromSafety_service_members = {
  "autonomy_interfaces::srv",  // service namespace
  "RecoverFromSafety",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<autonomy_interfaces::srv::RecoverFromSafety>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t RecoverFromSafety_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RecoverFromSafety_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<autonomy_interfaces::srv::RecoverFromSafety>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::RecoverFromSafety_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::RecoverFromSafety_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::RecoverFromSafety_Response
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, RecoverFromSafety)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<autonomy_interfaces::srv::RecoverFromSafety>();
}

#ifdef __cplusplus
}
#endif
