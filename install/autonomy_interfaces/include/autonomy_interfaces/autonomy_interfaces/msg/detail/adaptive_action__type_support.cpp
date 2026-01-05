// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/msg/detail/adaptive_action__functions.h"
#include "autonomy_interfaces/msg/detail/adaptive_action__struct.hpp"
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

void AdaptiveAction_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::msg::AdaptiveAction(_init);
}

void AdaptiveAction_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::msg::AdaptiveAction *>(message_memory);
  typed_message->~AdaptiveAction();
}

size_t size_function__AdaptiveAction__parameters(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AdaptiveAction__parameters(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__AdaptiveAction__parameters(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__AdaptiveAction__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__AdaptiveAction__parameters(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__AdaptiveAction__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__AdaptiveAction__parameters(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__AdaptiveAction__parameters(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AdaptiveAction_message_member_array[7] = {
  {
    "action_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, action_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "parameters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, parameters),  // bytes offset in struct
    nullptr,  // default value
    size_function__AdaptiveAction__parameters,  // size() function pointer
    get_const_function__AdaptiveAction__parameters,  // get_const(index) function pointer
    get_function__AdaptiveAction__parameters,  // get(index) function pointer
    fetch_function__AdaptiveAction__parameters,  // fetch(index, &value) function pointer
    assign_function__AdaptiveAction__parameters,  // assign(index, value) function pointer
    resize_function__AdaptiveAction__parameters  // resize(index) function pointer
  },
  {
    "trigger_context",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::msg::ContextState>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, trigger_context),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "priority",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, priority),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "expected_duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, expected_duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "success_criteria",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, success_criteria),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::msg::AdaptiveAction, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AdaptiveAction_message_members = {
  "autonomy_interfaces::msg",  // message namespace
  "AdaptiveAction",  // message name
  7,  // number of fields
  sizeof(autonomy_interfaces::msg::AdaptiveAction),
  false,  // has_any_key_member_
  AdaptiveAction_message_member_array,  // message members
  AdaptiveAction_init_function,  // function to initialize message memory (memory has to be allocated)
  AdaptiveAction_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AdaptiveAction_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AdaptiveAction_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__AdaptiveAction__get_type_hash,
  &autonomy_interfaces__msg__AdaptiveAction__get_type_description,
  &autonomy_interfaces__msg__AdaptiveAction__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::msg::AdaptiveAction>()
{
  return &::autonomy_interfaces::msg::rosidl_typesupport_introspection_cpp::AdaptiveAction_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, msg, AdaptiveAction)() {
  return &::autonomy_interfaces::msg::rosidl_typesupport_introspection_cpp::AdaptiveAction_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
