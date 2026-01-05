// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"
#include "autonomy_interfaces/srv/detail/detect_mission_aruco__struct.hpp"
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

void DetectMissionAruco_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::DetectMissionAruco_Request(_init);
}

void DetectMissionAruco_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::DetectMissionAruco_Request *>(message_memory);
  typed_message->~DetectMissionAruco_Request();
}

size_t size_function__DetectMissionAruco_Request__required_tag_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Request__required_tag_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Request__required_tag_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Request__required_tag_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__DetectMissionAruco_Request__required_tag_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Request__required_tag_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__DetectMissionAruco_Request__required_tag_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Request__required_tag_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Request__optional_tag_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Request__optional_tag_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Request__optional_tag_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Request__optional_tag_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__DetectMissionAruco_Request__optional_tag_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Request__optional_tag_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__DetectMissionAruco_Request__optional_tag_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Request__optional_tag_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectMissionAruco_Request_message_member_array[8] = {
  {
    "mission_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, mission_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "required_tag_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, required_tag_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Request__required_tag_ids,  // size() function pointer
    get_const_function__DetectMissionAruco_Request__required_tag_ids,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Request__required_tag_ids,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Request__required_tag_ids,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Request__required_tag_ids,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Request__required_tag_ids  // resize(index) function pointer
  },
  {
    "optional_tag_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, optional_tag_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Request__optional_tag_ids,  // size() function pointer
    get_const_function__DetectMissionAruco_Request__optional_tag_ids,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Request__optional_tag_ids,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Request__optional_tag_ids,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Request__optional_tag_ids,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Request__optional_tag_ids  // resize(index) function pointer
  },
  {
    "detection_timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, detection_timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "target_depth",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, target_depth),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_detection_distance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, max_detection_distance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "require_all_tags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, require_all_tags),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "min_alignment_quality",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Request, min_alignment_quality),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectMissionAruco_Request_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "DetectMissionAruco_Request",  // message name
  8,  // number of fields
  sizeof(autonomy_interfaces::srv::DetectMissionAruco_Request),
  false,  // has_any_key_member_
  DetectMissionAruco_Request_message_member_array,  // message members
  DetectMissionAruco_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectMissionAruco_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectMissionAruco_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectMissionAruco_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_hash,
  &autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description,
  &autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Request>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, DetectMissionAruco_Request)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Request_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__struct.hpp"
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

void DetectMissionAruco_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::DetectMissionAruco_Response(_init);
}

void DetectMissionAruco_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::DetectMissionAruco_Response *>(message_memory);
  typed_message->~DetectMissionAruco_Response();
}

size_t size_function__DetectMissionAruco_Response__detected_tag_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__detected_tag_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__detected_tag_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__detected_tag_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__DetectMissionAruco_Response__detected_tag_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__detected_tag_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__DetectMissionAruco_Response__detected_tag_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__detected_tag_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__tag_positions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__tag_positions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__tag_positions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__tag_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__DetectMissionAruco_Response__tag_positions(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__tag_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__DetectMissionAruco_Response__tag_positions(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__tag_positions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__tag_distances(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__tag_distances(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__tag_distances(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__tag_distances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__DetectMissionAruco_Response__tag_distances(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__tag_distances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__DetectMissionAruco_Response__tag_distances(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__tag_distances(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__tag_angles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__tag_angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__tag_angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__tag_angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__DetectMissionAruco_Response__tag_angles(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__tag_angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__DetectMissionAruco_Response__tag_angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__tag_angles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__alignment_errors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__alignment_errors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__alignment_errors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__alignment_errors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__DetectMissionAruco_Response__alignment_errors(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__alignment_errors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__DetectMissionAruco_Response__alignment_errors(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__alignment_errors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__missing_required_tags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__missing_required_tags(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__missing_required_tags(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__missing_required_tags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__DetectMissionAruco_Response__missing_required_tags(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__missing_required_tags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__DetectMissionAruco_Response__missing_required_tags(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__missing_required_tags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__detected_optional_tags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__detected_optional_tags(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__detected_optional_tags(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__detected_optional_tags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__DetectMissionAruco_Response__detected_optional_tags(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__detected_optional_tags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__DetectMissionAruco_Response__detected_optional_tags(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__detected_optional_tags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__alignment_warnings(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__alignment_warnings(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__alignment_warnings(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__alignment_warnings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__DetectMissionAruco_Response__alignment_warnings(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__alignment_warnings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__DetectMissionAruco_Response__alignment_warnings(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__alignment_warnings(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Response__mission_recommendations(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Response__mission_recommendations(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Response__mission_recommendations(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Response__mission_recommendations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__DetectMissionAruco_Response__mission_recommendations(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Response__mission_recommendations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__DetectMissionAruco_Response__mission_recommendations(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Response__mission_recommendations(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectMissionAruco_Response_message_member_array[19] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, success),  // bytes offset in struct
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
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mission_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, mission_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "detected_tag_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, detected_tag_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__detected_tag_ids,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__detected_tag_ids,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__detected_tag_ids,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__detected_tag_ids,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__detected_tag_ids,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__detected_tag_ids  // resize(index) function pointer
  },
  {
    "tag_positions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, tag_positions),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__tag_positions,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__tag_positions,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__tag_positions,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__tag_positions,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__tag_positions,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__tag_positions  // resize(index) function pointer
  },
  {
    "tag_distances",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, tag_distances),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__tag_distances,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__tag_distances,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__tag_distances,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__tag_distances,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__tag_distances,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__tag_distances  // resize(index) function pointer
  },
  {
    "tag_angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, tag_angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__tag_angles,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__tag_angles,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__tag_angles,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__tag_angles,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__tag_angles,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__tag_angles  // resize(index) function pointer
  },
  {
    "detection_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, detection_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_available",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_available),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_center",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_center),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_orientation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Quaternion>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_orientation),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "arm_target_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, arm_target_position),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_quality",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_quality),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_errors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_errors),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__alignment_errors,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__alignment_errors,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__alignment_errors,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__alignment_errors,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__alignment_errors,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__alignment_errors  // resize(index) function pointer
  },
  {
    "mission_ready",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, mission_ready),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "missing_required_tags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, missing_required_tags),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__missing_required_tags,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__missing_required_tags,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__missing_required_tags,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__missing_required_tags,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__missing_required_tags,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__missing_required_tags  // resize(index) function pointer
  },
  {
    "detected_optional_tags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, detected_optional_tags),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__detected_optional_tags,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__detected_optional_tags,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__detected_optional_tags,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__detected_optional_tags,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__detected_optional_tags,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__detected_optional_tags  // resize(index) function pointer
  },
  {
    "alignment_warnings",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, alignment_warnings),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__alignment_warnings,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__alignment_warnings,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__alignment_warnings,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__alignment_warnings,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__alignment_warnings,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__alignment_warnings  // resize(index) function pointer
  },
  {
    "mission_recommendations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Response, mission_recommendations),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Response__mission_recommendations,  // size() function pointer
    get_const_function__DetectMissionAruco_Response__mission_recommendations,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Response__mission_recommendations,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Response__mission_recommendations,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Response__mission_recommendations,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Response__mission_recommendations  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectMissionAruco_Response_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "DetectMissionAruco_Response",  // message name
  19,  // number of fields
  sizeof(autonomy_interfaces::srv::DetectMissionAruco_Response),
  false,  // has_any_key_member_
  DetectMissionAruco_Response_message_member_array,  // message members
  DetectMissionAruco_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectMissionAruco_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectMissionAruco_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectMissionAruco_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_hash,
  &autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description,
  &autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Response>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, DetectMissionAruco_Response)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Response_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__struct.hpp"
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

void DetectMissionAruco_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autonomy_interfaces::srv::DetectMissionAruco_Event(_init);
}

void DetectMissionAruco_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autonomy_interfaces::srv::DetectMissionAruco_Event *>(message_memory);
  typed_message->~DetectMissionAruco_Event();
}

size_t size_function__DetectMissionAruco_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::DetectMissionAruco_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::DetectMissionAruco_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::DetectMissionAruco_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::DetectMissionAruco_Request *>(
    get_const_function__DetectMissionAruco_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::DetectMissionAruco_Request *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::DetectMissionAruco_Request *>(
    get_function__DetectMissionAruco_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::DetectMissionAruco_Request *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::DetectMissionAruco_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectMissionAruco_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autonomy_interfaces::srv::DetectMissionAruco_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectMissionAruco_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autonomy_interfaces::srv::DetectMissionAruco_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectMissionAruco_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autonomy_interfaces::srv::DetectMissionAruco_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectMissionAruco_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autonomy_interfaces::srv::DetectMissionAruco_Response *>(
    get_const_function__DetectMissionAruco_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<autonomy_interfaces::srv::DetectMissionAruco_Response *>(untyped_value);
  value = item;
}

void assign_function__DetectMissionAruco_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autonomy_interfaces::srv::DetectMissionAruco_Response *>(
    get_function__DetectMissionAruco_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const autonomy_interfaces::srv::DetectMissionAruco_Response *>(untyped_value);
  item = value;
}

void resize_function__DetectMissionAruco_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autonomy_interfaces::srv::DetectMissionAruco_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectMissionAruco_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Event, info),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Event__request,  // size() function pointer
    get_const_function__DetectMissionAruco_Event__request,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Event__request,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Event__request,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Event__request,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces::srv::DetectMissionAruco_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectMissionAruco_Event__response,  // size() function pointer
    get_const_function__DetectMissionAruco_Event__response,  // get_const(index) function pointer
    get_function__DetectMissionAruco_Event__response,  // get(index) function pointer
    fetch_function__DetectMissionAruco_Event__response,  // fetch(index, &value) function pointer
    assign_function__DetectMissionAruco_Event__response,  // assign(index, value) function pointer
    resize_function__DetectMissionAruco_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectMissionAruco_Event_message_members = {
  "autonomy_interfaces::srv",  // message namespace
  "DetectMissionAruco_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces::srv::DetectMissionAruco_Event),
  false,  // has_any_key_member_
  DetectMissionAruco_Event_message_member_array,  // message members
  DetectMissionAruco_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectMissionAruco_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectMissionAruco_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectMissionAruco_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_hash,
  &autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_description,
  &autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Event>()
{
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, DetectMissionAruco_Event)() {
  return &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_Event_message_type_support_handle;
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
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers DetectMissionAruco_service_members = {
  "autonomy_interfaces::srv",  // service namespace
  "DetectMissionAruco",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t DetectMissionAruco_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectMissionAruco_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<autonomy_interfaces::srv::DetectMissionAruco>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<autonomy_interfaces::srv::DetectMissionAruco>,
  &autonomy_interfaces__srv__DetectMissionAruco__get_type_hash,
  &autonomy_interfaces__srv__DetectMissionAruco__get_type_description,
  &autonomy_interfaces__srv__DetectMissionAruco__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace autonomy_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::autonomy_interfaces::srv::rosidl_typesupport_introspection_cpp::DetectMissionAruco_service_type_support_handle;
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
        ::autonomy_interfaces::srv::DetectMissionAruco_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::DetectMissionAruco_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::autonomy_interfaces::srv::DetectMissionAruco_Event
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autonomy_interfaces, srv, DetectMissionAruco)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<autonomy_interfaces::srv::DetectMissionAruco>();
}

#ifdef __cplusplus
}
#endif
