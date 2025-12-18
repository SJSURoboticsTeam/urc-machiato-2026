// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/FollowMeStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__FollowMeStatus __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__FollowMeStatus __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowMeStatus_
{
  using Type = FollowMeStatus_<ContainerAllocator>;

  explicit FollowMeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    target_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_following = false;
      this->target_tag_id = 0l;
      this->target_distance = 0.0f;
      this->target_angle = 0.0f;
      this->safety_distance = 0.0f;
      this->safety_violation = false;
      this->current_speed = 0.0f;
      this->target_visible = false;
      this->last_detection_time = 0.0f;
      this->max_speed = 0.0f;
      this->operator_id = "";
    }
  }

  explicit FollowMeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    target_position(_alloc, _init),
    operator_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_following = false;
      this->target_tag_id = 0l;
      this->target_distance = 0.0f;
      this->target_angle = 0.0f;
      this->safety_distance = 0.0f;
      this->safety_violation = false;
      this->current_speed = 0.0f;
      this->target_visible = false;
      this->last_detection_time = 0.0f;
      this->max_speed = 0.0f;
      this->operator_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_following_type =
    bool;
  _is_following_type is_following;
  using _target_tag_id_type =
    int32_t;
  _target_tag_id_type target_tag_id;
  using _target_distance_type =
    float;
  _target_distance_type target_distance;
  using _target_angle_type =
    float;
  _target_angle_type target_angle;
  using _safety_distance_type =
    float;
  _safety_distance_type safety_distance;
  using _safety_violation_type =
    bool;
  _safety_violation_type safety_violation;
  using _current_speed_type =
    float;
  _current_speed_type current_speed;
  using _target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_position_type target_position;
  using _target_visible_type =
    bool;
  _target_visible_type target_visible;
  using _last_detection_time_type =
    float;
  _last_detection_time_type last_detection_time;
  using _max_speed_type =
    float;
  _max_speed_type max_speed;
  using _operator_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operator_id_type operator_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_following(
    const bool & _arg)
  {
    this->is_following = _arg;
    return *this;
  }
  Type & set__target_tag_id(
    const int32_t & _arg)
  {
    this->target_tag_id = _arg;
    return *this;
  }
  Type & set__target_distance(
    const float & _arg)
  {
    this->target_distance = _arg;
    return *this;
  }
  Type & set__target_angle(
    const float & _arg)
  {
    this->target_angle = _arg;
    return *this;
  }
  Type & set__safety_distance(
    const float & _arg)
  {
    this->safety_distance = _arg;
    return *this;
  }
  Type & set__safety_violation(
    const bool & _arg)
  {
    this->safety_violation = _arg;
    return *this;
  }
  Type & set__current_speed(
    const float & _arg)
  {
    this->current_speed = _arg;
    return *this;
  }
  Type & set__target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__target_visible(
    const bool & _arg)
  {
    this->target_visible = _arg;
    return *this;
  }
  Type & set__last_detection_time(
    const float & _arg)
  {
    this->last_detection_time = _arg;
    return *this;
  }
  Type & set__max_speed(
    const float & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__operator_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operator_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__FollowMeStatus
    std::shared_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__FollowMeStatus
    std::shared_ptr<autonomy_interfaces::msg::FollowMeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowMeStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_following != other.is_following) {
      return false;
    }
    if (this->target_tag_id != other.target_tag_id) {
      return false;
    }
    if (this->target_distance != other.target_distance) {
      return false;
    }
    if (this->target_angle != other.target_angle) {
      return false;
    }
    if (this->safety_distance != other.safety_distance) {
      return false;
    }
    if (this->safety_violation != other.safety_violation) {
      return false;
    }
    if (this->current_speed != other.current_speed) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->target_visible != other.target_visible) {
      return false;
    }
    if (this->last_detection_time != other.last_detection_time) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->operator_id != other.operator_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowMeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowMeStatus_

// alias to use template instance with default allocator
using FollowMeStatus =
  autonomy_interfaces::msg::FollowMeStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__STRUCT_HPP_
