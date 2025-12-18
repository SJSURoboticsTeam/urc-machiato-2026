// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_HPP_

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
// Member 'keyboard_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'key_dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__TypingGoal __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__TypingGoal __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TypingGoal_
{
  using Type = TypingGoal_<ContainerAllocator>;

  explicit TypingGoal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    keyboard_pose(_init),
    key_dimensions(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_text = "";
      this->accuracy_requirement = 0.0f;
      this->speed_requirement = 0.0f;
      this->timeout = 0.0f;
      this->keyboard_layout = "";
      this->key_spacing_m = 0.0f;
      this->standoff_distance = 0.0f;
      this->contact_force = 0.0f;
    }
  }

  explicit TypingGoal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    target_text(_alloc),
    keyboard_pose(_alloc, _init),
    keyboard_layout(_alloc),
    key_dimensions(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_text = "";
      this->accuracy_requirement = 0.0f;
      this->speed_requirement = 0.0f;
      this->timeout = 0.0f;
      this->keyboard_layout = "";
      this->key_spacing_m = 0.0f;
      this->standoff_distance = 0.0f;
      this->contact_force = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _target_text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_text_type target_text;
  using _keyboard_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _keyboard_pose_type keyboard_pose;
  using _accuracy_requirement_type =
    float;
  _accuracy_requirement_type accuracy_requirement;
  using _speed_requirement_type =
    float;
  _speed_requirement_type speed_requirement;
  using _timeout_type =
    float;
  _timeout_type timeout;
  using _keyboard_layout_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _keyboard_layout_type keyboard_layout;
  using _key_spacing_m_type =
    float;
  _key_spacing_m_type key_spacing_m;
  using _key_dimensions_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _key_dimensions_type key_dimensions;
  using _standoff_distance_type =
    float;
  _standoff_distance_type standoff_distance;
  using _contact_force_type =
    float;
  _contact_force_type contact_force;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__target_text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_text = _arg;
    return *this;
  }
  Type & set__keyboard_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->keyboard_pose = _arg;
    return *this;
  }
  Type & set__accuracy_requirement(
    const float & _arg)
  {
    this->accuracy_requirement = _arg;
    return *this;
  }
  Type & set__speed_requirement(
    const float & _arg)
  {
    this->speed_requirement = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__keyboard_layout(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->keyboard_layout = _arg;
    return *this;
  }
  Type & set__key_spacing_m(
    const float & _arg)
  {
    this->key_spacing_m = _arg;
    return *this;
  }
  Type & set__key_dimensions(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->key_dimensions = _arg;
    return *this;
  }
  Type & set__standoff_distance(
    const float & _arg)
  {
    this->standoff_distance = _arg;
    return *this;
  }
  Type & set__contact_force(
    const float & _arg)
  {
    this->contact_force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__TypingGoal
    std::shared_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__TypingGoal
    std::shared_ptr<autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TypingGoal_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->target_text != other.target_text) {
      return false;
    }
    if (this->keyboard_pose != other.keyboard_pose) {
      return false;
    }
    if (this->accuracy_requirement != other.accuracy_requirement) {
      return false;
    }
    if (this->speed_requirement != other.speed_requirement) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->keyboard_layout != other.keyboard_layout) {
      return false;
    }
    if (this->key_spacing_m != other.key_spacing_m) {
      return false;
    }
    if (this->key_dimensions != other.key_dimensions) {
      return false;
    }
    if (this->standoff_distance != other.standoff_distance) {
      return false;
    }
    if (this->contact_force != other.contact_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const TypingGoal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TypingGoal_

// alias to use template instance with default allocator
using TypingGoal =
  autonomy_interfaces::msg::TypingGoal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__STRUCT_HPP_
