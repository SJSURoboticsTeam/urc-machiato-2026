// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/LedCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/led_command.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__LedCommand __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__LedCommand __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LedCommand_
{
  using Type = LedCommand_<ContainerAllocator>;

  explicit LedCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_code = 0l;
      this->red = 0.0f;
      this->green = 0.0f;
      this->blue = 0.0f;
      this->pattern = "";
      this->frequency = 0.0f;
      this->priority = 0l;
      this->duration = 0.0f;
      this->override = false;
    }
  }

  explicit LedCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pattern(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_code = 0l;
      this->red = 0.0f;
      this->green = 0.0f;
      this->blue = 0.0f;
      this->pattern = "";
      this->frequency = 0.0f;
      this->priority = 0l;
      this->duration = 0.0f;
      this->override = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_code_type =
    int32_t;
  _status_code_type status_code;
  using _red_type =
    float;
  _red_type red;
  using _green_type =
    float;
  _green_type green;
  using _blue_type =
    float;
  _blue_type blue;
  using _pattern_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pattern_type pattern;
  using _frequency_type =
    float;
  _frequency_type frequency;
  using _priority_type =
    int32_t;
  _priority_type priority;
  using _duration_type =
    float;
  _duration_type duration;
  using _override_type =
    bool;
  _override_type override;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status_code(
    const int32_t & _arg)
  {
    this->status_code = _arg;
    return *this;
  }
  Type & set__red(
    const float & _arg)
  {
    this->red = _arg;
    return *this;
  }
  Type & set__green(
    const float & _arg)
  {
    this->green = _arg;
    return *this;
  }
  Type & set__blue(
    const float & _arg)
  {
    this->blue = _arg;
    return *this;
  }
  Type & set__pattern(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pattern = _arg;
    return *this;
  }
  Type & set__frequency(
    const float & _arg)
  {
    this->frequency = _arg;
    return *this;
  }
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__override(
    const bool & _arg)
  {
    this->override = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::LedCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::LedCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::LedCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::LedCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__LedCommand
    std::shared_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__LedCommand
    std::shared_ptr<autonomy_interfaces::msg::LedCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LedCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status_code != other.status_code) {
      return false;
    }
    if (this->red != other.red) {
      return false;
    }
    if (this->green != other.green) {
      return false;
    }
    if (this->blue != other.blue) {
      return false;
    }
    if (this->pattern != other.pattern) {
      return false;
    }
    if (this->frequency != other.frequency) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->override != other.override) {
      return false;
    }
    return true;
  }
  bool operator!=(const LedCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LedCommand_

// alias to use template instance with default allocator
using LedCommand =
  autonomy_interfaces::msg::LedCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_
