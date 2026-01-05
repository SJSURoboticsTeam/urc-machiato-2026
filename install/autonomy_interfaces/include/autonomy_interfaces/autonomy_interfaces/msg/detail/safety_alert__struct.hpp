// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_alert.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__SafetyAlert __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__SafetyAlert __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetyAlert_
{
  using Type = SafetyAlert_<ContainerAllocator>;

  explicit SafetyAlert_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->property = "";
      this->severity = "";
      this->details = "";
      this->timestamp = 0.0;
      this->acknowledged = false;
    }
  }

  explicit SafetyAlert_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : property(_alloc),
    severity(_alloc),
    details(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->property = "";
      this->severity = "";
      this->details = "";
      this->timestamp = 0.0;
      this->acknowledged = false;
    }
  }

  // field types and members
  using _property_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _property_type property;
  using _severity_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _severity_type severity;
  using _details_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _details_type details;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _acknowledged_type =
    bool;
  _acknowledged_type acknowledged;

  // setters for named parameter idiom
  Type & set__property(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->property = _arg;
    return *this;
  }
  Type & set__severity(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->severity = _arg;
    return *this;
  }
  Type & set__details(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->details = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__acknowledged(
    const bool & _arg)
  {
    this->acknowledged = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__SafetyAlert
    std::shared_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__SafetyAlert
    std::shared_ptr<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetyAlert_ & other) const
  {
    if (this->property != other.property) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    if (this->details != other.details) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->acknowledged != other.acknowledged) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetyAlert_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetyAlert_

// alias to use template instance with default allocator
using SafetyAlert =
  autonomy_interfaces::msg::SafetyAlert_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__STRUCT_HPP_
