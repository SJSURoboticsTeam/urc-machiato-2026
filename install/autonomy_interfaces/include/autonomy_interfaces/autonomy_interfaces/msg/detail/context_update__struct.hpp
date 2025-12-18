// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__ContextUpdate __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__ContextUpdate __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContextUpdate_
{
  using Type = ContextUpdate_<ContainerAllocator>;

  explicit ContextUpdate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->mission_status = "";
      this->mission_progress = 0.0f;
      this->communication_active = false;
      this->safety_active = false;
      this->alert_level = "";
    }
  }

  explicit ContextUpdate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission_status(_alloc),
    alert_level(_alloc),
    timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->mission_status = "";
      this->mission_progress = 0.0f;
      this->communication_active = false;
      this->safety_active = false;
      this->alert_level = "";
    }
  }

  // field types and members
  using _battery_level_type =
    float;
  _battery_level_type battery_level;
  using _mission_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_status_type mission_status;
  using _mission_progress_type =
    float;
  _mission_progress_type mission_progress;
  using _communication_active_type =
    bool;
  _communication_active_type communication_active;
  using _safety_active_type =
    bool;
  _safety_active_type safety_active;
  using _active_adaptations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_adaptations_type active_adaptations;
  using _alert_level_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _alert_level_type alert_level;
  using _available_actions_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _available_actions_type available_actions;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__battery_level(
    const float & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }
  Type & set__mission_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_status = _arg;
    return *this;
  }
  Type & set__mission_progress(
    const float & _arg)
  {
    this->mission_progress = _arg;
    return *this;
  }
  Type & set__communication_active(
    const bool & _arg)
  {
    this->communication_active = _arg;
    return *this;
  }
  Type & set__safety_active(
    const bool & _arg)
  {
    this->safety_active = _arg;
    return *this;
  }
  Type & set__active_adaptations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_adaptations = _arg;
    return *this;
  }
  Type & set__alert_level(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->alert_level = _arg;
    return *this;
  }
  Type & set__available_actions(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->available_actions = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__ContextUpdate
    std::shared_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__ContextUpdate
    std::shared_ptr<autonomy_interfaces::msg::ContextUpdate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContextUpdate_ & other) const
  {
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->mission_status != other.mission_status) {
      return false;
    }
    if (this->mission_progress != other.mission_progress) {
      return false;
    }
    if (this->communication_active != other.communication_active) {
      return false;
    }
    if (this->safety_active != other.safety_active) {
      return false;
    }
    if (this->active_adaptations != other.active_adaptations) {
      return false;
    }
    if (this->alert_level != other.alert_level) {
      return false;
    }
    if (this->available_actions != other.available_actions) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContextUpdate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContextUpdate_

// alias to use template instance with default allocator
using ContextUpdate =
  autonomy_interfaces::msg::ContextUpdate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_HPP_
