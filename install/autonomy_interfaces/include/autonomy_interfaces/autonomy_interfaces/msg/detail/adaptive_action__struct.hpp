// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/adaptive_action.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'trigger_context'
#include "autonomy_interfaces/msg/detail/context_state__struct.hpp"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__AdaptiveAction __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__AdaptiveAction __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AdaptiveAction_
{
  using Type = AdaptiveAction_<ContainerAllocator>;

  explicit AdaptiveAction_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trigger_context(_init),
    timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_type = "";
      this->priority = 0l;
      this->expected_duration = 0.0f;
      this->success_criteria = "";
    }
  }

  explicit AdaptiveAction_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : action_type(_alloc),
    trigger_context(_alloc, _init),
    success_criteria(_alloc),
    timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_type = "";
      this->priority = 0l;
      this->expected_duration = 0.0f;
      this->success_criteria = "";
    }
  }

  // field types and members
  using _action_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_type_type action_type;
  using _parameters_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _parameters_type parameters;
  using _trigger_context_type =
    autonomy_interfaces::msg::ContextState_<ContainerAllocator>;
  _trigger_context_type trigger_context;
  using _priority_type =
    int32_t;
  _priority_type priority;
  using _expected_duration_type =
    float;
  _expected_duration_type expected_duration;
  using _success_criteria_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _success_criteria_type success_criteria;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__action_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action_type = _arg;
    return *this;
  }
  Type & set__parameters(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }
  Type & set__trigger_context(
    const autonomy_interfaces::msg::ContextState_<ContainerAllocator> & _arg)
  {
    this->trigger_context = _arg;
    return *this;
  }
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__expected_duration(
    const float & _arg)
  {
    this->expected_duration = _arg;
    return *this;
  }
  Type & set__success_criteria(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->success_criteria = _arg;
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
    autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__AdaptiveAction
    std::shared_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__AdaptiveAction
    std::shared_ptr<autonomy_interfaces::msg::AdaptiveAction_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AdaptiveAction_ & other) const
  {
    if (this->action_type != other.action_type) {
      return false;
    }
    if (this->parameters != other.parameters) {
      return false;
    }
    if (this->trigger_context != other.trigger_context) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->expected_duration != other.expected_duration) {
      return false;
    }
    if (this->success_criteria != other.success_criteria) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const AdaptiveAction_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AdaptiveAction_

// alias to use template instance with default allocator
using AdaptiveAction =
  autonomy_interfaces::msg::AdaptiveAction_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_HPP_
