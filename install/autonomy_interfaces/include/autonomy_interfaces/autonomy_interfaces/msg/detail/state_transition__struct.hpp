// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/state_transition.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_HPP_

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
// Member 'start_time'
// Member 'end_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__StateTransition __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__StateTransition __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateTransition_
{
  using Type = StateTransition_<ContainerAllocator>;

  explicit StateTransition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    start_time(_init),
    end_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_state = "";
      this->to_state = "";
      this->transition_duration = 0.0;
      this->success = false;
      this->reason = "";
      this->initiated_by = "";
      this->failure_reason = "";
    }
  }

  explicit StateTransition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    from_state(_alloc),
    to_state(_alloc),
    start_time(_alloc, _init),
    end_time(_alloc, _init),
    reason(_alloc),
    initiated_by(_alloc),
    failure_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_state = "";
      this->to_state = "";
      this->transition_duration = 0.0;
      this->success = false;
      this->reason = "";
      this->initiated_by = "";
      this->failure_reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _from_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _from_state_type from_state;
  using _to_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _to_state_type to_state;
  using _start_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _start_time_type start_time;
  using _end_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _end_time_type end_time;
  using _transition_duration_type =
    double;
  _transition_duration_type transition_duration;
  using _success_type =
    bool;
  _success_type success;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _initiated_by_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _initiated_by_type initiated_by;
  using _failure_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _failure_reason_type failure_reason;
  using _preconditions_checked_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _preconditions_checked_type preconditions_checked;
  using _entry_actions_executed_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _entry_actions_executed_type entry_actions_executed;
  using _exit_actions_executed_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _exit_actions_executed_type exit_actions_executed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__from_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->from_state = _arg;
    return *this;
  }
  Type & set__to_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->to_state = _arg;
    return *this;
  }
  Type & set__start_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->start_time = _arg;
    return *this;
  }
  Type & set__end_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->end_time = _arg;
    return *this;
  }
  Type & set__transition_duration(
    const double & _arg)
  {
    this->transition_duration = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__initiated_by(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->initiated_by = _arg;
    return *this;
  }
  Type & set__failure_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->failure_reason = _arg;
    return *this;
  }
  Type & set__preconditions_checked(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->preconditions_checked = _arg;
    return *this;
  }
  Type & set__entry_actions_executed(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->entry_actions_executed = _arg;
    return *this;
  }
  Type & set__exit_actions_executed(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->exit_actions_executed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::StateTransition_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::StateTransition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::StateTransition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::StateTransition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__StateTransition
    std::shared_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__StateTransition
    std::shared_ptr<autonomy_interfaces::msg::StateTransition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateTransition_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->from_state != other.from_state) {
      return false;
    }
    if (this->to_state != other.to_state) {
      return false;
    }
    if (this->start_time != other.start_time) {
      return false;
    }
    if (this->end_time != other.end_time) {
      return false;
    }
    if (this->transition_duration != other.transition_duration) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->initiated_by != other.initiated_by) {
      return false;
    }
    if (this->failure_reason != other.failure_reason) {
      return false;
    }
    if (this->preconditions_checked != other.preconditions_checked) {
      return false;
    }
    if (this->entry_actions_executed != other.entry_actions_executed) {
      return false;
    }
    if (this->exit_actions_executed != other.exit_actions_executed) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateTransition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateTransition_

// alias to use template instance with default allocator
using StateTransition =
  autonomy_interfaces::msg::StateTransition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_HPP_
