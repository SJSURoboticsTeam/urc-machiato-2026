// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_

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
// Member 'transition_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__SystemState __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__SystemState __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SystemState_
{
  using Type = SystemState_<ContainerAllocator>;

  explicit SystemState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    transition_timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->substate = "";
      this->sub_substate = "";
      this->time_in_state = 0.0;
      this->state_timeout = 0.0;
      this->previous_state = "";
      this->is_transitioning = false;
      this->preconditions_met = false;
      this->mission_phase = "";
      this->operator_id = "";
      this->state_reason = "";
    }
  }

  explicit SystemState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    current_state(_alloc),
    substate(_alloc),
    sub_substate(_alloc),
    previous_state(_alloc),
    transition_timestamp(_alloc, _init),
    mission_phase(_alloc),
    operator_id(_alloc),
    state_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->substate = "";
      this->sub_substate = "";
      this->time_in_state = 0.0;
      this->state_timeout = 0.0;
      this->previous_state = "";
      this->is_transitioning = false;
      this->preconditions_met = false;
      this->mission_phase = "";
      this->operator_id = "";
      this->state_reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _current_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_state_type current_state;
  using _substate_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _substate_type substate;
  using _sub_substate_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sub_substate_type sub_substate;
  using _time_in_state_type =
    double;
  _time_in_state_type time_in_state;
  using _state_timeout_type =
    double;
  _state_timeout_type state_timeout;
  using _previous_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _previous_state_type previous_state;
  using _transition_timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _transition_timestamp_type transition_timestamp;
  using _is_transitioning_type =
    bool;
  _is_transitioning_type is_transitioning;
  using _preconditions_met_type =
    bool;
  _preconditions_met_type preconditions_met;
  using _active_subsystems_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_subsystems_type active_subsystems;
  using _failed_subsystems_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _failed_subsystems_type failed_subsystems;
  using _mission_phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_phase_type mission_phase;
  using _operator_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operator_id_type operator_id;
  using _state_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_reason_type state_reason;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__current_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_state = _arg;
    return *this;
  }
  Type & set__substate(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->substate = _arg;
    return *this;
  }
  Type & set__sub_substate(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sub_substate = _arg;
    return *this;
  }
  Type & set__time_in_state(
    const double & _arg)
  {
    this->time_in_state = _arg;
    return *this;
  }
  Type & set__state_timeout(
    const double & _arg)
  {
    this->state_timeout = _arg;
    return *this;
  }
  Type & set__previous_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->previous_state = _arg;
    return *this;
  }
  Type & set__transition_timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->transition_timestamp = _arg;
    return *this;
  }
  Type & set__is_transitioning(
    const bool & _arg)
  {
    this->is_transitioning = _arg;
    return *this;
  }
  Type & set__preconditions_met(
    const bool & _arg)
  {
    this->preconditions_met = _arg;
    return *this;
  }
  Type & set__active_subsystems(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_subsystems = _arg;
    return *this;
  }
  Type & set__failed_subsystems(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->failed_subsystems = _arg;
    return *this;
  }
  Type & set__mission_phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_phase = _arg;
    return *this;
  }
  Type & set__operator_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operator_id = _arg;
    return *this;
  }
  Type & set__state_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state_reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::SystemState_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::SystemState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SystemState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SystemState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__SystemState
    std::shared_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__SystemState
    std::shared_ptr<autonomy_interfaces::msg::SystemState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SystemState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->current_state != other.current_state) {
      return false;
    }
    if (this->substate != other.substate) {
      return false;
    }
    if (this->sub_substate != other.sub_substate) {
      return false;
    }
    if (this->time_in_state != other.time_in_state) {
      return false;
    }
    if (this->state_timeout != other.state_timeout) {
      return false;
    }
    if (this->previous_state != other.previous_state) {
      return false;
    }
    if (this->transition_timestamp != other.transition_timestamp) {
      return false;
    }
    if (this->is_transitioning != other.is_transitioning) {
      return false;
    }
    if (this->preconditions_met != other.preconditions_met) {
      return false;
    }
    if (this->active_subsystems != other.active_subsystems) {
      return false;
    }
    if (this->failed_subsystems != other.failed_subsystems) {
      return false;
    }
    if (this->mission_phase != other.mission_phase) {
      return false;
    }
    if (this->operator_id != other.operator_id) {
      return false;
    }
    if (this->state_reason != other.state_reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const SystemState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SystemState_

// alias to use template instance with default allocator
using SystemState =
  autonomy_interfaces::msg::SystemState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_
