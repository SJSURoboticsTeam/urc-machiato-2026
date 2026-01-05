// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_HPP_

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
// Member 'trigger_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__SafetyStatus __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__SafetyStatus __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetyStatus_
{
  using Type = SafetyStatus_<ContainerAllocator>;

  explicit SafetyStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    trigger_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_safe = false;
      this->safety_level = "";
      this->trigger_type = "";
      this->trigger_source = "";
      this->trigger_description = "";
      this->requires_manual_intervention = false;
      this->can_auto_recover = false;
      this->estimated_recovery_time = 0.0;
      this->context_state = "";
      this->mission_phase = "";
      this->safe_to_retry = false;
      this->battery_level = 0.0;
      this->temperature = 0.0;
      this->communication_ok = false;
      this->sensors_ok = false;
    }
  }

  explicit SafetyStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    safety_level(_alloc),
    trigger_type(_alloc),
    trigger_source(_alloc),
    trigger_time(_alloc, _init),
    trigger_description(_alloc),
    context_state(_alloc),
    mission_phase(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_safe = false;
      this->safety_level = "";
      this->trigger_type = "";
      this->trigger_source = "";
      this->trigger_description = "";
      this->requires_manual_intervention = false;
      this->can_auto_recover = false;
      this->estimated_recovery_time = 0.0;
      this->context_state = "";
      this->mission_phase = "";
      this->safe_to_retry = false;
      this->battery_level = 0.0;
      this->temperature = 0.0;
      this->communication_ok = false;
      this->sensors_ok = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_safe_type =
    bool;
  _is_safe_type is_safe;
  using _safety_level_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _safety_level_type safety_level;
  using _active_triggers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_triggers_type active_triggers;
  using _trigger_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _trigger_type_type trigger_type;
  using _trigger_source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _trigger_source_type trigger_source;
  using _trigger_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _trigger_time_type trigger_time;
  using _trigger_description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _trigger_description_type trigger_description;
  using _requires_manual_intervention_type =
    bool;
  _requires_manual_intervention_type requires_manual_intervention;
  using _can_auto_recover_type =
    bool;
  _can_auto_recover_type can_auto_recover;
  using _recovery_steps_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _recovery_steps_type recovery_steps;
  using _estimated_recovery_time_type =
    double;
  _estimated_recovery_time_type estimated_recovery_time;
  using _context_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _context_state_type context_state;
  using _mission_phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_phase_type mission_phase;
  using _safe_to_retry_type =
    bool;
  _safe_to_retry_type safe_to_retry;
  using _battery_level_type =
    double;
  _battery_level_type battery_level;
  using _temperature_type =
    double;
  _temperature_type temperature;
  using _communication_ok_type =
    bool;
  _communication_ok_type communication_ok;
  using _sensors_ok_type =
    bool;
  _sensors_ok_type sensors_ok;
  using _degraded_capabilities_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _degraded_capabilities_type degraded_capabilities;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_safe(
    const bool & _arg)
  {
    this->is_safe = _arg;
    return *this;
  }
  Type & set__safety_level(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->safety_level = _arg;
    return *this;
  }
  Type & set__active_triggers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_triggers = _arg;
    return *this;
  }
  Type & set__trigger_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->trigger_type = _arg;
    return *this;
  }
  Type & set__trigger_source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->trigger_source = _arg;
    return *this;
  }
  Type & set__trigger_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->trigger_time = _arg;
    return *this;
  }
  Type & set__trigger_description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->trigger_description = _arg;
    return *this;
  }
  Type & set__requires_manual_intervention(
    const bool & _arg)
  {
    this->requires_manual_intervention = _arg;
    return *this;
  }
  Type & set__can_auto_recover(
    const bool & _arg)
  {
    this->can_auto_recover = _arg;
    return *this;
  }
  Type & set__recovery_steps(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->recovery_steps = _arg;
    return *this;
  }
  Type & set__estimated_recovery_time(
    const double & _arg)
  {
    this->estimated_recovery_time = _arg;
    return *this;
  }
  Type & set__context_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->context_state = _arg;
    return *this;
  }
  Type & set__mission_phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_phase = _arg;
    return *this;
  }
  Type & set__safe_to_retry(
    const bool & _arg)
  {
    this->safe_to_retry = _arg;
    return *this;
  }
  Type & set__battery_level(
    const double & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }
  Type & set__temperature(
    const double & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__communication_ok(
    const bool & _arg)
  {
    this->communication_ok = _arg;
    return *this;
  }
  Type & set__sensors_ok(
    const bool & _arg)
  {
    this->sensors_ok = _arg;
    return *this;
  }
  Type & set__degraded_capabilities(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->degraded_capabilities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__SafetyStatus
    std::shared_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__SafetyStatus
    std::shared_ptr<autonomy_interfaces::msg::SafetyStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetyStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_safe != other.is_safe) {
      return false;
    }
    if (this->safety_level != other.safety_level) {
      return false;
    }
    if (this->active_triggers != other.active_triggers) {
      return false;
    }
    if (this->trigger_type != other.trigger_type) {
      return false;
    }
    if (this->trigger_source != other.trigger_source) {
      return false;
    }
    if (this->trigger_time != other.trigger_time) {
      return false;
    }
    if (this->trigger_description != other.trigger_description) {
      return false;
    }
    if (this->requires_manual_intervention != other.requires_manual_intervention) {
      return false;
    }
    if (this->can_auto_recover != other.can_auto_recover) {
      return false;
    }
    if (this->recovery_steps != other.recovery_steps) {
      return false;
    }
    if (this->estimated_recovery_time != other.estimated_recovery_time) {
      return false;
    }
    if (this->context_state != other.context_state) {
      return false;
    }
    if (this->mission_phase != other.mission_phase) {
      return false;
    }
    if (this->safe_to_retry != other.safe_to_retry) {
      return false;
    }
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->communication_ok != other.communication_ok) {
      return false;
    }
    if (this->sensors_ok != other.sensors_ok) {
      return false;
    }
    if (this->degraded_capabilities != other.degraded_capabilities) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetyStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetyStatus_

// alias to use template instance with default allocator
using SafetyStatus =
  autonomy_interfaces::msg::SafetyStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_HPP_
