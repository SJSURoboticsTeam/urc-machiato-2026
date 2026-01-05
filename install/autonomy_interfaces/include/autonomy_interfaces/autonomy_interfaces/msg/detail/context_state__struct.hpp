// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_state.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_HPP_

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
# define DEPRECATED__autonomy_interfaces__msg__ContextState __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__ContextState __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContextState_
{
  using Type = ContextState_<ContainerAllocator>;

  explicit ContextState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->battery_voltage = 0.0f;
      this->battery_critical = false;
      this->battery_warning = false;
      this->mission_type = "";
      this->mission_status = "";
      this->mission_progress = 0.0f;
      this->mission_time_remaining = 0.0f;
      this->communication_active = false;
      this->communication_latency = 0.0f;
      this->communication_quality = 0l;
      this->cpu_usage = 0.0f;
      this->memory_usage = 0.0f;
      this->temperature = 0.0f;
      this->obstacle_detected = false;
      this->obstacle_distance = 0.0f;
      this->terrain_difficulty = 0.0f;
      this->weather_adverse = false;
      this->safety_active = false;
      this->safety_reason = "";
    }
  }

  explicit ContextState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission_type(_alloc),
    mission_status(_alloc),
    safety_reason(_alloc),
    timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->battery_voltage = 0.0f;
      this->battery_critical = false;
      this->battery_warning = false;
      this->mission_type = "";
      this->mission_status = "";
      this->mission_progress = 0.0f;
      this->mission_time_remaining = 0.0f;
      this->communication_active = false;
      this->communication_latency = 0.0f;
      this->communication_quality = 0l;
      this->cpu_usage = 0.0f;
      this->memory_usage = 0.0f;
      this->temperature = 0.0f;
      this->obstacle_detected = false;
      this->obstacle_distance = 0.0f;
      this->terrain_difficulty = 0.0f;
      this->weather_adverse = false;
      this->safety_active = false;
      this->safety_reason = "";
    }
  }

  // field types and members
  using _battery_level_type =
    float;
  _battery_level_type battery_level;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_critical_type =
    bool;
  _battery_critical_type battery_critical;
  using _battery_warning_type =
    bool;
  _battery_warning_type battery_warning;
  using _mission_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type_type mission_type;
  using _mission_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_status_type mission_status;
  using _mission_progress_type =
    float;
  _mission_progress_type mission_progress;
  using _mission_time_remaining_type =
    float;
  _mission_time_remaining_type mission_time_remaining;
  using _communication_active_type =
    bool;
  _communication_active_type communication_active;
  using _communication_latency_type =
    float;
  _communication_latency_type communication_latency;
  using _communication_quality_type =
    int32_t;
  _communication_quality_type communication_quality;
  using _cpu_usage_type =
    float;
  _cpu_usage_type cpu_usage;
  using _memory_usage_type =
    float;
  _memory_usage_type memory_usage;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _obstacle_detected_type =
    bool;
  _obstacle_detected_type obstacle_detected;
  using _obstacle_distance_type =
    float;
  _obstacle_distance_type obstacle_distance;
  using _terrain_difficulty_type =
    float;
  _terrain_difficulty_type terrain_difficulty;
  using _weather_adverse_type =
    bool;
  _weather_adverse_type weather_adverse;
  using _safety_active_type =
    bool;
  _safety_active_type safety_active;
  using _safety_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _safety_reason_type safety_reason;
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
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_critical(
    const bool & _arg)
  {
    this->battery_critical = _arg;
    return *this;
  }
  Type & set__battery_warning(
    const bool & _arg)
  {
    this->battery_warning = _arg;
    return *this;
  }
  Type & set__mission_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_type = _arg;
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
  Type & set__mission_time_remaining(
    const float & _arg)
  {
    this->mission_time_remaining = _arg;
    return *this;
  }
  Type & set__communication_active(
    const bool & _arg)
  {
    this->communication_active = _arg;
    return *this;
  }
  Type & set__communication_latency(
    const float & _arg)
  {
    this->communication_latency = _arg;
    return *this;
  }
  Type & set__communication_quality(
    const int32_t & _arg)
  {
    this->communication_quality = _arg;
    return *this;
  }
  Type & set__cpu_usage(
    const float & _arg)
  {
    this->cpu_usage = _arg;
    return *this;
  }
  Type & set__memory_usage(
    const float & _arg)
  {
    this->memory_usage = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__obstacle_detected(
    const bool & _arg)
  {
    this->obstacle_detected = _arg;
    return *this;
  }
  Type & set__obstacle_distance(
    const float & _arg)
  {
    this->obstacle_distance = _arg;
    return *this;
  }
  Type & set__terrain_difficulty(
    const float & _arg)
  {
    this->terrain_difficulty = _arg;
    return *this;
  }
  Type & set__weather_adverse(
    const bool & _arg)
  {
    this->weather_adverse = _arg;
    return *this;
  }
  Type & set__safety_active(
    const bool & _arg)
  {
    this->safety_active = _arg;
    return *this;
  }
  Type & set__safety_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->safety_reason = _arg;
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
    autonomy_interfaces::msg::ContextState_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::ContextState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ContextState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ContextState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__ContextState
    std::shared_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__ContextState
    std::shared_ptr<autonomy_interfaces::msg::ContextState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContextState_ & other) const
  {
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_critical != other.battery_critical) {
      return false;
    }
    if (this->battery_warning != other.battery_warning) {
      return false;
    }
    if (this->mission_type != other.mission_type) {
      return false;
    }
    if (this->mission_status != other.mission_status) {
      return false;
    }
    if (this->mission_progress != other.mission_progress) {
      return false;
    }
    if (this->mission_time_remaining != other.mission_time_remaining) {
      return false;
    }
    if (this->communication_active != other.communication_active) {
      return false;
    }
    if (this->communication_latency != other.communication_latency) {
      return false;
    }
    if (this->communication_quality != other.communication_quality) {
      return false;
    }
    if (this->cpu_usage != other.cpu_usage) {
      return false;
    }
    if (this->memory_usage != other.memory_usage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->obstacle_detected != other.obstacle_detected) {
      return false;
    }
    if (this->obstacle_distance != other.obstacle_distance) {
      return false;
    }
    if (this->terrain_difficulty != other.terrain_difficulty) {
      return false;
    }
    if (this->weather_adverse != other.weather_adverse) {
      return false;
    }
    if (this->safety_active != other.safety_active) {
      return false;
    }
    if (this->safety_reason != other.safety_reason) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContextState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContextState_

// alias to use template instance with default allocator
using ContextState =
  autonomy_interfaces::msg::ContextState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_HPP_
