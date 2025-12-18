// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/context_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_ContextState_timestamp
{
public:
  explicit Init_ContextState_timestamp(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::ContextState timestamp(::autonomy_interfaces::msg::ContextState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_safety_reason
{
public:
  explicit Init_ContextState_safety_reason(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_timestamp safety_reason(::autonomy_interfaces::msg::ContextState::_safety_reason_type arg)
  {
    msg_.safety_reason = std::move(arg);
    return Init_ContextState_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_safety_active
{
public:
  explicit Init_ContextState_safety_active(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_safety_reason safety_active(::autonomy_interfaces::msg::ContextState::_safety_active_type arg)
  {
    msg_.safety_active = std::move(arg);
    return Init_ContextState_safety_reason(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_weather_adverse
{
public:
  explicit Init_ContextState_weather_adverse(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_safety_active weather_adverse(::autonomy_interfaces::msg::ContextState::_weather_adverse_type arg)
  {
    msg_.weather_adverse = std::move(arg);
    return Init_ContextState_safety_active(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_terrain_difficulty
{
public:
  explicit Init_ContextState_terrain_difficulty(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_weather_adverse terrain_difficulty(::autonomy_interfaces::msg::ContextState::_terrain_difficulty_type arg)
  {
    msg_.terrain_difficulty = std::move(arg);
    return Init_ContextState_weather_adverse(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_obstacle_distance
{
public:
  explicit Init_ContextState_obstacle_distance(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_terrain_difficulty obstacle_distance(::autonomy_interfaces::msg::ContextState::_obstacle_distance_type arg)
  {
    msg_.obstacle_distance = std::move(arg);
    return Init_ContextState_terrain_difficulty(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_obstacle_detected
{
public:
  explicit Init_ContextState_obstacle_detected(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_obstacle_distance obstacle_detected(::autonomy_interfaces::msg::ContextState::_obstacle_detected_type arg)
  {
    msg_.obstacle_detected = std::move(arg);
    return Init_ContextState_obstacle_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_temperature
{
public:
  explicit Init_ContextState_temperature(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_obstacle_detected temperature(::autonomy_interfaces::msg::ContextState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_ContextState_obstacle_detected(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_memory_usage
{
public:
  explicit Init_ContextState_memory_usage(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_temperature memory_usage(::autonomy_interfaces::msg::ContextState::_memory_usage_type arg)
  {
    msg_.memory_usage = std::move(arg);
    return Init_ContextState_temperature(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_cpu_usage
{
public:
  explicit Init_ContextState_cpu_usage(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_memory_usage cpu_usage(::autonomy_interfaces::msg::ContextState::_cpu_usage_type arg)
  {
    msg_.cpu_usage = std::move(arg);
    return Init_ContextState_memory_usage(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_communication_quality
{
public:
  explicit Init_ContextState_communication_quality(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_cpu_usage communication_quality(::autonomy_interfaces::msg::ContextState::_communication_quality_type arg)
  {
    msg_.communication_quality = std::move(arg);
    return Init_ContextState_cpu_usage(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_communication_latency
{
public:
  explicit Init_ContextState_communication_latency(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_communication_quality communication_latency(::autonomy_interfaces::msg::ContextState::_communication_latency_type arg)
  {
    msg_.communication_latency = std::move(arg);
    return Init_ContextState_communication_quality(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_communication_active
{
public:
  explicit Init_ContextState_communication_active(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_communication_latency communication_active(::autonomy_interfaces::msg::ContextState::_communication_active_type arg)
  {
    msg_.communication_active = std::move(arg);
    return Init_ContextState_communication_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_mission_time_remaining
{
public:
  explicit Init_ContextState_mission_time_remaining(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_communication_active mission_time_remaining(::autonomy_interfaces::msg::ContextState::_mission_time_remaining_type arg)
  {
    msg_.mission_time_remaining = std::move(arg);
    return Init_ContextState_communication_active(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_mission_progress
{
public:
  explicit Init_ContextState_mission_progress(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_mission_time_remaining mission_progress(::autonomy_interfaces::msg::ContextState::_mission_progress_type arg)
  {
    msg_.mission_progress = std::move(arg);
    return Init_ContextState_mission_time_remaining(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_mission_status
{
public:
  explicit Init_ContextState_mission_status(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_mission_progress mission_status(::autonomy_interfaces::msg::ContextState::_mission_status_type arg)
  {
    msg_.mission_status = std::move(arg);
    return Init_ContextState_mission_progress(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_mission_type
{
public:
  explicit Init_ContextState_mission_type(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_mission_status mission_type(::autonomy_interfaces::msg::ContextState::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return Init_ContextState_mission_status(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_battery_warning
{
public:
  explicit Init_ContextState_battery_warning(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_mission_type battery_warning(::autonomy_interfaces::msg::ContextState::_battery_warning_type arg)
  {
    msg_.battery_warning = std::move(arg);
    return Init_ContextState_mission_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_battery_critical
{
public:
  explicit Init_ContextState_battery_critical(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_battery_warning battery_critical(::autonomy_interfaces::msg::ContextState::_battery_critical_type arg)
  {
    msg_.battery_critical = std::move(arg);
    return Init_ContextState_battery_warning(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_battery_voltage
{
public:
  explicit Init_ContextState_battery_voltage(::autonomy_interfaces::msg::ContextState & msg)
  : msg_(msg)
  {}
  Init_ContextState_battery_critical battery_voltage(::autonomy_interfaces::msg::ContextState::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_ContextState_battery_critical(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

class Init_ContextState_battery_level
{
public:
  Init_ContextState_battery_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContextState_battery_voltage battery_level(::autonomy_interfaces::msg::ContextState::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_ContextState_battery_voltage(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::ContextState>()
{
  return autonomy_interfaces::msg::builder::Init_ContextState_battery_level();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__BUILDER_HPP_
