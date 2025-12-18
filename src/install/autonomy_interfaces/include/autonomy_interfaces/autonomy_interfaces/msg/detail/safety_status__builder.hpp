// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/safety_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_SafetyStatus_degraded_capabilities
{
public:
  explicit Init_SafetyStatus_degraded_capabilities(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::SafetyStatus degraded_capabilities(::autonomy_interfaces::msg::SafetyStatus::_degraded_capabilities_type arg)
  {
    msg_.degraded_capabilities = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_sensors_ok
{
public:
  explicit Init_SafetyStatus_sensors_ok(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_degraded_capabilities sensors_ok(::autonomy_interfaces::msg::SafetyStatus::_sensors_ok_type arg)
  {
    msg_.sensors_ok = std::move(arg);
    return Init_SafetyStatus_degraded_capabilities(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_communication_ok
{
public:
  explicit Init_SafetyStatus_communication_ok(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_sensors_ok communication_ok(::autonomy_interfaces::msg::SafetyStatus::_communication_ok_type arg)
  {
    msg_.communication_ok = std::move(arg);
    return Init_SafetyStatus_sensors_ok(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_temperature
{
public:
  explicit Init_SafetyStatus_temperature(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_communication_ok temperature(::autonomy_interfaces::msg::SafetyStatus::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_SafetyStatus_communication_ok(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_battery_level
{
public:
  explicit Init_SafetyStatus_battery_level(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_temperature battery_level(::autonomy_interfaces::msg::SafetyStatus::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_SafetyStatus_temperature(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_safe_to_retry
{
public:
  explicit Init_SafetyStatus_safe_to_retry(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_battery_level safe_to_retry(::autonomy_interfaces::msg::SafetyStatus::_safe_to_retry_type arg)
  {
    msg_.safe_to_retry = std::move(arg);
    return Init_SafetyStatus_battery_level(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_mission_phase
{
public:
  explicit Init_SafetyStatus_mission_phase(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_safe_to_retry mission_phase(::autonomy_interfaces::msg::SafetyStatus::_mission_phase_type arg)
  {
    msg_.mission_phase = std::move(arg);
    return Init_SafetyStatus_safe_to_retry(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_context_state
{
public:
  explicit Init_SafetyStatus_context_state(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_mission_phase context_state(::autonomy_interfaces::msg::SafetyStatus::_context_state_type arg)
  {
    msg_.context_state = std::move(arg);
    return Init_SafetyStatus_mission_phase(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_estimated_recovery_time
{
public:
  explicit Init_SafetyStatus_estimated_recovery_time(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_context_state estimated_recovery_time(::autonomy_interfaces::msg::SafetyStatus::_estimated_recovery_time_type arg)
  {
    msg_.estimated_recovery_time = std::move(arg);
    return Init_SafetyStatus_context_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_recovery_steps
{
public:
  explicit Init_SafetyStatus_recovery_steps(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_estimated_recovery_time recovery_steps(::autonomy_interfaces::msg::SafetyStatus::_recovery_steps_type arg)
  {
    msg_.recovery_steps = std::move(arg);
    return Init_SafetyStatus_estimated_recovery_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_can_auto_recover
{
public:
  explicit Init_SafetyStatus_can_auto_recover(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_recovery_steps can_auto_recover(::autonomy_interfaces::msg::SafetyStatus::_can_auto_recover_type arg)
  {
    msg_.can_auto_recover = std::move(arg);
    return Init_SafetyStatus_recovery_steps(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_requires_manual_intervention
{
public:
  explicit Init_SafetyStatus_requires_manual_intervention(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_can_auto_recover requires_manual_intervention(::autonomy_interfaces::msg::SafetyStatus::_requires_manual_intervention_type arg)
  {
    msg_.requires_manual_intervention = std::move(arg);
    return Init_SafetyStatus_can_auto_recover(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_trigger_description
{
public:
  explicit Init_SafetyStatus_trigger_description(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_requires_manual_intervention trigger_description(::autonomy_interfaces::msg::SafetyStatus::_trigger_description_type arg)
  {
    msg_.trigger_description = std::move(arg);
    return Init_SafetyStatus_requires_manual_intervention(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_trigger_time
{
public:
  explicit Init_SafetyStatus_trigger_time(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_trigger_description trigger_time(::autonomy_interfaces::msg::SafetyStatus::_trigger_time_type arg)
  {
    msg_.trigger_time = std::move(arg);
    return Init_SafetyStatus_trigger_description(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_trigger_source
{
public:
  explicit Init_SafetyStatus_trigger_source(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_trigger_time trigger_source(::autonomy_interfaces::msg::SafetyStatus::_trigger_source_type arg)
  {
    msg_.trigger_source = std::move(arg);
    return Init_SafetyStatus_trigger_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_trigger_type
{
public:
  explicit Init_SafetyStatus_trigger_type(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_trigger_source trigger_type(::autonomy_interfaces::msg::SafetyStatus::_trigger_type_type arg)
  {
    msg_.trigger_type = std::move(arg);
    return Init_SafetyStatus_trigger_source(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_active_triggers
{
public:
  explicit Init_SafetyStatus_active_triggers(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_trigger_type active_triggers(::autonomy_interfaces::msg::SafetyStatus::_active_triggers_type arg)
  {
    msg_.active_triggers = std::move(arg);
    return Init_SafetyStatus_trigger_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_safety_level
{
public:
  explicit Init_SafetyStatus_safety_level(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_active_triggers safety_level(::autonomy_interfaces::msg::SafetyStatus::_safety_level_type arg)
  {
    msg_.safety_level = std::move(arg);
    return Init_SafetyStatus_active_triggers(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_is_safe
{
public:
  explicit Init_SafetyStatus_is_safe(::autonomy_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_safety_level is_safe(::autonomy_interfaces::msg::SafetyStatus::_is_safe_type arg)
  {
    msg_.is_safe = std::move(arg);
    return Init_SafetyStatus_safety_level(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_header
{
public:
  Init_SafetyStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafetyStatus_is_safe header(::autonomy_interfaces::msg::SafetyStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SafetyStatus_is_safe(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::SafetyStatus>()
{
  return autonomy_interfaces::msg::builder::Init_SafetyStatus_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_
