// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_update.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/context_update__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_ContextUpdate_timestamp
{
public:
  explicit Init_ContextUpdate_timestamp(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::ContextUpdate timestamp(::autonomy_interfaces::msg::ContextUpdate::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_available_actions
{
public:
  explicit Init_ContextUpdate_available_actions(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_timestamp available_actions(::autonomy_interfaces::msg::ContextUpdate::_available_actions_type arg)
  {
    msg_.available_actions = std::move(arg);
    return Init_ContextUpdate_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_alert_level
{
public:
  explicit Init_ContextUpdate_alert_level(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_available_actions alert_level(::autonomy_interfaces::msg::ContextUpdate::_alert_level_type arg)
  {
    msg_.alert_level = std::move(arg);
    return Init_ContextUpdate_available_actions(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_active_adaptations
{
public:
  explicit Init_ContextUpdate_active_adaptations(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_alert_level active_adaptations(::autonomy_interfaces::msg::ContextUpdate::_active_adaptations_type arg)
  {
    msg_.active_adaptations = std::move(arg);
    return Init_ContextUpdate_alert_level(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_safety_active
{
public:
  explicit Init_ContextUpdate_safety_active(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_active_adaptations safety_active(::autonomy_interfaces::msg::ContextUpdate::_safety_active_type arg)
  {
    msg_.safety_active = std::move(arg);
    return Init_ContextUpdate_active_adaptations(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_communication_active
{
public:
  explicit Init_ContextUpdate_communication_active(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_safety_active communication_active(::autonomy_interfaces::msg::ContextUpdate::_communication_active_type arg)
  {
    msg_.communication_active = std::move(arg);
    return Init_ContextUpdate_safety_active(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_mission_progress
{
public:
  explicit Init_ContextUpdate_mission_progress(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_communication_active mission_progress(::autonomy_interfaces::msg::ContextUpdate::_mission_progress_type arg)
  {
    msg_.mission_progress = std::move(arg);
    return Init_ContextUpdate_communication_active(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_mission_status
{
public:
  explicit Init_ContextUpdate_mission_status(::autonomy_interfaces::msg::ContextUpdate & msg)
  : msg_(msg)
  {}
  Init_ContextUpdate_mission_progress mission_status(::autonomy_interfaces::msg::ContextUpdate::_mission_status_type arg)
  {
    msg_.mission_status = std::move(arg);
    return Init_ContextUpdate_mission_progress(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

class Init_ContextUpdate_battery_level
{
public:
  Init_ContextUpdate_battery_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContextUpdate_mission_status battery_level(::autonomy_interfaces::msg::ContextUpdate::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_ContextUpdate_mission_status(msg_);
  }

private:
  ::autonomy_interfaces::msg::ContextUpdate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::ContextUpdate>()
{
  return autonomy_interfaces::msg::builder::Init_ContextUpdate_battery_level();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__BUILDER_HPP_
