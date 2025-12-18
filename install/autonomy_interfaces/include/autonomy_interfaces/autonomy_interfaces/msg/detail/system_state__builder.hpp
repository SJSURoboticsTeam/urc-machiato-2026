// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/system_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_SystemState_state_reason
{
public:
  explicit Init_SystemState_state_reason(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::SystemState state_reason(::autonomy_interfaces::msg::SystemState::_state_reason_type arg)
  {
    msg_.state_reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_operator_id
{
public:
  explicit Init_SystemState_operator_id(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_state_reason operator_id(::autonomy_interfaces::msg::SystemState::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return Init_SystemState_state_reason(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_mission_phase
{
public:
  explicit Init_SystemState_mission_phase(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_operator_id mission_phase(::autonomy_interfaces::msg::SystemState::_mission_phase_type arg)
  {
    msg_.mission_phase = std::move(arg);
    return Init_SystemState_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_failed_subsystems
{
public:
  explicit Init_SystemState_failed_subsystems(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_mission_phase failed_subsystems(::autonomy_interfaces::msg::SystemState::_failed_subsystems_type arg)
  {
    msg_.failed_subsystems = std::move(arg);
    return Init_SystemState_mission_phase(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_active_subsystems
{
public:
  explicit Init_SystemState_active_subsystems(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_failed_subsystems active_subsystems(::autonomy_interfaces::msg::SystemState::_active_subsystems_type arg)
  {
    msg_.active_subsystems = std::move(arg);
    return Init_SystemState_failed_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_preconditions_met
{
public:
  explicit Init_SystemState_preconditions_met(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_active_subsystems preconditions_met(::autonomy_interfaces::msg::SystemState::_preconditions_met_type arg)
  {
    msg_.preconditions_met = std::move(arg);
    return Init_SystemState_active_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_is_transitioning
{
public:
  explicit Init_SystemState_is_transitioning(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_preconditions_met is_transitioning(::autonomy_interfaces::msg::SystemState::_is_transitioning_type arg)
  {
    msg_.is_transitioning = std::move(arg);
    return Init_SystemState_preconditions_met(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_transition_timestamp
{
public:
  explicit Init_SystemState_transition_timestamp(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_is_transitioning transition_timestamp(::autonomy_interfaces::msg::SystemState::_transition_timestamp_type arg)
  {
    msg_.transition_timestamp = std::move(arg);
    return Init_SystemState_is_transitioning(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_previous_state
{
public:
  explicit Init_SystemState_previous_state(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_transition_timestamp previous_state(::autonomy_interfaces::msg::SystemState::_previous_state_type arg)
  {
    msg_.previous_state = std::move(arg);
    return Init_SystemState_transition_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_state_timeout
{
public:
  explicit Init_SystemState_state_timeout(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_previous_state state_timeout(::autonomy_interfaces::msg::SystemState::_state_timeout_type arg)
  {
    msg_.state_timeout = std::move(arg);
    return Init_SystemState_previous_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_time_in_state
{
public:
  explicit Init_SystemState_time_in_state(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_state_timeout time_in_state(::autonomy_interfaces::msg::SystemState::_time_in_state_type arg)
  {
    msg_.time_in_state = std::move(arg);
    return Init_SystemState_state_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_sub_substate
{
public:
  explicit Init_SystemState_sub_substate(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_time_in_state sub_substate(::autonomy_interfaces::msg::SystemState::_sub_substate_type arg)
  {
    msg_.sub_substate = std::move(arg);
    return Init_SystemState_time_in_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_substate
{
public:
  explicit Init_SystemState_substate(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_sub_substate substate(::autonomy_interfaces::msg::SystemState::_substate_type arg)
  {
    msg_.substate = std::move(arg);
    return Init_SystemState_sub_substate(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_current_state
{
public:
  explicit Init_SystemState_current_state(::autonomy_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  Init_SystemState_substate current_state(::autonomy_interfaces::msg::SystemState::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_SystemState_substate(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

class Init_SystemState_header
{
public:
  Init_SystemState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemState_current_state header(::autonomy_interfaces::msg::SystemState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemState_current_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::SystemState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::SystemState>()
{
  return autonomy_interfaces::msg::builder::Init_SystemState_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
