// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/state_transition.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/state_transition__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_StateTransition_exit_actions_executed
{
public:
  explicit Init_StateTransition_exit_actions_executed(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::StateTransition exit_actions_executed(::autonomy_interfaces::msg::StateTransition::_exit_actions_executed_type arg)
  {
    msg_.exit_actions_executed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_entry_actions_executed
{
public:
  explicit Init_StateTransition_entry_actions_executed(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_exit_actions_executed entry_actions_executed(::autonomy_interfaces::msg::StateTransition::_entry_actions_executed_type arg)
  {
    msg_.entry_actions_executed = std::move(arg);
    return Init_StateTransition_exit_actions_executed(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_preconditions_checked
{
public:
  explicit Init_StateTransition_preconditions_checked(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_entry_actions_executed preconditions_checked(::autonomy_interfaces::msg::StateTransition::_preconditions_checked_type arg)
  {
    msg_.preconditions_checked = std::move(arg);
    return Init_StateTransition_entry_actions_executed(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_failure_reason
{
public:
  explicit Init_StateTransition_failure_reason(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_preconditions_checked failure_reason(::autonomy_interfaces::msg::StateTransition::_failure_reason_type arg)
  {
    msg_.failure_reason = std::move(arg);
    return Init_StateTransition_preconditions_checked(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_initiated_by
{
public:
  explicit Init_StateTransition_initiated_by(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_failure_reason initiated_by(::autonomy_interfaces::msg::StateTransition::_initiated_by_type arg)
  {
    msg_.initiated_by = std::move(arg);
    return Init_StateTransition_failure_reason(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_reason
{
public:
  explicit Init_StateTransition_reason(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_initiated_by reason(::autonomy_interfaces::msg::StateTransition::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_StateTransition_initiated_by(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_success
{
public:
  explicit Init_StateTransition_success(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_reason success(::autonomy_interfaces::msg::StateTransition::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StateTransition_reason(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_transition_duration
{
public:
  explicit Init_StateTransition_transition_duration(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_success transition_duration(::autonomy_interfaces::msg::StateTransition::_transition_duration_type arg)
  {
    msg_.transition_duration = std::move(arg);
    return Init_StateTransition_success(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_end_time
{
public:
  explicit Init_StateTransition_end_time(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_transition_duration end_time(::autonomy_interfaces::msg::StateTransition::_end_time_type arg)
  {
    msg_.end_time = std::move(arg);
    return Init_StateTransition_transition_duration(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_start_time
{
public:
  explicit Init_StateTransition_start_time(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_end_time start_time(::autonomy_interfaces::msg::StateTransition::_start_time_type arg)
  {
    msg_.start_time = std::move(arg);
    return Init_StateTransition_end_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_to_state
{
public:
  explicit Init_StateTransition_to_state(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_start_time to_state(::autonomy_interfaces::msg::StateTransition::_to_state_type arg)
  {
    msg_.to_state = std::move(arg);
    return Init_StateTransition_start_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_from_state
{
public:
  explicit Init_StateTransition_from_state(::autonomy_interfaces::msg::StateTransition & msg)
  : msg_(msg)
  {}
  Init_StateTransition_to_state from_state(::autonomy_interfaces::msg::StateTransition::_from_state_type arg)
  {
    msg_.from_state = std::move(arg);
    return Init_StateTransition_to_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

class Init_StateTransition_header
{
public:
  Init_StateTransition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateTransition_from_state header(::autonomy_interfaces::msg::StateTransition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StateTransition_from_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::StateTransition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::StateTransition>()
{
  return autonomy_interfaces::msg::builder::Init_StateTransition_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__BUILDER_HPP_
