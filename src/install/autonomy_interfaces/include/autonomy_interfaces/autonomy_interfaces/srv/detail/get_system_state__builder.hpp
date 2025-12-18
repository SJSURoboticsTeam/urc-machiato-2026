// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_system_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSystemState_Request_history_limit
{
public:
  explicit Init_GetSystemState_Request_history_limit(::autonomy_interfaces::srv::GetSystemState_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSystemState_Request history_limit(::autonomy_interfaces::srv::GetSystemState_Request::_history_limit_type arg)
  {
    msg_.history_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Request msg_;
};

class Init_GetSystemState_Request_include_subsystems
{
public:
  explicit Init_GetSystemState_Request_include_subsystems(::autonomy_interfaces::srv::GetSystemState_Request & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Request_history_limit include_subsystems(::autonomy_interfaces::srv::GetSystemState_Request::_include_subsystems_type arg)
  {
    msg_.include_subsystems = std::move(arg);
    return Init_GetSystemState_Request_history_limit(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Request msg_;
};

class Init_GetSystemState_Request_include_history
{
public:
  Init_GetSystemState_Request_include_history()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSystemState_Request_include_subsystems include_history(::autonomy_interfaces::srv::GetSystemState_Request::_include_history_type arg)
  {
    msg_.include_history = std::move(arg);
    return Init_GetSystemState_Request_include_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSystemState_Request>()
{
  return autonomy_interfaces::srv::builder::Init_GetSystemState_Request_include_history();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSystemState_Response_failed_subsystems
{
public:
  explicit Init_GetSystemState_Response_failed_subsystems(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSystemState_Response failed_subsystems(::autonomy_interfaces::srv::GetSystemState_Response::_failed_subsystems_type arg)
  {
    msg_.failed_subsystems = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_inactive_subsystems
{
public:
  explicit Init_GetSystemState_Response_inactive_subsystems(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_failed_subsystems inactive_subsystems(::autonomy_interfaces::srv::GetSystemState_Response::_inactive_subsystems_type arg)
  {
    msg_.inactive_subsystems = std::move(arg);
    return Init_GetSystemState_Response_failed_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_active_subsystems
{
public:
  explicit Init_GetSystemState_Response_active_subsystems(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_inactive_subsystems active_subsystems(::autonomy_interfaces::srv::GetSystemState_Response::_active_subsystems_type arg)
  {
    msg_.active_subsystems = std::move(arg);
    return Init_GetSystemState_Response_inactive_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_transition_reasons
{
public:
  explicit Init_GetSystemState_Response_transition_reasons(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_active_subsystems transition_reasons(::autonomy_interfaces::srv::GetSystemState_Response::_transition_reasons_type arg)
  {
    msg_.transition_reasons = std::move(arg);
    return Init_GetSystemState_Response_active_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_state_timestamps
{
public:
  explicit Init_GetSystemState_Response_state_timestamps(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_transition_reasons state_timestamps(::autonomy_interfaces::srv::GetSystemState_Response::_state_timestamps_type arg)
  {
    msg_.state_timestamps = std::move(arg);
    return Init_GetSystemState_Response_transition_reasons(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_recent_states
{
public:
  explicit Init_GetSystemState_Response_recent_states(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_state_timestamps recent_states(::autonomy_interfaces::srv::GetSystemState_Response::_recent_states_type arg)
  {
    msg_.recent_states = std::move(arg);
    return Init_GetSystemState_Response_state_timestamps(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_state_entered
{
public:
  explicit Init_GetSystemState_Response_state_entered(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_recent_states state_entered(::autonomy_interfaces::srv::GetSystemState_Response::_state_entered_type arg)
  {
    msg_.state_entered = std::move(arg);
    return Init_GetSystemState_Response_recent_states(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_time_in_state
{
public:
  explicit Init_GetSystemState_Response_time_in_state(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_state_entered time_in_state(::autonomy_interfaces::srv::GetSystemState_Response::_time_in_state_type arg)
  {
    msg_.time_in_state = std::move(arg);
    return Init_GetSystemState_Response_state_entered(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_sub_substate
{
public:
  explicit Init_GetSystemState_Response_sub_substate(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_time_in_state sub_substate(::autonomy_interfaces::srv::GetSystemState_Response::_sub_substate_type arg)
  {
    msg_.sub_substate = std::move(arg);
    return Init_GetSystemState_Response_time_in_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_substate
{
public:
  explicit Init_GetSystemState_Response_substate(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_sub_substate substate(::autonomy_interfaces::srv::GetSystemState_Response::_substate_type arg)
  {
    msg_.substate = std::move(arg);
    return Init_GetSystemState_Response_sub_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_current_state
{
public:
  explicit Init_GetSystemState_Response_current_state(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_substate current_state(::autonomy_interfaces::srv::GetSystemState_Response::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_GetSystemState_Response_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_message
{
public:
  explicit Init_GetSystemState_Response_message(::autonomy_interfaces::srv::GetSystemState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSystemState_Response_current_state message(::autonomy_interfaces::srv::GetSystemState_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetSystemState_Response_current_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

class Init_GetSystemState_Response_success
{
public:
  Init_GetSystemState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSystemState_Response_message success(::autonomy_interfaces::srv::GetSystemState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetSystemState_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSystemState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSystemState_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetSystemState_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__BUILDER_HPP_
