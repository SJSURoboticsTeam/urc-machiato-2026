// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/change_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChangeState_Request_metadata
{
public:
  explicit Init_ChangeState_Request_metadata(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ChangeState_Request metadata(::autonomy_interfaces::srv::ChangeState_Request::_metadata_type arg)
  {
    msg_.metadata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_force
{
public:
  explicit Init_ChangeState_Request_force(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Request_metadata force(::autonomy_interfaces::srv::ChangeState_Request::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_ChangeState_Request_metadata(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_operator_id
{
public:
  explicit Init_ChangeState_Request_operator_id(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Request_force operator_id(::autonomy_interfaces::srv::ChangeState_Request::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return Init_ChangeState_Request_force(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_reason
{
public:
  explicit Init_ChangeState_Request_reason(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Request_operator_id reason(::autonomy_interfaces::srv::ChangeState_Request::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_ChangeState_Request_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_desired_calibration_substate
{
public:
  explicit Init_ChangeState_Request_desired_calibration_substate(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Request_reason desired_calibration_substate(::autonomy_interfaces::srv::ChangeState_Request::_desired_calibration_substate_type arg)
  {
    msg_.desired_calibration_substate = std::move(arg);
    return Init_ChangeState_Request_reason(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_desired_substate
{
public:
  explicit Init_ChangeState_Request_desired_substate(::autonomy_interfaces::srv::ChangeState_Request & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Request_desired_calibration_substate desired_substate(::autonomy_interfaces::srv::ChangeState_Request::_desired_substate_type arg)
  {
    msg_.desired_substate = std::move(arg);
    return Init_ChangeState_Request_desired_calibration_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

class Init_ChangeState_Request_desired_state
{
public:
  Init_ChangeState_Request_desired_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangeState_Request_desired_substate desired_state(::autonomy_interfaces::srv::ChangeState_Request::_desired_state_type arg)
  {
    msg_.desired_state = std::move(arg);
    return Init_ChangeState_Request_desired_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ChangeState_Request>()
{
  return autonomy_interfaces::srv::builder::Init_ChangeState_Request_desired_state();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChangeState_Response_warnings
{
public:
  explicit Init_ChangeState_Response_warnings(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ChangeState_Response warnings(::autonomy_interfaces::srv::ChangeState_Response::_warnings_type arg)
  {
    msg_.warnings = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_failed_preconditions
{
public:
  explicit Init_ChangeState_Response_failed_preconditions(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_warnings failed_preconditions(::autonomy_interfaces::srv::ChangeState_Response::_failed_preconditions_type arg)
  {
    msg_.failed_preconditions = std::move(arg);
    return Init_ChangeState_Response_warnings(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_preconditions_met
{
public:
  explicit Init_ChangeState_Response_preconditions_met(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_failed_preconditions preconditions_met(::autonomy_interfaces::srv::ChangeState_Response::_preconditions_met_type arg)
  {
    msg_.preconditions_met = std::move(arg);
    return Init_ChangeState_Response_failed_preconditions(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_message
{
public:
  explicit Init_ChangeState_Response_message(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_preconditions_met message(::autonomy_interfaces::srv::ChangeState_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ChangeState_Response_preconditions_met(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_transition_time
{
public:
  explicit Init_ChangeState_Response_transition_time(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_message transition_time(::autonomy_interfaces::srv::ChangeState_Response::_transition_time_type arg)
  {
    msg_.transition_time = std::move(arg);
    return Init_ChangeState_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_actual_calibration_substate
{
public:
  explicit Init_ChangeState_Response_actual_calibration_substate(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_transition_time actual_calibration_substate(::autonomy_interfaces::srv::ChangeState_Response::_actual_calibration_substate_type arg)
  {
    msg_.actual_calibration_substate = std::move(arg);
    return Init_ChangeState_Response_transition_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_actual_substate
{
public:
  explicit Init_ChangeState_Response_actual_substate(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_actual_calibration_substate actual_substate(::autonomy_interfaces::srv::ChangeState_Response::_actual_substate_type arg)
  {
    msg_.actual_substate = std::move(arg);
    return Init_ChangeState_Response_actual_calibration_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_actual_state
{
public:
  explicit Init_ChangeState_Response_actual_state(::autonomy_interfaces::srv::ChangeState_Response & msg)
  : msg_(msg)
  {}
  Init_ChangeState_Response_actual_substate actual_state(::autonomy_interfaces::srv::ChangeState_Response::_actual_state_type arg)
  {
    msg_.actual_state = std::move(arg);
    return Init_ChangeState_Response_actual_substate(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

class Init_ChangeState_Response_success
{
public:
  Init_ChangeState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangeState_Response_actual_state success(::autonomy_interfaces::srv::ChangeState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ChangeState_Response_actual_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::ChangeState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ChangeState_Response>()
{
  return autonomy_interfaces::srv::builder::Init_ChangeState_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_
