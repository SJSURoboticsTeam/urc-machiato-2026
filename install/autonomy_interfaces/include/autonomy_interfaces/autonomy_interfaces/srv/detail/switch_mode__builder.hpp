// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/SwitchMode.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/switch_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SwitchMode_Request_reason
{
public:
  explicit Init_SwitchMode_Request_reason(::autonomy_interfaces::srv::SwitchMode_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SwitchMode_Request reason(::autonomy_interfaces::srv::SwitchMode_Request::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Request msg_;
};

class Init_SwitchMode_Request_requested_mode
{
public:
  Init_SwitchMode_Request_requested_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwitchMode_Request_reason requested_mode(::autonomy_interfaces::srv::SwitchMode_Request::_requested_mode_type arg)
  {
    msg_.requested_mode = std::move(arg);
    return Init_SwitchMode_Request_reason(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SwitchMode_Request>()
{
  return autonomy_interfaces::srv::builder::Init_SwitchMode_Request_requested_mode();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SwitchMode_Response_transition_time
{
public:
  explicit Init_SwitchMode_Response_transition_time(::autonomy_interfaces::srv::SwitchMode_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SwitchMode_Response transition_time(::autonomy_interfaces::srv::SwitchMode_Response::_transition_time_type arg)
  {
    msg_.transition_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Response msg_;
};

class Init_SwitchMode_Response_actual_mode
{
public:
  explicit Init_SwitchMode_Response_actual_mode(::autonomy_interfaces::srv::SwitchMode_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchMode_Response_transition_time actual_mode(::autonomy_interfaces::srv::SwitchMode_Response::_actual_mode_type arg)
  {
    msg_.actual_mode = std::move(arg);
    return Init_SwitchMode_Response_transition_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Response msg_;
};

class Init_SwitchMode_Response_message
{
public:
  explicit Init_SwitchMode_Response_message(::autonomy_interfaces::srv::SwitchMode_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchMode_Response_actual_mode message(::autonomy_interfaces::srv::SwitchMode_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SwitchMode_Response_actual_mode(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Response msg_;
};

class Init_SwitchMode_Response_success
{
public:
  Init_SwitchMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwitchMode_Response_message success(::autonomy_interfaces::srv::SwitchMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SwitchMode_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::SwitchMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SwitchMode_Response>()
{
  return autonomy_interfaces::srv::builder::Init_SwitchMode_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__BUILDER_HPP_
