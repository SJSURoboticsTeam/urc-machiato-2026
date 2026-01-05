// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/SafestopControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/safestop_control.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/safestop_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SafestopControl_Request_affected_subsystems
{
public:
  explicit Init_SafestopControl_Request_affected_subsystems(::autonomy_interfaces::srv::SafestopControl_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SafestopControl_Request affected_subsystems(::autonomy_interfaces::srv::SafestopControl_Request::_affected_subsystems_type arg)
  {
    msg_.affected_subsystems = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Request msg_;
};

class Init_SafestopControl_Request_reason
{
public:
  explicit Init_SafestopControl_Request_reason(::autonomy_interfaces::srv::SafestopControl_Request & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Request_affected_subsystems reason(::autonomy_interfaces::srv::SafestopControl_Request::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_SafestopControl_Request_affected_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Request msg_;
};

class Init_SafestopControl_Request_acknowledge_risks
{
public:
  explicit Init_SafestopControl_Request_acknowledge_risks(::autonomy_interfaces::srv::SafestopControl_Request & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Request_reason acknowledge_risks(::autonomy_interfaces::srv::SafestopControl_Request::_acknowledge_risks_type arg)
  {
    msg_.acknowledge_risks = std::move(arg);
    return Init_SafestopControl_Request_reason(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Request msg_;
};

class Init_SafestopControl_Request_operator_id
{
public:
  explicit Init_SafestopControl_Request_operator_id(::autonomy_interfaces::srv::SafestopControl_Request & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Request_acknowledge_risks operator_id(::autonomy_interfaces::srv::SafestopControl_Request::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return Init_SafestopControl_Request_acknowledge_risks(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Request msg_;
};

class Init_SafestopControl_Request_command
{
public:
  Init_SafestopControl_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafestopControl_Request_operator_id command(::autonomy_interfaces::srv::SafestopControl_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_SafestopControl_Request_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SafestopControl_Request>()
{
  return autonomy_interfaces::srv::builder::Init_SafestopControl_Request_command();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SafestopControl_Response_operator_id
{
public:
  explicit Init_SafestopControl_Response_operator_id(::autonomy_interfaces::srv::SafestopControl_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SafestopControl_Response operator_id(::autonomy_interfaces::srv::SafestopControl_Response::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

class Init_SafestopControl_Response_timestamp
{
public:
  explicit Init_SafestopControl_Response_timestamp(::autonomy_interfaces::srv::SafestopControl_Response & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Response_operator_id timestamp(::autonomy_interfaces::srv::SafestopControl_Response::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SafestopControl_Response_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

class Init_SafestopControl_Response_affected_subsystems
{
public:
  explicit Init_SafestopControl_Response_affected_subsystems(::autonomy_interfaces::srv::SafestopControl_Response & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Response_timestamp affected_subsystems(::autonomy_interfaces::srv::SafestopControl_Response::_affected_subsystems_type arg)
  {
    msg_.affected_subsystems = std::move(arg);
    return Init_SafestopControl_Response_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

class Init_SafestopControl_Response_current_state
{
public:
  explicit Init_SafestopControl_Response_current_state(::autonomy_interfaces::srv::SafestopControl_Response & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Response_affected_subsystems current_state(::autonomy_interfaces::srv::SafestopControl_Response::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_SafestopControl_Response_affected_subsystems(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

class Init_SafestopControl_Response_message
{
public:
  explicit Init_SafestopControl_Response_message(::autonomy_interfaces::srv::SafestopControl_Response & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Response_current_state message(::autonomy_interfaces::srv::SafestopControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SafestopControl_Response_current_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

class Init_SafestopControl_Response_success
{
public:
  Init_SafestopControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafestopControl_Response_message success(::autonomy_interfaces::srv::SafestopControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SafestopControl_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SafestopControl_Response>()
{
  return autonomy_interfaces::srv::builder::Init_SafestopControl_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SafestopControl_Event_response
{
public:
  explicit Init_SafestopControl_Event_response(::autonomy_interfaces::srv::SafestopControl_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SafestopControl_Event response(::autonomy_interfaces::srv::SafestopControl_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Event msg_;
};

class Init_SafestopControl_Event_request
{
public:
  explicit Init_SafestopControl_Event_request(::autonomy_interfaces::srv::SafestopControl_Event & msg)
  : msg_(msg)
  {}
  Init_SafestopControl_Event_response request(::autonomy_interfaces::srv::SafestopControl_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SafestopControl_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Event msg_;
};

class Init_SafestopControl_Event_info
{
public:
  Init_SafestopControl_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafestopControl_Event_request info(::autonomy_interfaces::srv::SafestopControl_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SafestopControl_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::SafestopControl_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SafestopControl_Event>()
{
  return autonomy_interfaces::srv::builder::Init_SafestopControl_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__BUILDER_HPP_
