// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetSubsystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_subsystem_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_subsystem_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSubsystemStatus_Request_subsystem_name
{
public:
  Init_GetSubsystemStatus_Request_subsystem_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::srv::GetSubsystemStatus_Request subsystem_name(::autonomy_interfaces::srv::GetSubsystemStatus_Request::_subsystem_name_type arg)
  {
    msg_.subsystem_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSubsystemStatus_Request>()
{
  return autonomy_interfaces::srv::builder::Init_GetSubsystemStatus_Request_subsystem_name();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSubsystemStatus_Response_status_messages
{
public:
  explicit Init_GetSubsystemStatus_Response_status_messages(::autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response status_messages(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_status_messages_type arg)
  {
    msg_.status_messages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

class Init_GetSubsystemStatus_Response_subsystem_health
{
public:
  explicit Init_GetSubsystemStatus_Response_subsystem_health(::autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetSubsystemStatus_Response_status_messages subsystem_health(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_subsystem_health_type arg)
  {
    msg_.subsystem_health = std::move(arg);
    return Init_GetSubsystemStatus_Response_status_messages(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

class Init_GetSubsystemStatus_Response_subsystem_states
{
public:
  explicit Init_GetSubsystemStatus_Response_subsystem_states(::autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetSubsystemStatus_Response_subsystem_health subsystem_states(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_subsystem_states_type arg)
  {
    msg_.subsystem_states = std::move(arg);
    return Init_GetSubsystemStatus_Response_subsystem_health(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

class Init_GetSubsystemStatus_Response_subsystem_names
{
public:
  explicit Init_GetSubsystemStatus_Response_subsystem_names(::autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetSubsystemStatus_Response_subsystem_states subsystem_names(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_subsystem_names_type arg)
  {
    msg_.subsystem_names = std::move(arg);
    return Init_GetSubsystemStatus_Response_subsystem_states(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

class Init_GetSubsystemStatus_Response_error_message
{
public:
  explicit Init_GetSubsystemStatus_Response_error_message(::autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetSubsystemStatus_Response_subsystem_names error_message(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_GetSubsystemStatus_Response_subsystem_names(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

class Init_GetSubsystemStatus_Response_success
{
public:
  Init_GetSubsystemStatus_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSubsystemStatus_Response_error_message success(::autonomy_interfaces::srv::GetSubsystemStatus_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetSubsystemStatus_Response_error_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSubsystemStatus_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetSubsystemStatus_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSubsystemStatus_Event_response
{
public:
  explicit Init_GetSubsystemStatus_Event_response(::autonomy_interfaces::srv::GetSubsystemStatus_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSubsystemStatus_Event response(::autonomy_interfaces::srv::GetSubsystemStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Event msg_;
};

class Init_GetSubsystemStatus_Event_request
{
public:
  explicit Init_GetSubsystemStatus_Event_request(::autonomy_interfaces::srv::GetSubsystemStatus_Event & msg)
  : msg_(msg)
  {}
  Init_GetSubsystemStatus_Event_response request(::autonomy_interfaces::srv::GetSubsystemStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetSubsystemStatus_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Event msg_;
};

class Init_GetSubsystemStatus_Event_info
{
public:
  Init_GetSubsystemStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSubsystemStatus_Event_request info(::autonomy_interfaces::srv::GetSubsystemStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetSubsystemStatus_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSubsystemStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSubsystemStatus_Event>()
{
  return autonomy_interfaces::srv::builder::Init_GetSubsystemStatus_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__BUILDER_HPP_
