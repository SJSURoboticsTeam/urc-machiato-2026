// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetAdaptationHistory.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_adaptation_history__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetAdaptationHistory_Request_include_context
{
public:
  explicit Init_GetAdaptationHistory_Request_include_context(::autonomy_interfaces::srv::GetAdaptationHistory_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetAdaptationHistory_Request include_context(::autonomy_interfaces::srv::GetAdaptationHistory_Request::_include_context_type arg)
  {
    msg_.include_context = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAdaptationHistory_Request msg_;
};

class Init_GetAdaptationHistory_Request_limit
{
public:
  Init_GetAdaptationHistory_Request_limit()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAdaptationHistory_Request_include_context limit(::autonomy_interfaces::srv::GetAdaptationHistory_Request::_limit_type arg)
  {
    msg_.limit = std::move(arg);
    return Init_GetAdaptationHistory_Request_include_context(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAdaptationHistory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetAdaptationHistory_Request>()
{
  return autonomy_interfaces::srv::builder::Init_GetAdaptationHistory_Request_limit();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetAdaptationHistory_Response_contexts
{
public:
  explicit Init_GetAdaptationHistory_Response_contexts(::autonomy_interfaces::srv::GetAdaptationHistory_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetAdaptationHistory_Response contexts(::autonomy_interfaces::srv::GetAdaptationHistory_Response::_contexts_type arg)
  {
    msg_.contexts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAdaptationHistory_Response msg_;
};

class Init_GetAdaptationHistory_Response_actions
{
public:
  Init_GetAdaptationHistory_Response_actions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAdaptationHistory_Response_contexts actions(::autonomy_interfaces::srv::GetAdaptationHistory_Response::_actions_type arg)
  {
    msg_.actions = std::move(arg);
    return Init_GetAdaptationHistory_Response_contexts(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAdaptationHistory_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetAdaptationHistory_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetAdaptationHistory_Response_actions();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__BUILDER_HPP_
