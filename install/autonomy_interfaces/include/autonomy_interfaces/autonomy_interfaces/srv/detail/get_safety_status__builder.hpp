// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_safety_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSafetyStatus_Request>()
{
  return ::autonomy_interfaces::srv::GetSafetyStatus_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSafetyStatus_Response_monitoring_stats
{
public:
  explicit Init_GetSafetyStatus_Response_monitoring_stats(::autonomy_interfaces::srv::GetSafetyStatus_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSafetyStatus_Response monitoring_stats(::autonomy_interfaces::srv::GetSafetyStatus_Response::_monitoring_stats_type arg)
  {
    msg_.monitoring_stats = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Response msg_;
};

class Init_GetSafetyStatus_Response_active_alerts
{
public:
  explicit Init_GetSafetyStatus_Response_active_alerts(::autonomy_interfaces::srv::GetSafetyStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetSafetyStatus_Response_monitoring_stats active_alerts(::autonomy_interfaces::srv::GetSafetyStatus_Response::_active_alerts_type arg)
  {
    msg_.active_alerts = std::move(arg);
    return Init_GetSafetyStatus_Response_monitoring_stats(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Response msg_;
};

class Init_GetSafetyStatus_Response_overall_safety
{
public:
  Init_GetSafetyStatus_Response_overall_safety()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSafetyStatus_Response_active_alerts overall_safety(::autonomy_interfaces::srv::GetSafetyStatus_Response::_overall_safety_type arg)
  {
    msg_.overall_safety = std::move(arg);
    return Init_GetSafetyStatus_Response_active_alerts(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSafetyStatus_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetSafetyStatus_Response_overall_safety();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSafetyStatus_Event_response
{
public:
  explicit Init_GetSafetyStatus_Event_response(::autonomy_interfaces::srv::GetSafetyStatus_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetSafetyStatus_Event response(::autonomy_interfaces::srv::GetSafetyStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Event msg_;
};

class Init_GetSafetyStatus_Event_request
{
public:
  explicit Init_GetSafetyStatus_Event_request(::autonomy_interfaces::srv::GetSafetyStatus_Event & msg)
  : msg_(msg)
  {}
  Init_GetSafetyStatus_Event_response request(::autonomy_interfaces::srv::GetSafetyStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetSafetyStatus_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Event msg_;
};

class Init_GetSafetyStatus_Event_info
{
public:
  Init_GetSafetyStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSafetyStatus_Event_request info(::autonomy_interfaces::srv::GetSafetyStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetSafetyStatus_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetSafetyStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetSafetyStatus_Event>()
{
  return autonomy_interfaces::srv::builder::Init_GetSafetyStatus_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__BUILDER_HPP_
