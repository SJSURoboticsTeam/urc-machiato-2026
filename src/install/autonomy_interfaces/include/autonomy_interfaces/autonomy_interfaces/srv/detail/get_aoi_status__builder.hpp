// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_aoi_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetAOIStatus_Request_history_samples
{
public:
  explicit Init_GetAOIStatus_Request_history_samples(::autonomy_interfaces::srv::GetAOIStatus_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetAOIStatus_Request history_samples(::autonomy_interfaces::srv::GetAOIStatus_Request::_history_samples_type arg)
  {
    msg_.history_samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Request msg_;
};

class Init_GetAOIStatus_Request_include_history
{
public:
  explicit Init_GetAOIStatus_Request_include_history(::autonomy_interfaces::srv::GetAOIStatus_Request & msg)
  : msg_(msg)
  {}
  Init_GetAOIStatus_Request_history_samples include_history(::autonomy_interfaces::srv::GetAOIStatus_Request::_include_history_type arg)
  {
    msg_.include_history = std::move(arg);
    return Init_GetAOIStatus_Request_history_samples(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Request msg_;
};

class Init_GetAOIStatus_Request_sensor_name
{
public:
  Init_GetAOIStatus_Request_sensor_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAOIStatus_Request_include_history sensor_name(::autonomy_interfaces::srv::GetAOIStatus_Request::_sensor_name_type arg)
  {
    msg_.sensor_name = std::move(arg);
    return Init_GetAOIStatus_Request_include_history(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetAOIStatus_Request>()
{
  return autonomy_interfaces::srv::builder::Init_GetAOIStatus_Request_sensor_name();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetAOIStatus_Response_system_metrics
{
public:
  explicit Init_GetAOIStatus_Response_system_metrics(::autonomy_interfaces::srv::GetAOIStatus_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetAOIStatus_Response system_metrics(::autonomy_interfaces::srv::GetAOIStatus_Response::_system_metrics_type arg)
  {
    msg_.system_metrics = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

class Init_GetAOIStatus_Response_timestamp_history
{
public:
  explicit Init_GetAOIStatus_Response_timestamp_history(::autonomy_interfaces::srv::GetAOIStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetAOIStatus_Response_system_metrics timestamp_history(::autonomy_interfaces::srv::GetAOIStatus_Response::_timestamp_history_type arg)
  {
    msg_.timestamp_history = std::move(arg);
    return Init_GetAOIStatus_Response_system_metrics(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

class Init_GetAOIStatus_Response_aoi_history
{
public:
  explicit Init_GetAOIStatus_Response_aoi_history(::autonomy_interfaces::srv::GetAOIStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetAOIStatus_Response_timestamp_history aoi_history(::autonomy_interfaces::srv::GetAOIStatus_Response::_aoi_history_type arg)
  {
    msg_.aoi_history = std::move(arg);
    return Init_GetAOIStatus_Response_timestamp_history(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

class Init_GetAOIStatus_Response_sensor_status
{
public:
  explicit Init_GetAOIStatus_Response_sensor_status(::autonomy_interfaces::srv::GetAOIStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetAOIStatus_Response_aoi_history sensor_status(::autonomy_interfaces::srv::GetAOIStatus_Response::_sensor_status_type arg)
  {
    msg_.sensor_status = std::move(arg);
    return Init_GetAOIStatus_Response_aoi_history(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

class Init_GetAOIStatus_Response_message
{
public:
  explicit Init_GetAOIStatus_Response_message(::autonomy_interfaces::srv::GetAOIStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetAOIStatus_Response_sensor_status message(::autonomy_interfaces::srv::GetAOIStatus_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetAOIStatus_Response_sensor_status(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

class Init_GetAOIStatus_Response_success
{
public:
  Init_GetAOIStatus_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAOIStatus_Response_message success(::autonomy_interfaces::srv::GetAOIStatus_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetAOIStatus_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetAOIStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetAOIStatus_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetAOIStatus_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__BUILDER_HPP_
