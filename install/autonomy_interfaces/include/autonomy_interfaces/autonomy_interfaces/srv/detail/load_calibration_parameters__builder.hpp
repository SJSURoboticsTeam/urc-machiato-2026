// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/LoadCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/load_calibration_parameters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_LoadCalibrationParameters_Request_parameter_namespace
{
public:
  explicit Init_LoadCalibrationParameters_Request_parameter_namespace(::autonomy_interfaces::srv::LoadCalibrationParameters_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Request parameter_namespace(::autonomy_interfaces::srv::LoadCalibrationParameters_Request::_parameter_namespace_type arg)
  {
    msg_.parameter_namespace = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Request msg_;
};

class Init_LoadCalibrationParameters_Request_calibration_file
{
public:
  Init_LoadCalibrationParameters_Request_calibration_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadCalibrationParameters_Request_parameter_namespace calibration_file(::autonomy_interfaces::srv::LoadCalibrationParameters_Request::_calibration_file_type arg)
  {
    msg_.calibration_file = std::move(arg);
    return Init_LoadCalibrationParameters_Request_parameter_namespace(msg_);
  }

private:
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::LoadCalibrationParameters_Request>()
{
  return autonomy_interfaces::srv::builder::Init_LoadCalibrationParameters_Request_calibration_file();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_LoadCalibrationParameters_Response_error_message
{
public:
  explicit Init_LoadCalibrationParameters_Response_error_message(::autonomy_interfaces::srv::LoadCalibrationParameters_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Response error_message(::autonomy_interfaces::srv::LoadCalibrationParameters_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Response msg_;
};

class Init_LoadCalibrationParameters_Response_success
{
public:
  Init_LoadCalibrationParameters_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadCalibrationParameters_Response_error_message success(::autonomy_interfaces::srv::LoadCalibrationParameters_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LoadCalibrationParameters_Response_error_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::LoadCalibrationParameters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::LoadCalibrationParameters_Response>()
{
  return autonomy_interfaces::srv::builder::Init_LoadCalibrationParameters_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__BUILDER_HPP_
