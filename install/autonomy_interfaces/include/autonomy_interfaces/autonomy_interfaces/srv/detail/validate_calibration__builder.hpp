// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/validate_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ValidateCalibration_Request_test_images
{
public:
  explicit Init_ValidateCalibration_Request_test_images(::autonomy_interfaces::srv::ValidateCalibration_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ValidateCalibration_Request test_images(::autonomy_interfaces::srv::ValidateCalibration_Request::_test_images_type arg)
  {
    msg_.test_images = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Request msg_;
};

class Init_ValidateCalibration_Request_calibration_file
{
public:
  Init_ValidateCalibration_Request_calibration_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ValidateCalibration_Request_test_images calibration_file(::autonomy_interfaces::srv::ValidateCalibration_Request::_calibration_file_type arg)
  {
    msg_.calibration_file = std::move(arg);
    return Init_ValidateCalibration_Request_test_images(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ValidateCalibration_Request>()
{
  return autonomy_interfaces::srv::builder::Init_ValidateCalibration_Request_calibration_file();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ValidateCalibration_Response_error_message
{
public:
  explicit Init_ValidateCalibration_Response_error_message(::autonomy_interfaces::srv::ValidateCalibration_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ValidateCalibration_Response error_message(::autonomy_interfaces::srv::ValidateCalibration_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Response msg_;
};

class Init_ValidateCalibration_Response_recommendations
{
public:
  explicit Init_ValidateCalibration_Response_recommendations(::autonomy_interfaces::srv::ValidateCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_ValidateCalibration_Response_error_message recommendations(::autonomy_interfaces::srv::ValidateCalibration_Response::_recommendations_type arg)
  {
    msg_.recommendations = std::move(arg);
    return Init_ValidateCalibration_Response_error_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Response msg_;
};

class Init_ValidateCalibration_Response_quality_assessment
{
public:
  explicit Init_ValidateCalibration_Response_quality_assessment(::autonomy_interfaces::srv::ValidateCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_ValidateCalibration_Response_recommendations quality_assessment(::autonomy_interfaces::srv::ValidateCalibration_Response::_quality_assessment_type arg)
  {
    msg_.quality_assessment = std::move(arg);
    return Init_ValidateCalibration_Response_recommendations(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Response msg_;
};

class Init_ValidateCalibration_Response_reprojection_error
{
public:
  explicit Init_ValidateCalibration_Response_reprojection_error(::autonomy_interfaces::srv::ValidateCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_ValidateCalibration_Response_quality_assessment reprojection_error(::autonomy_interfaces::srv::ValidateCalibration_Response::_reprojection_error_type arg)
  {
    msg_.reprojection_error = std::move(arg);
    return Init_ValidateCalibration_Response_quality_assessment(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Response msg_;
};

class Init_ValidateCalibration_Response_success
{
public:
  Init_ValidateCalibration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ValidateCalibration_Response_reprojection_error success(::autonomy_interfaces::srv::ValidateCalibration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ValidateCalibration_Response_reprojection_error(msg_);
  }

private:
  ::autonomy_interfaces::srv::ValidateCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ValidateCalibration_Response>()
{
  return autonomy_interfaces::srv::builder::Init_ValidateCalibration_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__BUILDER_HPP_
