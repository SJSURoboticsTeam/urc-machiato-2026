// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/calibrate_camera.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/calibrate_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_CalibrateCamera_Request_marker_size
{
public:
  explicit Init_CalibrateCamera_Request_marker_size(::autonomy_interfaces::srv::CalibrateCamera_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::CalibrateCamera_Request marker_size(::autonomy_interfaces::srv::CalibrateCamera_Request::_marker_size_type arg)
  {
    msg_.marker_size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

class Init_CalibrateCamera_Request_square_size
{
public:
  explicit Init_CalibrateCamera_Request_square_size(::autonomy_interfaces::srv::CalibrateCamera_Request & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Request_marker_size square_size(::autonomy_interfaces::srv::CalibrateCamera_Request::_square_size_type arg)
  {
    msg_.square_size = std::move(arg);
    return Init_CalibrateCamera_Request_marker_size(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

class Init_CalibrateCamera_Request_squares_y
{
public:
  explicit Init_CalibrateCamera_Request_squares_y(::autonomy_interfaces::srv::CalibrateCamera_Request & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Request_square_size squares_y(::autonomy_interfaces::srv::CalibrateCamera_Request::_squares_y_type arg)
  {
    msg_.squares_y = std::move(arg);
    return Init_CalibrateCamera_Request_square_size(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

class Init_CalibrateCamera_Request_squares_x
{
public:
  explicit Init_CalibrateCamera_Request_squares_x(::autonomy_interfaces::srv::CalibrateCamera_Request & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Request_squares_y squares_x(::autonomy_interfaces::srv::CalibrateCamera_Request::_squares_x_type arg)
  {
    msg_.squares_x = std::move(arg);
    return Init_CalibrateCamera_Request_squares_y(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

class Init_CalibrateCamera_Request_board_type
{
public:
  explicit Init_CalibrateCamera_Request_board_type(::autonomy_interfaces::srv::CalibrateCamera_Request & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Request_squares_x board_type(::autonomy_interfaces::srv::CalibrateCamera_Request::_board_type_type arg)
  {
    msg_.board_type = std::move(arg);
    return Init_CalibrateCamera_Request_squares_x(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

class Init_CalibrateCamera_Request_image_directory
{
public:
  Init_CalibrateCamera_Request_image_directory()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCamera_Request_board_type image_directory(::autonomy_interfaces::srv::CalibrateCamera_Request::_image_directory_type arg)
  {
    msg_.image_directory = std::move(arg);
    return Init_CalibrateCamera_Request_board_type(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::CalibrateCamera_Request>()
{
  return autonomy_interfaces::srv::builder::Init_CalibrateCamera_Request_image_directory();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_CalibrateCamera_Response_error_message
{
public:
  explicit Init_CalibrateCamera_Response_error_message(::autonomy_interfaces::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::CalibrateCamera_Response error_message(::autonomy_interfaces::srv::CalibrateCamera_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_calibration_summary
{
public:
  explicit Init_CalibrateCamera_Response_calibration_summary(::autonomy_interfaces::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Response_error_message calibration_summary(::autonomy_interfaces::srv::CalibrateCamera_Response::_calibration_summary_type arg)
  {
    msg_.calibration_summary = std::move(arg);
    return Init_CalibrateCamera_Response_error_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_result_file
{
public:
  explicit Init_CalibrateCamera_Response_result_file(::autonomy_interfaces::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Response_calibration_summary result_file(::autonomy_interfaces::srv::CalibrateCamera_Response::_result_file_type arg)
  {
    msg_.result_file = std::move(arg);
    return Init_CalibrateCamera_Response_calibration_summary(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_success
{
public:
  Init_CalibrateCamera_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCamera_Response_result_file success(::autonomy_interfaces::srv::CalibrateCamera_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CalibrateCamera_Response_result_file(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::CalibrateCamera_Response>()
{
  return autonomy_interfaces::srv::builder::Init_CalibrateCamera_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_CalibrateCamera_Event_response
{
public:
  explicit Init_CalibrateCamera_Event_response(::autonomy_interfaces::srv::CalibrateCamera_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::CalibrateCamera_Event response(::autonomy_interfaces::srv::CalibrateCamera_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Event msg_;
};

class Init_CalibrateCamera_Event_request
{
public:
  explicit Init_CalibrateCamera_Event_request(::autonomy_interfaces::srv::CalibrateCamera_Event & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Event_response request(::autonomy_interfaces::srv::CalibrateCamera_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CalibrateCamera_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Event msg_;
};

class Init_CalibrateCamera_Event_info
{
public:
  Init_CalibrateCamera_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCamera_Event_request info(::autonomy_interfaces::srv::CalibrateCamera_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CalibrateCamera_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::CalibrateCamera_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::CalibrateCamera_Event>()
{
  return autonomy_interfaces::srv::builder::Init_CalibrateCamera_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_
