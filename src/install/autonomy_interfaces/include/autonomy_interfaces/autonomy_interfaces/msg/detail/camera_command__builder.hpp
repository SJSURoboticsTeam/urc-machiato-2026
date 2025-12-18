// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/camera_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_CameraCommand_timeout
{
public:
  explicit Init_CameraCommand_timeout(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::CameraCommand timeout(::autonomy_interfaces::msg::CameraCommand::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_priority
{
public:
  explicit Init_CameraCommand_priority(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_timeout priority(::autonomy_interfaces::msg::CameraCommand::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_CameraCommand_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_max_tilt_speed
{
public:
  explicit Init_CameraCommand_max_tilt_speed(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_priority max_tilt_speed(::autonomy_interfaces::msg::CameraCommand::_max_tilt_speed_type arg)
  {
    msg_.max_tilt_speed = std::move(arg);
    return Init_CameraCommand_priority(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_max_pan_speed
{
public:
  explicit Init_CameraCommand_max_pan_speed(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_max_tilt_speed max_pan_speed(::autonomy_interfaces::msg::CameraCommand::_max_pan_speed_type arg)
  {
    msg_.max_pan_speed = std::move(arg);
    return Init_CameraCommand_max_tilt_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_scan_range
{
public:
  explicit Init_CameraCommand_scan_range(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_max_pan_speed scan_range(::autonomy_interfaces::msg::CameraCommand::_scan_range_type arg)
  {
    msg_.scan_range = std::move(arg);
    return Init_CameraCommand_max_pan_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_scan_speed
{
public:
  explicit Init_CameraCommand_scan_speed(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_scan_range scan_speed(::autonomy_interfaces::msg::CameraCommand::_scan_speed_type arg)
  {
    msg_.scan_speed = std::move(arg);
    return Init_CameraCommand_scan_range(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_scan_pattern
{
public:
  explicit Init_CameraCommand_scan_pattern(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_scan_speed scan_pattern(::autonomy_interfaces::msg::CameraCommand::_scan_pattern_type arg)
  {
    msg_.scan_pattern = std::move(arg);
    return Init_CameraCommand_scan_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_tracking_timeout
{
public:
  explicit Init_CameraCommand_tracking_timeout(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_scan_pattern tracking_timeout(::autonomy_interfaces::msg::CameraCommand::_tracking_timeout_type arg)
  {
    msg_.tracking_timeout = std::move(arg);
    return Init_CameraCommand_scan_pattern(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_target_position
{
public:
  explicit Init_CameraCommand_target_position(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_tracking_timeout target_position(::autonomy_interfaces::msg::CameraCommand::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_CameraCommand_tracking_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_autofocus
{
public:
  explicit Init_CameraCommand_autofocus(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_target_position autofocus(::autonomy_interfaces::msg::CameraCommand::_autofocus_type arg)
  {
    msg_.autofocus = std::move(arg);
    return Init_CameraCommand_target_position(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_zoom_level
{
public:
  explicit Init_CameraCommand_zoom_level(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_autofocus zoom_level(::autonomy_interfaces::msg::CameraCommand::_zoom_level_type arg)
  {
    msg_.zoom_level = std::move(arg);
    return Init_CameraCommand_autofocus(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_tilt_speed
{
public:
  explicit Init_CameraCommand_tilt_speed(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_zoom_level tilt_speed(::autonomy_interfaces::msg::CameraCommand::_tilt_speed_type arg)
  {
    msg_.tilt_speed = std::move(arg);
    return Init_CameraCommand_zoom_level(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_pan_speed
{
public:
  explicit Init_CameraCommand_pan_speed(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_tilt_speed pan_speed(::autonomy_interfaces::msg::CameraCommand::_pan_speed_type arg)
  {
    msg_.pan_speed = std::move(arg);
    return Init_CameraCommand_tilt_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_tilt_angle
{
public:
  explicit Init_CameraCommand_tilt_angle(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_pan_speed tilt_angle(::autonomy_interfaces::msg::CameraCommand::_tilt_angle_type arg)
  {
    msg_.tilt_angle = std::move(arg);
    return Init_CameraCommand_pan_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_pan_angle
{
public:
  explicit Init_CameraCommand_pan_angle(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_tilt_angle pan_angle(::autonomy_interfaces::msg::CameraCommand::_pan_angle_type arg)
  {
    msg_.pan_angle = std::move(arg);
    return Init_CameraCommand_tilt_angle(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_command_type
{
public:
  explicit Init_CameraCommand_command_type(::autonomy_interfaces::msg::CameraCommand & msg)
  : msg_(msg)
  {}
  Init_CameraCommand_pan_angle command_type(::autonomy_interfaces::msg::CameraCommand::_command_type_type arg)
  {
    msg_.command_type = std::move(arg);
    return Init_CameraCommand_pan_angle(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

class Init_CameraCommand_header
{
public:
  Init_CameraCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraCommand_command_type header(::autonomy_interfaces::msg::CameraCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CameraCommand_command_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::CameraCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::CameraCommand>()
{
  return autonomy_interfaces::msg::builder::Init_CameraCommand_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__BUILDER_HPP_
