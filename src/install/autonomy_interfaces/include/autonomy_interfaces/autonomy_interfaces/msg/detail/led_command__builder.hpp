// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/led_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_LedCommand_override
{
public:
  explicit Init_LedCommand_override(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::LedCommand override(::autonomy_interfaces::msg::LedCommand::_override_type arg)
  {
    msg_.override = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_duration
{
public:
  explicit Init_LedCommand_duration(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_override duration(::autonomy_interfaces::msg::LedCommand::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_LedCommand_override(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_priority
{
public:
  explicit Init_LedCommand_priority(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_duration priority(::autonomy_interfaces::msg::LedCommand::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_LedCommand_duration(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_frequency
{
public:
  explicit Init_LedCommand_frequency(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_priority frequency(::autonomy_interfaces::msg::LedCommand::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_LedCommand_priority(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_pattern
{
public:
  explicit Init_LedCommand_pattern(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_frequency pattern(::autonomy_interfaces::msg::LedCommand::_pattern_type arg)
  {
    msg_.pattern = std::move(arg);
    return Init_LedCommand_frequency(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_blue
{
public:
  explicit Init_LedCommand_blue(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_pattern blue(::autonomy_interfaces::msg::LedCommand::_blue_type arg)
  {
    msg_.blue = std::move(arg);
    return Init_LedCommand_pattern(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_green
{
public:
  explicit Init_LedCommand_green(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_blue green(::autonomy_interfaces::msg::LedCommand::_green_type arg)
  {
    msg_.green = std::move(arg);
    return Init_LedCommand_blue(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_red
{
public:
  explicit Init_LedCommand_red(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_green red(::autonomy_interfaces::msg::LedCommand::_red_type arg)
  {
    msg_.red = std::move(arg);
    return Init_LedCommand_green(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_status_code
{
public:
  explicit Init_LedCommand_status_code(::autonomy_interfaces::msg::LedCommand & msg)
  : msg_(msg)
  {}
  Init_LedCommand_red status_code(::autonomy_interfaces::msg::LedCommand::_status_code_type arg)
  {
    msg_.status_code = std::move(arg);
    return Init_LedCommand_red(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

class Init_LedCommand_header
{
public:
  Init_LedCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LedCommand_status_code header(::autonomy_interfaces::msg::LedCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LedCommand_status_code(msg_);
  }

private:
  ::autonomy_interfaces::msg::LedCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::LedCommand>()
{
  return autonomy_interfaces::msg::builder::Init_LedCommand_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_
