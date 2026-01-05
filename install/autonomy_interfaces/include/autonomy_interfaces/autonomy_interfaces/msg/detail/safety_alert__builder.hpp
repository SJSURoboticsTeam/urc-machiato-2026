// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_alert.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/safety_alert__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_SafetyAlert_acknowledged
{
public:
  explicit Init_SafetyAlert_acknowledged(::autonomy_interfaces::msg::SafetyAlert & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::SafetyAlert acknowledged(::autonomy_interfaces::msg::SafetyAlert::_acknowledged_type arg)
  {
    msg_.acknowledged = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyAlert msg_;
};

class Init_SafetyAlert_timestamp
{
public:
  explicit Init_SafetyAlert_timestamp(::autonomy_interfaces::msg::SafetyAlert & msg)
  : msg_(msg)
  {}
  Init_SafetyAlert_acknowledged timestamp(::autonomy_interfaces::msg::SafetyAlert::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SafetyAlert_acknowledged(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyAlert msg_;
};

class Init_SafetyAlert_details
{
public:
  explicit Init_SafetyAlert_details(::autonomy_interfaces::msg::SafetyAlert & msg)
  : msg_(msg)
  {}
  Init_SafetyAlert_timestamp details(::autonomy_interfaces::msg::SafetyAlert::_details_type arg)
  {
    msg_.details = std::move(arg);
    return Init_SafetyAlert_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyAlert msg_;
};

class Init_SafetyAlert_severity
{
public:
  explicit Init_SafetyAlert_severity(::autonomy_interfaces::msg::SafetyAlert & msg)
  : msg_(msg)
  {}
  Init_SafetyAlert_details severity(::autonomy_interfaces::msg::SafetyAlert::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_SafetyAlert_details(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyAlert msg_;
};

class Init_SafetyAlert_property
{
public:
  Init_SafetyAlert_property()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafetyAlert_severity property(::autonomy_interfaces::msg::SafetyAlert::_property_type arg)
  {
    msg_.property = std::move(arg);
    return Init_SafetyAlert_severity(msg_);
  }

private:
  ::autonomy_interfaces::msg::SafetyAlert msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::SafetyAlert>()
{
  return autonomy_interfaces::msg::builder::Init_SafetyAlert_property();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__BUILDER_HPP_
