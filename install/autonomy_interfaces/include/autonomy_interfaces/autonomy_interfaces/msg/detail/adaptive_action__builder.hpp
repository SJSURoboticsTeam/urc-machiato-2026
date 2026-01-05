// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/adaptive_action.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/adaptive_action__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_AdaptiveAction_timestamp
{
public:
  explicit Init_AdaptiveAction_timestamp(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::AdaptiveAction timestamp(::autonomy_interfaces::msg::AdaptiveAction::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_success_criteria
{
public:
  explicit Init_AdaptiveAction_success_criteria(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  Init_AdaptiveAction_timestamp success_criteria(::autonomy_interfaces::msg::AdaptiveAction::_success_criteria_type arg)
  {
    msg_.success_criteria = std::move(arg);
    return Init_AdaptiveAction_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_expected_duration
{
public:
  explicit Init_AdaptiveAction_expected_duration(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  Init_AdaptiveAction_success_criteria expected_duration(::autonomy_interfaces::msg::AdaptiveAction::_expected_duration_type arg)
  {
    msg_.expected_duration = std::move(arg);
    return Init_AdaptiveAction_success_criteria(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_priority
{
public:
  explicit Init_AdaptiveAction_priority(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  Init_AdaptiveAction_expected_duration priority(::autonomy_interfaces::msg::AdaptiveAction::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_AdaptiveAction_expected_duration(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_trigger_context
{
public:
  explicit Init_AdaptiveAction_trigger_context(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  Init_AdaptiveAction_priority trigger_context(::autonomy_interfaces::msg::AdaptiveAction::_trigger_context_type arg)
  {
    msg_.trigger_context = std::move(arg);
    return Init_AdaptiveAction_priority(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_parameters
{
public:
  explicit Init_AdaptiveAction_parameters(::autonomy_interfaces::msg::AdaptiveAction & msg)
  : msg_(msg)
  {}
  Init_AdaptiveAction_trigger_context parameters(::autonomy_interfaces::msg::AdaptiveAction::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_AdaptiveAction_trigger_context(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

class Init_AdaptiveAction_action_type
{
public:
  Init_AdaptiveAction_action_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AdaptiveAction_parameters action_type(::autonomy_interfaces::msg::AdaptiveAction::_action_type_type arg)
  {
    msg_.action_type = std::move(arg);
    return Init_AdaptiveAction_parameters(msg_);
  }

private:
  ::autonomy_interfaces::msg::AdaptiveAction msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::AdaptiveAction>()
{
  return autonomy_interfaces::msg::builder::Init_AdaptiveAction_action_type();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__BUILDER_HPP_
