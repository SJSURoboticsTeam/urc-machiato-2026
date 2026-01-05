// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/typing_goal.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/typing_goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_TypingGoal_contact_force
{
public:
  explicit Init_TypingGoal_contact_force(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::TypingGoal contact_force(::autonomy_interfaces::msg::TypingGoal::_contact_force_type arg)
  {
    msg_.contact_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_standoff_distance
{
public:
  explicit Init_TypingGoal_standoff_distance(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_contact_force standoff_distance(::autonomy_interfaces::msg::TypingGoal::_standoff_distance_type arg)
  {
    msg_.standoff_distance = std::move(arg);
    return Init_TypingGoal_contact_force(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_key_dimensions
{
public:
  explicit Init_TypingGoal_key_dimensions(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_standoff_distance key_dimensions(::autonomy_interfaces::msg::TypingGoal::_key_dimensions_type arg)
  {
    msg_.key_dimensions = std::move(arg);
    return Init_TypingGoal_standoff_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_key_spacing_m
{
public:
  explicit Init_TypingGoal_key_spacing_m(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_key_dimensions key_spacing_m(::autonomy_interfaces::msg::TypingGoal::_key_spacing_m_type arg)
  {
    msg_.key_spacing_m = std::move(arg);
    return Init_TypingGoal_key_dimensions(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_keyboard_layout
{
public:
  explicit Init_TypingGoal_keyboard_layout(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_key_spacing_m keyboard_layout(::autonomy_interfaces::msg::TypingGoal::_keyboard_layout_type arg)
  {
    msg_.keyboard_layout = std::move(arg);
    return Init_TypingGoal_key_spacing_m(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_timeout
{
public:
  explicit Init_TypingGoal_timeout(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_keyboard_layout timeout(::autonomy_interfaces::msg::TypingGoal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_TypingGoal_keyboard_layout(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_speed_requirement
{
public:
  explicit Init_TypingGoal_speed_requirement(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_timeout speed_requirement(::autonomy_interfaces::msg::TypingGoal::_speed_requirement_type arg)
  {
    msg_.speed_requirement = std::move(arg);
    return Init_TypingGoal_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_accuracy_requirement
{
public:
  explicit Init_TypingGoal_accuracy_requirement(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_speed_requirement accuracy_requirement(::autonomy_interfaces::msg::TypingGoal::_accuracy_requirement_type arg)
  {
    msg_.accuracy_requirement = std::move(arg);
    return Init_TypingGoal_speed_requirement(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_keyboard_pose
{
public:
  explicit Init_TypingGoal_keyboard_pose(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_accuracy_requirement keyboard_pose(::autonomy_interfaces::msg::TypingGoal::_keyboard_pose_type arg)
  {
    msg_.keyboard_pose = std::move(arg);
    return Init_TypingGoal_accuracy_requirement(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_target_text
{
public:
  explicit Init_TypingGoal_target_text(::autonomy_interfaces::msg::TypingGoal & msg)
  : msg_(msg)
  {}
  Init_TypingGoal_keyboard_pose target_text(::autonomy_interfaces::msg::TypingGoal::_target_text_type arg)
  {
    msg_.target_text = std::move(arg);
    return Init_TypingGoal_keyboard_pose(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

class Init_TypingGoal_header
{
public:
  Init_TypingGoal_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TypingGoal_target_text header(::autonomy_interfaces::msg::TypingGoal::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TypingGoal_target_text(msg_);
  }

private:
  ::autonomy_interfaces::msg::TypingGoal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::TypingGoal>()
{
  return autonomy_interfaces::msg::builder::Init_TypingGoal_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__BUILDER_HPP_
