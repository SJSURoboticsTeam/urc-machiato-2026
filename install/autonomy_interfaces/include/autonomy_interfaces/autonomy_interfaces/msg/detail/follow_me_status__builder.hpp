// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/FollowMeStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/follow_me_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/follow_me_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_FollowMeStatus_operator_id
{
public:
  explicit Init_FollowMeStatus_operator_id(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::FollowMeStatus operator_id(::autonomy_interfaces::msg::FollowMeStatus::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_max_speed
{
public:
  explicit Init_FollowMeStatus_max_speed(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_operator_id max_speed(::autonomy_interfaces::msg::FollowMeStatus::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_FollowMeStatus_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_last_detection_time
{
public:
  explicit Init_FollowMeStatus_last_detection_time(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_max_speed last_detection_time(::autonomy_interfaces::msg::FollowMeStatus::_last_detection_time_type arg)
  {
    msg_.last_detection_time = std::move(arg);
    return Init_FollowMeStatus_max_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_target_visible
{
public:
  explicit Init_FollowMeStatus_target_visible(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_last_detection_time target_visible(::autonomy_interfaces::msg::FollowMeStatus::_target_visible_type arg)
  {
    msg_.target_visible = std::move(arg);
    return Init_FollowMeStatus_last_detection_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_target_position
{
public:
  explicit Init_FollowMeStatus_target_position(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_target_visible target_position(::autonomy_interfaces::msg::FollowMeStatus::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_FollowMeStatus_target_visible(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_current_speed
{
public:
  explicit Init_FollowMeStatus_current_speed(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_target_position current_speed(::autonomy_interfaces::msg::FollowMeStatus::_current_speed_type arg)
  {
    msg_.current_speed = std::move(arg);
    return Init_FollowMeStatus_target_position(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_safety_violation
{
public:
  explicit Init_FollowMeStatus_safety_violation(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_current_speed safety_violation(::autonomy_interfaces::msg::FollowMeStatus::_safety_violation_type arg)
  {
    msg_.safety_violation = std::move(arg);
    return Init_FollowMeStatus_current_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_safety_distance
{
public:
  explicit Init_FollowMeStatus_safety_distance(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_safety_violation safety_distance(::autonomy_interfaces::msg::FollowMeStatus::_safety_distance_type arg)
  {
    msg_.safety_distance = std::move(arg);
    return Init_FollowMeStatus_safety_violation(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_target_angle
{
public:
  explicit Init_FollowMeStatus_target_angle(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_safety_distance target_angle(::autonomy_interfaces::msg::FollowMeStatus::_target_angle_type arg)
  {
    msg_.target_angle = std::move(arg);
    return Init_FollowMeStatus_safety_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_target_distance
{
public:
  explicit Init_FollowMeStatus_target_distance(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_target_angle target_distance(::autonomy_interfaces::msg::FollowMeStatus::_target_distance_type arg)
  {
    msg_.target_distance = std::move(arg);
    return Init_FollowMeStatus_target_angle(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_target_tag_id
{
public:
  explicit Init_FollowMeStatus_target_tag_id(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_target_distance target_tag_id(::autonomy_interfaces::msg::FollowMeStatus::_target_tag_id_type arg)
  {
    msg_.target_tag_id = std::move(arg);
    return Init_FollowMeStatus_target_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_is_following
{
public:
  explicit Init_FollowMeStatus_is_following(::autonomy_interfaces::msg::FollowMeStatus & msg)
  : msg_(msg)
  {}
  Init_FollowMeStatus_target_tag_id is_following(::autonomy_interfaces::msg::FollowMeStatus::_is_following_type arg)
  {
    msg_.is_following = std::move(arg);
    return Init_FollowMeStatus_target_tag_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

class Init_FollowMeStatus_header
{
public:
  Init_FollowMeStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowMeStatus_is_following header(::autonomy_interfaces::msg::FollowMeStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FollowMeStatus_is_following(msg_);
  }

private:
  ::autonomy_interfaces::msg::FollowMeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::FollowMeStatus>()
{
  return autonomy_interfaces::msg::builder::Init_FollowMeStatus_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__BUILDER_HPP_
