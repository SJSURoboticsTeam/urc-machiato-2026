// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/follow_me_control.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/follow_me_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_FollowMeControl_Request_operator_id
{
public:
  explicit Init_FollowMeControl_Request_operator_id(::autonomy_interfaces::srv::FollowMeControl_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::FollowMeControl_Request operator_id(::autonomy_interfaces::srv::FollowMeControl_Request::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Request msg_;
};

class Init_FollowMeControl_Request_enable_following
{
public:
  explicit Init_FollowMeControl_Request_enable_following(::autonomy_interfaces::srv::FollowMeControl_Request & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Request_operator_id enable_following(::autonomy_interfaces::srv::FollowMeControl_Request::_enable_following_type arg)
  {
    msg_.enable_following = std::move(arg);
    return Init_FollowMeControl_Request_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Request msg_;
};

class Init_FollowMeControl_Request_max_speed
{
public:
  explicit Init_FollowMeControl_Request_max_speed(::autonomy_interfaces::srv::FollowMeControl_Request & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Request_enable_following max_speed(::autonomy_interfaces::srv::FollowMeControl_Request::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_FollowMeControl_Request_enable_following(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Request msg_;
};

class Init_FollowMeControl_Request_safety_distance
{
public:
  explicit Init_FollowMeControl_Request_safety_distance(::autonomy_interfaces::srv::FollowMeControl_Request & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Request_max_speed safety_distance(::autonomy_interfaces::srv::FollowMeControl_Request::_safety_distance_type arg)
  {
    msg_.safety_distance = std::move(arg);
    return Init_FollowMeControl_Request_max_speed(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Request msg_;
};

class Init_FollowMeControl_Request_target_tag_id
{
public:
  Init_FollowMeControl_Request_target_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowMeControl_Request_safety_distance target_tag_id(::autonomy_interfaces::srv::FollowMeControl_Request::_target_tag_id_type arg)
  {
    msg_.target_tag_id = std::move(arg);
    return Init_FollowMeControl_Request_safety_distance(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::FollowMeControl_Request>()
{
  return autonomy_interfaces::srv::builder::Init_FollowMeControl_Request_target_tag_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_FollowMeControl_Response_target_position
{
public:
  explicit Init_FollowMeControl_Response_target_position(::autonomy_interfaces::srv::FollowMeControl_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::FollowMeControl_Response target_position(::autonomy_interfaces::srv::FollowMeControl_Response::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

class Init_FollowMeControl_Response_current_distance
{
public:
  explicit Init_FollowMeControl_Response_current_distance(::autonomy_interfaces::srv::FollowMeControl_Response & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Response_target_position current_distance(::autonomy_interfaces::srv::FollowMeControl_Response::_current_distance_type arg)
  {
    msg_.current_distance = std::move(arg);
    return Init_FollowMeControl_Response_target_position(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

class Init_FollowMeControl_Response_current_target_tag
{
public:
  explicit Init_FollowMeControl_Response_current_target_tag(::autonomy_interfaces::srv::FollowMeControl_Response & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Response_current_distance current_target_tag(::autonomy_interfaces::srv::FollowMeControl_Response::_current_target_tag_type arg)
  {
    msg_.current_target_tag = std::move(arg);
    return Init_FollowMeControl_Response_current_distance(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

class Init_FollowMeControl_Response_is_following
{
public:
  explicit Init_FollowMeControl_Response_is_following(::autonomy_interfaces::srv::FollowMeControl_Response & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Response_current_target_tag is_following(::autonomy_interfaces::srv::FollowMeControl_Response::_is_following_type arg)
  {
    msg_.is_following = std::move(arg);
    return Init_FollowMeControl_Response_current_target_tag(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

class Init_FollowMeControl_Response_message
{
public:
  explicit Init_FollowMeControl_Response_message(::autonomy_interfaces::srv::FollowMeControl_Response & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Response_is_following message(::autonomy_interfaces::srv::FollowMeControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_FollowMeControl_Response_is_following(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

class Init_FollowMeControl_Response_success
{
public:
  Init_FollowMeControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowMeControl_Response_message success(::autonomy_interfaces::srv::FollowMeControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FollowMeControl_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::FollowMeControl_Response>()
{
  return autonomy_interfaces::srv::builder::Init_FollowMeControl_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_FollowMeControl_Event_response
{
public:
  explicit Init_FollowMeControl_Event_response(::autonomy_interfaces::srv::FollowMeControl_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::FollowMeControl_Event response(::autonomy_interfaces::srv::FollowMeControl_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Event msg_;
};

class Init_FollowMeControl_Event_request
{
public:
  explicit Init_FollowMeControl_Event_request(::autonomy_interfaces::srv::FollowMeControl_Event & msg)
  : msg_(msg)
  {}
  Init_FollowMeControl_Event_response request(::autonomy_interfaces::srv::FollowMeControl_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FollowMeControl_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Event msg_;
};

class Init_FollowMeControl_Event_info
{
public:
  Init_FollowMeControl_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowMeControl_Event_request info(::autonomy_interfaces::srv::FollowMeControl_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FollowMeControl_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::FollowMeControl_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::FollowMeControl_Event>()
{
  return autonomy_interfaces::srv::builder::Init_FollowMeControl_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__BUILDER_HPP_
