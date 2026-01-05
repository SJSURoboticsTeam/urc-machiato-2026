// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetQoSProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_qo_s_profile.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_qo_s_profile__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetQoSProfile_Request>()
{
  return ::autonomy_interfaces::srv::GetQoSProfile_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetQoSProfile_Response_network_stats
{
public:
  explicit Init_GetQoSProfile_Response_network_stats(::autonomy_interfaces::srv::GetQoSProfile_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetQoSProfile_Response network_stats(::autonomy_interfaces::srv::GetQoSProfile_Response::_network_stats_type arg)
  {
    msg_.network_stats = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetQoSProfile_Response msg_;
};

class Init_GetQoSProfile_Response_topic_profiles
{
public:
  Init_GetQoSProfile_Response_topic_profiles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetQoSProfile_Response_network_stats topic_profiles(::autonomy_interfaces::srv::GetQoSProfile_Response::_topic_profiles_type arg)
  {
    msg_.topic_profiles = std::move(arg);
    return Init_GetQoSProfile_Response_network_stats(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetQoSProfile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetQoSProfile_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetQoSProfile_Response_topic_profiles();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetQoSProfile_Event_response
{
public:
  explicit Init_GetQoSProfile_Event_response(::autonomy_interfaces::srv::GetQoSProfile_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetQoSProfile_Event response(::autonomy_interfaces::srv::GetQoSProfile_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetQoSProfile_Event msg_;
};

class Init_GetQoSProfile_Event_request
{
public:
  explicit Init_GetQoSProfile_Event_request(::autonomy_interfaces::srv::GetQoSProfile_Event & msg)
  : msg_(msg)
  {}
  Init_GetQoSProfile_Event_response request(::autonomy_interfaces::srv::GetQoSProfile_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetQoSProfile_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetQoSProfile_Event msg_;
};

class Init_GetQoSProfile_Event_info
{
public:
  Init_GetQoSProfile_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetQoSProfile_Event_request info(::autonomy_interfaces::srv::GetQoSProfile_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetQoSProfile_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetQoSProfile_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetQoSProfile_Event>()
{
  return autonomy_interfaces::srv::builder::Init_GetQoSProfile_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__BUILDER_HPP_
