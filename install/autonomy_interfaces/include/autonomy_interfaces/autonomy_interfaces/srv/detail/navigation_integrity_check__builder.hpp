// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/navigation_integrity_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_NavigationIntegrityCheck_Request_check_components
{
public:
  explicit Init_NavigationIntegrityCheck_Request_check_components(::autonomy_interfaces::srv::NavigationIntegrityCheck_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Request check_components(::autonomy_interfaces::srv::NavigationIntegrityCheck_Request::_check_components_type arg)
  {
    msg_.check_components = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Request msg_;
};

class Init_NavigationIntegrityCheck_Request_detailed_check
{
public:
  Init_NavigationIntegrityCheck_Request_detailed_check()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationIntegrityCheck_Request_check_components detailed_check(::autonomy_interfaces::srv::NavigationIntegrityCheck_Request::_detailed_check_type arg)
  {
    msg_.detailed_check = std::move(arg);
    return Init_NavigationIntegrityCheck_Request_check_components(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::NavigationIntegrityCheck_Request>()
{
  return autonomy_interfaces::srv::builder::Init_NavigationIntegrityCheck_Request_detailed_check();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_NavigationIntegrityCheck_Response_timestamp
{
public:
  explicit Init_NavigationIntegrityCheck_Response_timestamp(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response timestamp(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_recommendations
{
public:
  explicit Init_NavigationIntegrityCheck_Response_recommendations(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_timestamp recommendations(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_recommendations_type arg)
  {
    msg_.recommendations = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_hdop
{
public:
  explicit Init_NavigationIntegrityCheck_Response_hdop(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_recommendations hdop(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_recommendations(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_satellite_count
{
public:
  explicit Init_NavigationIntegrityCheck_Response_satellite_count(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_hdop satellite_count(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_satellite_count_type arg)
  {
    msg_.satellite_count = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_hdop(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_velocity_consistency
{
public:
  explicit Init_NavigationIntegrityCheck_Response_velocity_consistency(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_satellite_count velocity_consistency(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_velocity_consistency_type arg)
  {
    msg_.velocity_consistency = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_satellite_count(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_heading_accuracy
{
public:
  explicit Init_NavigationIntegrityCheck_Response_heading_accuracy(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_velocity_consistency heading_accuracy(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_heading_accuracy_type arg)
  {
    msg_.heading_accuracy = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_velocity_consistency(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_position_accuracy
{
public:
  explicit Init_NavigationIntegrityCheck_Response_position_accuracy(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_heading_accuracy position_accuracy(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_position_accuracy_type arg)
  {
    msg_.position_accuracy = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_heading_accuracy(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_component_details
{
public:
  explicit Init_NavigationIntegrityCheck_Response_component_details(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_position_accuracy component_details(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_component_details_type arg)
  {
    msg_.component_details = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_position_accuracy(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_component_status
{
public:
  explicit Init_NavigationIntegrityCheck_Response_component_status(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_component_details component_status(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_component_status_type arg)
  {
    msg_.component_status = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_component_details(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_checked_components
{
public:
  explicit Init_NavigationIntegrityCheck_Response_checked_components(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_component_status checked_components(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_checked_components_type arg)
  {
    msg_.checked_components = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_component_status(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_integrity_level
{
public:
  explicit Init_NavigationIntegrityCheck_Response_integrity_level(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_checked_components integrity_level(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_integrity_level_type arg)
  {
    msg_.integrity_level = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_checked_components(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_integrity_score
{
public:
  explicit Init_NavigationIntegrityCheck_Response_integrity_score(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
  : msg_(msg)
  {}
  Init_NavigationIntegrityCheck_Response_integrity_level integrity_score(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_integrity_score_type arg)
  {
    msg_.integrity_score = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_integrity_level(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

class Init_NavigationIntegrityCheck_Response_integrity_ok
{
public:
  Init_NavigationIntegrityCheck_Response_integrity_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationIntegrityCheck_Response_integrity_score integrity_ok(::autonomy_interfaces::srv::NavigationIntegrityCheck_Response::_integrity_ok_type arg)
  {
    msg_.integrity_ok = std::move(arg);
    return Init_NavigationIntegrityCheck_Response_integrity_score(msg_);
  }

private:
  ::autonomy_interfaces::srv::NavigationIntegrityCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::NavigationIntegrityCheck_Response>()
{
  return autonomy_interfaces::srv::builder::Init_NavigationIntegrityCheck_Response_integrity_ok();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__BUILDER_HPP_
