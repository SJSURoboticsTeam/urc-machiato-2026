// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/VerifySafetyProperty.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/verify_safety_property.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/verify_safety_property__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_VerifySafetyProperty_Request_property_name
{
public:
  Init_VerifySafetyProperty_Request_property_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::srv::VerifySafetyProperty_Request property_name(::autonomy_interfaces::srv::VerifySafetyProperty_Request::_property_name_type arg)
  {
    msg_.property_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::VerifySafetyProperty_Request>()
{
  return autonomy_interfaces::srv::builder::Init_VerifySafetyProperty_Request_property_name();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_VerifySafetyProperty_Response_last_violation
{
public:
  explicit Init_VerifySafetyProperty_Response_last_violation(::autonomy_interfaces::srv::VerifySafetyProperty_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response last_violation(::autonomy_interfaces::srv::VerifySafetyProperty_Response::_last_violation_type arg)
  {
    msg_.last_violation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response msg_;
};

class Init_VerifySafetyProperty_Response_violation_count
{
public:
  explicit Init_VerifySafetyProperty_Response_violation_count(::autonomy_interfaces::srv::VerifySafetyProperty_Response & msg)
  : msg_(msg)
  {}
  Init_VerifySafetyProperty_Response_last_violation violation_count(::autonomy_interfaces::srv::VerifySafetyProperty_Response::_violation_count_type arg)
  {
    msg_.violation_count = std::move(arg);
    return Init_VerifySafetyProperty_Response_last_violation(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response msg_;
};

class Init_VerifySafetyProperty_Response_details
{
public:
  explicit Init_VerifySafetyProperty_Response_details(::autonomy_interfaces::srv::VerifySafetyProperty_Response & msg)
  : msg_(msg)
  {}
  Init_VerifySafetyProperty_Response_violation_count details(::autonomy_interfaces::srv::VerifySafetyProperty_Response::_details_type arg)
  {
    msg_.details = std::move(arg);
    return Init_VerifySafetyProperty_Response_violation_count(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response msg_;
};

class Init_VerifySafetyProperty_Response_satisfied
{
public:
  explicit Init_VerifySafetyProperty_Response_satisfied(::autonomy_interfaces::srv::VerifySafetyProperty_Response & msg)
  : msg_(msg)
  {}
  Init_VerifySafetyProperty_Response_details satisfied(::autonomy_interfaces::srv::VerifySafetyProperty_Response::_satisfied_type arg)
  {
    msg_.satisfied = std::move(arg);
    return Init_VerifySafetyProperty_Response_details(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response msg_;
};

class Init_VerifySafetyProperty_Response_property_name
{
public:
  Init_VerifySafetyProperty_Response_property_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VerifySafetyProperty_Response_satisfied property_name(::autonomy_interfaces::srv::VerifySafetyProperty_Response::_property_name_type arg)
  {
    msg_.property_name = std::move(arg);
    return Init_VerifySafetyProperty_Response_satisfied(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::VerifySafetyProperty_Response>()
{
  return autonomy_interfaces::srv::builder::Init_VerifySafetyProperty_Response_property_name();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_VerifySafetyProperty_Event_response
{
public:
  explicit Init_VerifySafetyProperty_Event_response(::autonomy_interfaces::srv::VerifySafetyProperty_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::VerifySafetyProperty_Event response(::autonomy_interfaces::srv::VerifySafetyProperty_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Event msg_;
};

class Init_VerifySafetyProperty_Event_request
{
public:
  explicit Init_VerifySafetyProperty_Event_request(::autonomy_interfaces::srv::VerifySafetyProperty_Event & msg)
  : msg_(msg)
  {}
  Init_VerifySafetyProperty_Event_response request(::autonomy_interfaces::srv::VerifySafetyProperty_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_VerifySafetyProperty_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Event msg_;
};

class Init_VerifySafetyProperty_Event_info
{
public:
  Init_VerifySafetyProperty_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VerifySafetyProperty_Event_request info(::autonomy_interfaces::srv::VerifySafetyProperty_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_VerifySafetyProperty_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::VerifySafetyProperty_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::VerifySafetyProperty_Event>()
{
  return autonomy_interfaces::srv::builder::Init_VerifySafetyProperty_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__BUILDER_HPP_
