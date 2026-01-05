// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/recover_from_safety.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/recover_from_safety__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_RecoverFromSafety_Request_notes
{
public:
  explicit Init_RecoverFromSafety_Request_notes(::autonomy_interfaces::srv::RecoverFromSafety_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::RecoverFromSafety_Request notes(::autonomy_interfaces::srv::RecoverFromSafety_Request::_notes_type arg)
  {
    msg_.notes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Request msg_;
};

class Init_RecoverFromSafety_Request_completed_steps
{
public:
  explicit Init_RecoverFromSafety_Request_completed_steps(::autonomy_interfaces::srv::RecoverFromSafety_Request & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Request_notes completed_steps(::autonomy_interfaces::srv::RecoverFromSafety_Request::_completed_steps_type arg)
  {
    msg_.completed_steps = std::move(arg);
    return Init_RecoverFromSafety_Request_notes(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Request msg_;
};

class Init_RecoverFromSafety_Request_acknowledge_risks
{
public:
  explicit Init_RecoverFromSafety_Request_acknowledge_risks(::autonomy_interfaces::srv::RecoverFromSafety_Request & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Request_completed_steps acknowledge_risks(::autonomy_interfaces::srv::RecoverFromSafety_Request::_acknowledge_risks_type arg)
  {
    msg_.acknowledge_risks = std::move(arg);
    return Init_RecoverFromSafety_Request_completed_steps(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Request msg_;
};

class Init_RecoverFromSafety_Request_operator_id
{
public:
  explicit Init_RecoverFromSafety_Request_operator_id(::autonomy_interfaces::srv::RecoverFromSafety_Request & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Request_acknowledge_risks operator_id(::autonomy_interfaces::srv::RecoverFromSafety_Request::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return Init_RecoverFromSafety_Request_acknowledge_risks(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Request msg_;
};

class Init_RecoverFromSafety_Request_recovery_method
{
public:
  Init_RecoverFromSafety_Request_recovery_method()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecoverFromSafety_Request_operator_id recovery_method(::autonomy_interfaces::srv::RecoverFromSafety_Request::_recovery_method_type arg)
  {
    msg_.recovery_method = std::move(arg);
    return Init_RecoverFromSafety_Request_operator_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::RecoverFromSafety_Request>()
{
  return autonomy_interfaces::srv::builder::Init_RecoverFromSafety_Request_recovery_method();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_RecoverFromSafety_Response_restrictions
{
public:
  explicit Init_RecoverFromSafety_Response_restrictions(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::RecoverFromSafety_Response restrictions(::autonomy_interfaces::srv::RecoverFromSafety_Response::_restrictions_type arg)
  {
    msg_.restrictions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_recommended_next_state
{
public:
  explicit Init_RecoverFromSafety_Response_recommended_next_state(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_restrictions recommended_next_state(::autonomy_interfaces::srv::RecoverFromSafety_Response::_recommended_next_state_type arg)
  {
    msg_.recommended_next_state = std::move(arg);
    return Init_RecoverFromSafety_Response_restrictions(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_estimated_time
{
public:
  explicit Init_RecoverFromSafety_Response_estimated_time(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_recommended_next_state estimated_time(::autonomy_interfaces::srv::RecoverFromSafety_Response::_estimated_time_type arg)
  {
    msg_.estimated_time = std::move(arg);
    return Init_RecoverFromSafety_Response_recommended_next_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_failed_systems
{
public:
  explicit Init_RecoverFromSafety_Response_failed_systems(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_estimated_time failed_systems(::autonomy_interfaces::srv::RecoverFromSafety_Response::_failed_systems_type arg)
  {
    msg_.failed_systems = std::move(arg);
    return Init_RecoverFromSafety_Response_estimated_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_verified_systems
{
public:
  explicit Init_RecoverFromSafety_Response_verified_systems(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_failed_systems verified_systems(::autonomy_interfaces::srv::RecoverFromSafety_Response::_verified_systems_type arg)
  {
    msg_.verified_systems = std::move(arg);
    return Init_RecoverFromSafety_Response_failed_systems(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_remaining_steps
{
public:
  explicit Init_RecoverFromSafety_Response_remaining_steps(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_verified_systems remaining_steps(::autonomy_interfaces::srv::RecoverFromSafety_Response::_remaining_steps_type arg)
  {
    msg_.remaining_steps = std::move(arg);
    return Init_RecoverFromSafety_Response_verified_systems(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_is_safe_to_proceed
{
public:
  explicit Init_RecoverFromSafety_Response_is_safe_to_proceed(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_remaining_steps is_safe_to_proceed(::autonomy_interfaces::srv::RecoverFromSafety_Response::_is_safe_to_proceed_type arg)
  {
    msg_.is_safe_to_proceed = std::move(arg);
    return Init_RecoverFromSafety_Response_remaining_steps(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_recovery_state
{
public:
  explicit Init_RecoverFromSafety_Response_recovery_state(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_is_safe_to_proceed recovery_state(::autonomy_interfaces::srv::RecoverFromSafety_Response::_recovery_state_type arg)
  {
    msg_.recovery_state = std::move(arg);
    return Init_RecoverFromSafety_Response_is_safe_to_proceed(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_message
{
public:
  explicit Init_RecoverFromSafety_Response_message(::autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Response_recovery_state message(::autonomy_interfaces::srv::RecoverFromSafety_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_RecoverFromSafety_Response_recovery_state(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

class Init_RecoverFromSafety_Response_success
{
public:
  Init_RecoverFromSafety_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecoverFromSafety_Response_message success(::autonomy_interfaces::srv::RecoverFromSafety_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RecoverFromSafety_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::RecoverFromSafety_Response>()
{
  return autonomy_interfaces::srv::builder::Init_RecoverFromSafety_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_RecoverFromSafety_Event_response
{
public:
  explicit Init_RecoverFromSafety_Event_response(::autonomy_interfaces::srv::RecoverFromSafety_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::RecoverFromSafety_Event response(::autonomy_interfaces::srv::RecoverFromSafety_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Event msg_;
};

class Init_RecoverFromSafety_Event_request
{
public:
  explicit Init_RecoverFromSafety_Event_request(::autonomy_interfaces::srv::RecoverFromSafety_Event & msg)
  : msg_(msg)
  {}
  Init_RecoverFromSafety_Event_response request(::autonomy_interfaces::srv::RecoverFromSafety_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RecoverFromSafety_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Event msg_;
};

class Init_RecoverFromSafety_Event_info
{
public:
  Init_RecoverFromSafety_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecoverFromSafety_Event_request info(::autonomy_interfaces::srv::RecoverFromSafety_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RecoverFromSafety_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::RecoverFromSafety_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::RecoverFromSafety_Event>()
{
  return autonomy_interfaces::srv::builder::Init_RecoverFromSafety_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__BUILDER_HPP_
