// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/software_estop.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SoftwareEstop_Request_force_immediate
{
public:
  explicit Init_SoftwareEstop_Request_force_immediate(::autonomy_interfaces::srv::SoftwareEstop_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SoftwareEstop_Request force_immediate(::autonomy_interfaces::srv::SoftwareEstop_Request::_force_immediate_type arg)
  {
    msg_.force_immediate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Request msg_;
};

class Init_SoftwareEstop_Request_acknowledge_criticality
{
public:
  explicit Init_SoftwareEstop_Request_acknowledge_criticality(::autonomy_interfaces::srv::SoftwareEstop_Request & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Request_force_immediate acknowledge_criticality(::autonomy_interfaces::srv::SoftwareEstop_Request::_acknowledge_criticality_type arg)
  {
    msg_.acknowledge_criticality = std::move(arg);
    return Init_SoftwareEstop_Request_force_immediate(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Request msg_;
};

class Init_SoftwareEstop_Request_reason
{
public:
  explicit Init_SoftwareEstop_Request_reason(::autonomy_interfaces::srv::SoftwareEstop_Request & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Request_acknowledge_criticality reason(::autonomy_interfaces::srv::SoftwareEstop_Request::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_SoftwareEstop_Request_acknowledge_criticality(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Request msg_;
};

class Init_SoftwareEstop_Request_operator_id
{
public:
  Init_SoftwareEstop_Request_operator_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SoftwareEstop_Request_reason operator_id(::autonomy_interfaces::srv::SoftwareEstop_Request::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return Init_SoftwareEstop_Request_reason(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SoftwareEstop_Request>()
{
  return autonomy_interfaces::srv::builder::Init_SoftwareEstop_Request_operator_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SoftwareEstop_Response_coordination_started
{
public:
  explicit Init_SoftwareEstop_Response_coordination_started(::autonomy_interfaces::srv::SoftwareEstop_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SoftwareEstop_Response coordination_started(::autonomy_interfaces::srv::SoftwareEstop_Response::_coordination_started_type arg)
  {
    msg_.coordination_started = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

class Init_SoftwareEstop_Response_triggered_by
{
public:
  explicit Init_SoftwareEstop_Response_triggered_by(::autonomy_interfaces::srv::SoftwareEstop_Response & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Response_coordination_started triggered_by(::autonomy_interfaces::srv::SoftwareEstop_Response::_triggered_by_type arg)
  {
    msg_.triggered_by = std::move(arg);
    return Init_SoftwareEstop_Response_coordination_started(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

class Init_SoftwareEstop_Response_timestamp
{
public:
  explicit Init_SoftwareEstop_Response_timestamp(::autonomy_interfaces::srv::SoftwareEstop_Response & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Response_triggered_by timestamp(::autonomy_interfaces::srv::SoftwareEstop_Response::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SoftwareEstop_Response_triggered_by(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

class Init_SoftwareEstop_Response_estop_id
{
public:
  explicit Init_SoftwareEstop_Response_estop_id(::autonomy_interfaces::srv::SoftwareEstop_Response & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Response_timestamp estop_id(::autonomy_interfaces::srv::SoftwareEstop_Response::_estop_id_type arg)
  {
    msg_.estop_id = std::move(arg);
    return Init_SoftwareEstop_Response_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

class Init_SoftwareEstop_Response_message
{
public:
  explicit Init_SoftwareEstop_Response_message(::autonomy_interfaces::srv::SoftwareEstop_Response & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Response_estop_id message(::autonomy_interfaces::srv::SoftwareEstop_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SoftwareEstop_Response_estop_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

class Init_SoftwareEstop_Response_success
{
public:
  Init_SoftwareEstop_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SoftwareEstop_Response_message success(::autonomy_interfaces::srv::SoftwareEstop_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SoftwareEstop_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SoftwareEstop_Response>()
{
  return autonomy_interfaces::srv::builder::Init_SoftwareEstop_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_SoftwareEstop_Event_response
{
public:
  explicit Init_SoftwareEstop_Event_response(::autonomy_interfaces::srv::SoftwareEstop_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::SoftwareEstop_Event response(::autonomy_interfaces::srv::SoftwareEstop_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Event msg_;
};

class Init_SoftwareEstop_Event_request
{
public:
  explicit Init_SoftwareEstop_Event_request(::autonomy_interfaces::srv::SoftwareEstop_Event & msg)
  : msg_(msg)
  {}
  Init_SoftwareEstop_Event_response request(::autonomy_interfaces::srv::SoftwareEstop_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SoftwareEstop_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Event msg_;
};

class Init_SoftwareEstop_Event_info
{
public:
  Init_SoftwareEstop_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SoftwareEstop_Event_request info(::autonomy_interfaces::srv::SoftwareEstop_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SoftwareEstop_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::SoftwareEstop_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::SoftwareEstop_Event>()
{
  return autonomy_interfaces::srv::builder::Init_SoftwareEstop_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__BUILDER_HPP_
