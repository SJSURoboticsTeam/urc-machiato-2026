// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetNetworkStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_network_stats.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_NETWORK_STATS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_NETWORK_STATS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_network_stats__struct.hpp"
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
auto build<::autonomy_interfaces::srv::GetNetworkStats_Request>()
{
  return ::autonomy_interfaces::srv::GetNetworkStats_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetNetworkStats_Response_packet_loss_rate
{
public:
  explicit Init_GetNetworkStats_Response_packet_loss_rate(::autonomy_interfaces::srv::GetNetworkStats_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetNetworkStats_Response packet_loss_rate(::autonomy_interfaces::srv::GetNetworkStats_Response::_packet_loss_rate_type arg)
  {
    msg_.packet_loss_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Response msg_;
};

class Init_GetNetworkStats_Response_latency_ms
{
public:
  explicit Init_GetNetworkStats_Response_latency_ms(::autonomy_interfaces::srv::GetNetworkStats_Response & msg)
  : msg_(msg)
  {}
  Init_GetNetworkStats_Response_packet_loss_rate latency_ms(::autonomy_interfaces::srv::GetNetworkStats_Response::_latency_ms_type arg)
  {
    msg_.latency_ms = std::move(arg);
    return Init_GetNetworkStats_Response_packet_loss_rate(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Response msg_;
};

class Init_GetNetworkStats_Response_bandwidth_down_mbps
{
public:
  explicit Init_GetNetworkStats_Response_bandwidth_down_mbps(::autonomy_interfaces::srv::GetNetworkStats_Response & msg)
  : msg_(msg)
  {}
  Init_GetNetworkStats_Response_latency_ms bandwidth_down_mbps(::autonomy_interfaces::srv::GetNetworkStats_Response::_bandwidth_down_mbps_type arg)
  {
    msg_.bandwidth_down_mbps = std::move(arg);
    return Init_GetNetworkStats_Response_latency_ms(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Response msg_;
};

class Init_GetNetworkStats_Response_bandwidth_up_mbps
{
public:
  Init_GetNetworkStats_Response_bandwidth_up_mbps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetNetworkStats_Response_bandwidth_down_mbps bandwidth_up_mbps(::autonomy_interfaces::srv::GetNetworkStats_Response::_bandwidth_up_mbps_type arg)
  {
    msg_.bandwidth_up_mbps = std::move(arg);
    return Init_GetNetworkStats_Response_bandwidth_down_mbps(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetNetworkStats_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetNetworkStats_Response_bandwidth_up_mbps();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetNetworkStats_Event_response
{
public:
  explicit Init_GetNetworkStats_Event_response(::autonomy_interfaces::srv::GetNetworkStats_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::GetNetworkStats_Event response(::autonomy_interfaces::srv::GetNetworkStats_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Event msg_;
};

class Init_GetNetworkStats_Event_request
{
public:
  explicit Init_GetNetworkStats_Event_request(::autonomy_interfaces::srv::GetNetworkStats_Event & msg)
  : msg_(msg)
  {}
  Init_GetNetworkStats_Event_response request(::autonomy_interfaces::srv::GetNetworkStats_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetNetworkStats_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Event msg_;
};

class Init_GetNetworkStats_Event_info
{
public:
  Init_GetNetworkStats_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetNetworkStats_Event_request info(::autonomy_interfaces::srv::GetNetworkStats_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetNetworkStats_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetNetworkStats_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetNetworkStats_Event>()
{
  return autonomy_interfaces::srv::builder::Init_GetNetworkStats_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_NETWORK_STATS__BUILDER_HPP_
