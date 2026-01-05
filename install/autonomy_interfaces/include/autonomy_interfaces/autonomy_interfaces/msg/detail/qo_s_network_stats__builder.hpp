// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_network_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/qo_s_network_stats__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_QoSNetworkStats_signal_strength
{
public:
  explicit Init_QoSNetworkStats_signal_strength(::autonomy_interfaces::msg::QoSNetworkStats & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::QoSNetworkStats signal_strength(::autonomy_interfaces::msg::QoSNetworkStats::_signal_strength_type arg)
  {
    msg_.signal_strength = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

class Init_QoSNetworkStats_current_band
{
public:
  explicit Init_QoSNetworkStats_current_band(::autonomy_interfaces::msg::QoSNetworkStats & msg)
  : msg_(msg)
  {}
  Init_QoSNetworkStats_signal_strength current_band(::autonomy_interfaces::msg::QoSNetworkStats::_current_band_type arg)
  {
    msg_.current_band = std::move(arg);
    return Init_QoSNetworkStats_signal_strength(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

class Init_QoSNetworkStats_packet_loss_rate
{
public:
  explicit Init_QoSNetworkStats_packet_loss_rate(::autonomy_interfaces::msg::QoSNetworkStats & msg)
  : msg_(msg)
  {}
  Init_QoSNetworkStats_current_band packet_loss_rate(::autonomy_interfaces::msg::QoSNetworkStats::_packet_loss_rate_type arg)
  {
    msg_.packet_loss_rate = std::move(arg);
    return Init_QoSNetworkStats_current_band(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

class Init_QoSNetworkStats_latency_ms
{
public:
  explicit Init_QoSNetworkStats_latency_ms(::autonomy_interfaces::msg::QoSNetworkStats & msg)
  : msg_(msg)
  {}
  Init_QoSNetworkStats_packet_loss_rate latency_ms(::autonomy_interfaces::msg::QoSNetworkStats::_latency_ms_type arg)
  {
    msg_.latency_ms = std::move(arg);
    return Init_QoSNetworkStats_packet_loss_rate(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

class Init_QoSNetworkStats_bandwidth_down_mbps
{
public:
  explicit Init_QoSNetworkStats_bandwidth_down_mbps(::autonomy_interfaces::msg::QoSNetworkStats & msg)
  : msg_(msg)
  {}
  Init_QoSNetworkStats_latency_ms bandwidth_down_mbps(::autonomy_interfaces::msg::QoSNetworkStats::_bandwidth_down_mbps_type arg)
  {
    msg_.bandwidth_down_mbps = std::move(arg);
    return Init_QoSNetworkStats_latency_ms(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

class Init_QoSNetworkStats_bandwidth_up_mbps
{
public:
  Init_QoSNetworkStats_bandwidth_up_mbps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QoSNetworkStats_bandwidth_down_mbps bandwidth_up_mbps(::autonomy_interfaces::msg::QoSNetworkStats::_bandwidth_up_mbps_type arg)
  {
    msg_.bandwidth_up_mbps = std::move(arg);
    return Init_QoSNetworkStats_bandwidth_down_mbps(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSNetworkStats msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::QoSNetworkStats>()
{
  return autonomy_interfaces::msg::builder::Init_QoSNetworkStats_bandwidth_up_mbps();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__BUILDER_HPP_
