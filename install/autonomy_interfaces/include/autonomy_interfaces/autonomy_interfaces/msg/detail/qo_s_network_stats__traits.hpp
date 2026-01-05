// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_network_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/qo_s_network_stats__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const QoSNetworkStats & msg,
  std::ostream & out)
{
  out << "{";
  // member: bandwidth_up_mbps
  {
    out << "bandwidth_up_mbps: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_up_mbps, out);
    out << ", ";
  }

  // member: bandwidth_down_mbps
  {
    out << "bandwidth_down_mbps: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_down_mbps, out);
    out << ", ";
  }

  // member: latency_ms
  {
    out << "latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.latency_ms, out);
    out << ", ";
  }

  // member: packet_loss_rate
  {
    out << "packet_loss_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.packet_loss_rate, out);
    out << ", ";
  }

  // member: current_band
  {
    out << "current_band: ";
    rosidl_generator_traits::value_to_yaml(msg.current_band, out);
    out << ", ";
  }

  // member: signal_strength
  {
    out << "signal_strength: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_strength, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const QoSNetworkStats & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bandwidth_up_mbps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bandwidth_up_mbps: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_up_mbps, out);
    out << "\n";
  }

  // member: bandwidth_down_mbps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bandwidth_down_mbps: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_down_mbps, out);
    out << "\n";
  }

  // member: latency_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.latency_ms, out);
    out << "\n";
  }

  // member: packet_loss_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "packet_loss_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.packet_loss_rate, out);
    out << "\n";
  }

  // member: current_band
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_band: ";
    rosidl_generator_traits::value_to_yaml(msg.current_band, out);
    out << "\n";
  }

  // member: signal_strength
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "signal_strength: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_strength, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const QoSNetworkStats & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::msg::QoSNetworkStats & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::QoSNetworkStats & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::QoSNetworkStats>()
{
  return "autonomy_interfaces::msg::QoSNetworkStats";
}

template<>
inline const char * name<autonomy_interfaces::msg::QoSNetworkStats>()
{
  return "autonomy_interfaces/msg/QoSNetworkStats";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::QoSNetworkStats>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::QoSNetworkStats>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::QoSNetworkStats>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__TRAITS_HPP_
