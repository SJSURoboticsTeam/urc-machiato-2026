// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_topic_profile.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const QoSTopicProfile & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_name
  {
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << ", ";
  }

  // member: avg_latency_ms
  {
    out << "avg_latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_latency_ms, out);
    out << ", ";
  }

  // member: jitter_ms
  {
    out << "jitter_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.jitter_ms, out);
    out << ", ";
  }

  // member: packet_loss_rate
  {
    out << "packet_loss_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.packet_loss_rate, out);
    out << ", ";
  }

  // member: samples_count
  {
    out << "samples_count: ";
    rosidl_generator_traits::value_to_yaml(msg.samples_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const QoSTopicProfile & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << "\n";
  }

  // member: avg_latency_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_latency_ms, out);
    out << "\n";
  }

  // member: jitter_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jitter_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.jitter_ms, out);
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

  // member: samples_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "samples_count: ";
    rosidl_generator_traits::value_to_yaml(msg.samples_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const QoSTopicProfile & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::QoSTopicProfile & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::QoSTopicProfile & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::QoSTopicProfile>()
{
  return "autonomy_interfaces::msg::QoSTopicProfile";
}

template<>
inline const char * name<autonomy_interfaces::msg::QoSTopicProfile>()
{
  return "autonomy_interfaces/msg/QoSTopicProfile";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::QoSTopicProfile>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::QoSTopicProfile>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::QoSTopicProfile>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__TRAITS_HPP_
