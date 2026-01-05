// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/monitoring_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/monitoring_stats__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MonitoringStats & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_evaluations
  {
    out << "total_evaluations: ";
    rosidl_generator_traits::value_to_yaml(msg.total_evaluations, out);
    out << ", ";
  }

  // member: total_violations
  {
    out << "total_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.total_violations, out);
    out << ", ";
  }

  // member: evaluation_rate
  {
    out << "evaluation_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.evaluation_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MonitoringStats & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_evaluations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_evaluations: ";
    rosidl_generator_traits::value_to_yaml(msg.total_evaluations, out);
    out << "\n";
  }

  // member: total_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.total_violations, out);
    out << "\n";
  }

  // member: evaluation_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "evaluation_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.evaluation_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MonitoringStats & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::MonitoringStats & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::MonitoringStats & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::MonitoringStats>()
{
  return "autonomy_interfaces::msg::MonitoringStats";
}

template<>
inline const char * name<autonomy_interfaces::msg::MonitoringStats>()
{
  return "autonomy_interfaces/msg/MonitoringStats";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::MonitoringStats>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::MonitoringStats>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::msg::MonitoringStats>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__TRAITS_HPP_
