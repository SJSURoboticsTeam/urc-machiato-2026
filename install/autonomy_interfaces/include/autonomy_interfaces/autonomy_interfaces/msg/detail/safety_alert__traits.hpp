// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_alert.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/safety_alert__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SafetyAlert & msg,
  std::ostream & out)
{
  out << "{";
  // member: property
  {
    out << "property: ";
    rosidl_generator_traits::value_to_yaml(msg.property, out);
    out << ", ";
  }

  // member: severity
  {
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << ", ";
  }

  // member: details
  {
    out << "details: ";
    rosidl_generator_traits::value_to_yaml(msg.details, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: acknowledged
  {
    out << "acknowledged: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledged, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafetyAlert & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: property
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "property: ";
    rosidl_generator_traits::value_to_yaml(msg.property, out);
    out << "\n";
  }

  // member: severity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << "\n";
  }

  // member: details
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "details: ";
    rosidl_generator_traits::value_to_yaml(msg.details, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: acknowledged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledged: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledged, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafetyAlert & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::SafetyAlert & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::SafetyAlert & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::SafetyAlert>()
{
  return "autonomy_interfaces::msg::SafetyAlert";
}

template<>
inline const char * name<autonomy_interfaces::msg::SafetyAlert>()
{
  return "autonomy_interfaces/msg/SafetyAlert";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::SafetyAlert>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::SafetyAlert>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::SafetyAlert>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_ALERT__TRAITS_HPP_
