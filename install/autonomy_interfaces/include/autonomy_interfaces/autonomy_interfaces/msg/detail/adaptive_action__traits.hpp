// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/adaptive_action.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/adaptive_action__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'trigger_context'
#include "autonomy_interfaces/msg/detail/context_state__traits.hpp"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AdaptiveAction & msg,
  std::ostream & out)
{
  out << "{";
  // member: action_type
  {
    out << "action_type: ";
    rosidl_generator_traits::value_to_yaml(msg.action_type, out);
    out << ", ";
  }

  // member: parameters
  {
    if (msg.parameters.size() == 0) {
      out << "parameters: []";
    } else {
      out << "parameters: [";
      size_t pending_items = msg.parameters.size();
      for (auto item : msg.parameters) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: trigger_context
  {
    out << "trigger_context: ";
    to_flow_style_yaml(msg.trigger_context, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << ", ";
  }

  // member: expected_duration
  {
    out << "expected_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_duration, out);
    out << ", ";
  }

  // member: success_criteria
  {
    out << "success_criteria: ";
    rosidl_generator_traits::value_to_yaml(msg.success_criteria, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AdaptiveAction & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_type: ";
    rosidl_generator_traits::value_to_yaml(msg.action_type, out);
    out << "\n";
  }

  // member: parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.parameters.size() == 0) {
      out << "parameters: []\n";
    } else {
      out << "parameters:\n";
      for (auto item : msg.parameters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: trigger_context
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_context:\n";
    to_block_style_yaml(msg.trigger_context, out, indentation + 2);
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << "\n";
  }

  // member: expected_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expected_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_duration, out);
    out << "\n";
  }

  // member: success_criteria
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success_criteria: ";
    rosidl_generator_traits::value_to_yaml(msg.success_criteria, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AdaptiveAction & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::AdaptiveAction & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::AdaptiveAction & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::AdaptiveAction>()
{
  return "autonomy_interfaces::msg::AdaptiveAction";
}

template<>
inline const char * name<autonomy_interfaces::msg::AdaptiveAction>()
{
  return "autonomy_interfaces/msg/AdaptiveAction";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::AdaptiveAction>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::AdaptiveAction>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::AdaptiveAction>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__TRAITS_HPP_
