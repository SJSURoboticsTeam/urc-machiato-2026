// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_update.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/context_update__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ContextUpdate & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_level
  {
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << ", ";
  }

  // member: mission_status
  {
    out << "mission_status: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_status, out);
    out << ", ";
  }

  // member: mission_progress
  {
    out << "mission_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_progress, out);
    out << ", ";
  }

  // member: communication_active
  {
    out << "communication_active: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_active, out);
    out << ", ";
  }

  // member: safety_active
  {
    out << "safety_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_active, out);
    out << ", ";
  }

  // member: active_adaptations
  {
    if (msg.active_adaptations.size() == 0) {
      out << "active_adaptations: []";
    } else {
      out << "active_adaptations: [";
      size_t pending_items = msg.active_adaptations.size();
      for (auto item : msg.active_adaptations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: alert_level
  {
    out << "alert_level: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_level, out);
    out << ", ";
  }

  // member: available_actions
  {
    if (msg.available_actions.size() == 0) {
      out << "available_actions: []";
    } else {
      out << "available_actions: [";
      size_t pending_items = msg.available_actions.size();
      for (auto item : msg.available_actions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const ContextUpdate & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: battery_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << "\n";
  }

  // member: mission_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_status: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_status, out);
    out << "\n";
  }

  // member: mission_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_progress, out);
    out << "\n";
  }

  // member: communication_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_active: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_active, out);
    out << "\n";
  }

  // member: safety_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_active, out);
    out << "\n";
  }

  // member: active_adaptations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_adaptations.size() == 0) {
      out << "active_adaptations: []\n";
    } else {
      out << "active_adaptations:\n";
      for (auto item : msg.active_adaptations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: alert_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alert_level: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_level, out);
    out << "\n";
  }

  // member: available_actions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.available_actions.size() == 0) {
      out << "available_actions: []\n";
    } else {
      out << "available_actions:\n";
      for (auto item : msg.available_actions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const ContextUpdate & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::ContextUpdate & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::ContextUpdate & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::ContextUpdate>()
{
  return "autonomy_interfaces::msg::ContextUpdate";
}

template<>
inline const char * name<autonomy_interfaces::msg::ContextUpdate>()
{
  return "autonomy_interfaces/msg/ContextUpdate";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::ContextUpdate>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::ContextUpdate>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::ContextUpdate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__TRAITS_HPP_
