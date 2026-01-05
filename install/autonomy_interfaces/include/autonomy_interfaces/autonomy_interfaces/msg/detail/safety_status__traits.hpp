// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/safety_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/safety_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'trigger_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SafetyStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_safe
  {
    out << "is_safe: ";
    rosidl_generator_traits::value_to_yaml(msg.is_safe, out);
    out << ", ";
  }

  // member: safety_level
  {
    out << "safety_level: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_level, out);
    out << ", ";
  }

  // member: active_triggers
  {
    if (msg.active_triggers.size() == 0) {
      out << "active_triggers: []";
    } else {
      out << "active_triggers: [";
      size_t pending_items = msg.active_triggers.size();
      for (auto item : msg.active_triggers) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: trigger_type
  {
    out << "trigger_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_type, out);
    out << ", ";
  }

  // member: trigger_source
  {
    out << "trigger_source: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_source, out);
    out << ", ";
  }

  // member: trigger_time
  {
    out << "trigger_time: ";
    to_flow_style_yaml(msg.trigger_time, out);
    out << ", ";
  }

  // member: trigger_description
  {
    out << "trigger_description: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_description, out);
    out << ", ";
  }

  // member: requires_manual_intervention
  {
    out << "requires_manual_intervention: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_manual_intervention, out);
    out << ", ";
  }

  // member: can_auto_recover
  {
    out << "can_auto_recover: ";
    rosidl_generator_traits::value_to_yaml(msg.can_auto_recover, out);
    out << ", ";
  }

  // member: recovery_steps
  {
    if (msg.recovery_steps.size() == 0) {
      out << "recovery_steps: []";
    } else {
      out << "recovery_steps: [";
      size_t pending_items = msg.recovery_steps.size();
      for (auto item : msg.recovery_steps) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: estimated_recovery_time
  {
    out << "estimated_recovery_time: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_recovery_time, out);
    out << ", ";
  }

  // member: context_state
  {
    out << "context_state: ";
    rosidl_generator_traits::value_to_yaml(msg.context_state, out);
    out << ", ";
  }

  // member: mission_phase
  {
    out << "mission_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_phase, out);
    out << ", ";
  }

  // member: safe_to_retry
  {
    out << "safe_to_retry: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_to_retry, out);
    out << ", ";
  }

  // member: battery_level
  {
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: communication_ok
  {
    out << "communication_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_ok, out);
    out << ", ";
  }

  // member: sensors_ok
  {
    out << "sensors_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_ok, out);
    out << ", ";
  }

  // member: degraded_capabilities
  {
    if (msg.degraded_capabilities.size() == 0) {
      out << "degraded_capabilities: []";
    } else {
      out << "degraded_capabilities: [";
      size_t pending_items = msg.degraded_capabilities.size();
      for (auto item : msg.degraded_capabilities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafetyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_safe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_safe: ";
    rosidl_generator_traits::value_to_yaml(msg.is_safe, out);
    out << "\n";
  }

  // member: safety_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_level: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_level, out);
    out << "\n";
  }

  // member: active_triggers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_triggers.size() == 0) {
      out << "active_triggers: []\n";
    } else {
      out << "active_triggers:\n";
      for (auto item : msg.active_triggers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: trigger_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_type, out);
    out << "\n";
  }

  // member: trigger_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_source: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_source, out);
    out << "\n";
  }

  // member: trigger_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_time:\n";
    to_block_style_yaml(msg.trigger_time, out, indentation + 2);
  }

  // member: trigger_description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_description: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_description, out);
    out << "\n";
  }

  // member: requires_manual_intervention
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requires_manual_intervention: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_manual_intervention, out);
    out << "\n";
  }

  // member: can_auto_recover
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_auto_recover: ";
    rosidl_generator_traits::value_to_yaml(msg.can_auto_recover, out);
    out << "\n";
  }

  // member: recovery_steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.recovery_steps.size() == 0) {
      out << "recovery_steps: []\n";
    } else {
      out << "recovery_steps:\n";
      for (auto item : msg.recovery_steps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: estimated_recovery_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_recovery_time: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_recovery_time, out);
    out << "\n";
  }

  // member: context_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "context_state: ";
    rosidl_generator_traits::value_to_yaml(msg.context_state, out);
    out << "\n";
  }

  // member: mission_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_phase, out);
    out << "\n";
  }

  // member: safe_to_retry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_to_retry: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_to_retry, out);
    out << "\n";
  }

  // member: battery_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: communication_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_ok, out);
    out << "\n";
  }

  // member: sensors_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_ok, out);
    out << "\n";
  }

  // member: degraded_capabilities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.degraded_capabilities.size() == 0) {
      out << "degraded_capabilities: []\n";
    } else {
      out << "degraded_capabilities:\n";
      for (auto item : msg.degraded_capabilities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafetyStatus & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::SafetyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::SafetyStatus & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::SafetyStatus>()
{
  return "autonomy_interfaces::msg::SafetyStatus";
}

template<>
inline const char * name<autonomy_interfaces::msg::SafetyStatus>()
{
  return "autonomy_interfaces/msg/SafetyStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::SafetyStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::SafetyStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::SafetyStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_
