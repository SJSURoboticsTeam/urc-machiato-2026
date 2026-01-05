// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_state.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/context_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ContextState & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_level
  {
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_critical
  {
    out << "battery_critical: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_critical, out);
    out << ", ";
  }

  // member: battery_warning
  {
    out << "battery_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_warning, out);
    out << ", ";
  }

  // member: mission_type
  {
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
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

  // member: mission_time_remaining
  {
    out << "mission_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_time_remaining, out);
    out << ", ";
  }

  // member: communication_active
  {
    out << "communication_active: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_active, out);
    out << ", ";
  }

  // member: communication_latency
  {
    out << "communication_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_latency, out);
    out << ", ";
  }

  // member: communication_quality
  {
    out << "communication_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_quality, out);
    out << ", ";
  }

  // member: cpu_usage
  {
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << ", ";
  }

  // member: memory_usage
  {
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: obstacle_detected
  {
    out << "obstacle_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_detected, out);
    out << ", ";
  }

  // member: obstacle_distance
  {
    out << "obstacle_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_distance, out);
    out << ", ";
  }

  // member: terrain_difficulty
  {
    out << "terrain_difficulty: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_difficulty, out);
    out << ", ";
  }

  // member: weather_adverse
  {
    out << "weather_adverse: ";
    rosidl_generator_traits::value_to_yaml(msg.weather_adverse, out);
    out << ", ";
  }

  // member: safety_active
  {
    out << "safety_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_active, out);
    out << ", ";
  }

  // member: safety_reason
  {
    out << "safety_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_reason, out);
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
  const ContextState & msg,
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

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_critical
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_critical: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_critical, out);
    out << "\n";
  }

  // member: battery_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_warning, out);
    out << "\n";
  }

  // member: mission_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
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

  // member: mission_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_time_remaining, out);
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

  // member: communication_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_latency, out);
    out << "\n";
  }

  // member: communication_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_quality, out);
    out << "\n";
  }

  // member: cpu_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << "\n";
  }

  // member: memory_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
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

  // member: obstacle_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_detected, out);
    out << "\n";
  }

  // member: obstacle_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_distance, out);
    out << "\n";
  }

  // member: terrain_difficulty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "terrain_difficulty: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_difficulty, out);
    out << "\n";
  }

  // member: weather_adverse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weather_adverse: ";
    rosidl_generator_traits::value_to_yaml(msg.weather_adverse, out);
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

  // member: safety_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_reason, out);
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

inline std::string to_yaml(const ContextState & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::ContextState & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::ContextState & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::ContextState>()
{
  return "autonomy_interfaces::msg::ContextState";
}

template<>
inline const char * name<autonomy_interfaces::msg::ContextState>()
{
  return "autonomy_interfaces/msg/ContextState";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::ContextState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::ContextState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::ContextState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__TRAITS_HPP_
