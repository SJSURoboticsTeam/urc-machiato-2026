// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/system_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'transition_timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SystemState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: current_state
  {
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << ", ";
  }

  // member: substate
  {
    out << "substate: ";
    rosidl_generator_traits::value_to_yaml(msg.substate, out);
    out << ", ";
  }

  // member: sub_substate
  {
    out << "sub_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_substate, out);
    out << ", ";
  }

  // member: time_in_state
  {
    out << "time_in_state: ";
    rosidl_generator_traits::value_to_yaml(msg.time_in_state, out);
    out << ", ";
  }

  // member: state_timeout
  {
    out << "state_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.state_timeout, out);
    out << ", ";
  }

  // member: previous_state
  {
    out << "previous_state: ";
    rosidl_generator_traits::value_to_yaml(msg.previous_state, out);
    out << ", ";
  }

  // member: transition_timestamp
  {
    out << "transition_timestamp: ";
    to_flow_style_yaml(msg.transition_timestamp, out);
    out << ", ";
  }

  // member: is_transitioning
  {
    out << "is_transitioning: ";
    rosidl_generator_traits::value_to_yaml(msg.is_transitioning, out);
    out << ", ";
  }

  // member: preconditions_met
  {
    out << "preconditions_met: ";
    rosidl_generator_traits::value_to_yaml(msg.preconditions_met, out);
    out << ", ";
  }

  // member: active_subsystems
  {
    if (msg.active_subsystems.size() == 0) {
      out << "active_subsystems: []";
    } else {
      out << "active_subsystems: [";
      size_t pending_items = msg.active_subsystems.size();
      for (auto item : msg.active_subsystems) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: failed_subsystems
  {
    if (msg.failed_subsystems.size() == 0) {
      out << "failed_subsystems: []";
    } else {
      out << "failed_subsystems: [";
      size_t pending_items = msg.failed_subsystems.size();
      for (auto item : msg.failed_subsystems) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mission_phase
  {
    out << "mission_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_phase, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << ", ";
  }

  // member: state_reason
  {
    out << "state_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.state_reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SystemState & msg,
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

  // member: current_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << "\n";
  }

  // member: substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "substate: ";
    rosidl_generator_traits::value_to_yaml(msg.substate, out);
    out << "\n";
  }

  // member: sub_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sub_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_substate, out);
    out << "\n";
  }

  // member: time_in_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_in_state: ";
    rosidl_generator_traits::value_to_yaml(msg.time_in_state, out);
    out << "\n";
  }

  // member: state_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.state_timeout, out);
    out << "\n";
  }

  // member: previous_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "previous_state: ";
    rosidl_generator_traits::value_to_yaml(msg.previous_state, out);
    out << "\n";
  }

  // member: transition_timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transition_timestamp:\n";
    to_block_style_yaml(msg.transition_timestamp, out, indentation + 2);
  }

  // member: is_transitioning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_transitioning: ";
    rosidl_generator_traits::value_to_yaml(msg.is_transitioning, out);
    out << "\n";
  }

  // member: preconditions_met
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "preconditions_met: ";
    rosidl_generator_traits::value_to_yaml(msg.preconditions_met, out);
    out << "\n";
  }

  // member: active_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_subsystems.size() == 0) {
      out << "active_subsystems: []\n";
    } else {
      out << "active_subsystems:\n";
      for (auto item : msg.active_subsystems) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: failed_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.failed_subsystems.size() == 0) {
      out << "failed_subsystems: []\n";
    } else {
      out << "failed_subsystems:\n";
      for (auto item : msg.failed_subsystems) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

  // member: operator_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << "\n";
  }

  // member: state_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.state_reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SystemState & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::SystemState & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::SystemState & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::SystemState>()
{
  return "autonomy_interfaces::msg::SystemState";
}

template<>
inline const char * name<autonomy_interfaces::msg::SystemState>()
{
  return "autonomy_interfaces/msg/SystemState";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::SystemState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::SystemState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::SystemState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_
