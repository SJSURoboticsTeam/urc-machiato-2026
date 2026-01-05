// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/state_transition.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/state_transition__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'start_time'
// Member 'end_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StateTransition & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: from_state
  {
    out << "from_state: ";
    rosidl_generator_traits::value_to_yaml(msg.from_state, out);
    out << ", ";
  }

  // member: to_state
  {
    out << "to_state: ";
    rosidl_generator_traits::value_to_yaml(msg.to_state, out);
    out << ", ";
  }

  // member: start_time
  {
    out << "start_time: ";
    to_flow_style_yaml(msg.start_time, out);
    out << ", ";
  }

  // member: end_time
  {
    out << "end_time: ";
    to_flow_style_yaml(msg.end_time, out);
    out << ", ";
  }

  // member: transition_duration
  {
    out << "transition_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_duration, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: initiated_by
  {
    out << "initiated_by: ";
    rosidl_generator_traits::value_to_yaml(msg.initiated_by, out);
    out << ", ";
  }

  // member: failure_reason
  {
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << ", ";
  }

  // member: preconditions_checked
  {
    if (msg.preconditions_checked.size() == 0) {
      out << "preconditions_checked: []";
    } else {
      out << "preconditions_checked: [";
      size_t pending_items = msg.preconditions_checked.size();
      for (auto item : msg.preconditions_checked) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: entry_actions_executed
  {
    if (msg.entry_actions_executed.size() == 0) {
      out << "entry_actions_executed: []";
    } else {
      out << "entry_actions_executed: [";
      size_t pending_items = msg.entry_actions_executed.size();
      for (auto item : msg.entry_actions_executed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: exit_actions_executed
  {
    if (msg.exit_actions_executed.size() == 0) {
      out << "exit_actions_executed: []";
    } else {
      out << "exit_actions_executed: [";
      size_t pending_items = msg.exit_actions_executed.size();
      for (auto item : msg.exit_actions_executed) {
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
  const StateTransition & msg,
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

  // member: from_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_state: ";
    rosidl_generator_traits::value_to_yaml(msg.from_state, out);
    out << "\n";
  }

  // member: to_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "to_state: ";
    rosidl_generator_traits::value_to_yaml(msg.to_state, out);
    out << "\n";
  }

  // member: start_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_time:\n";
    to_block_style_yaml(msg.start_time, out, indentation + 2);
  }

  // member: end_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_time:\n";
    to_block_style_yaml(msg.end_time, out, indentation + 2);
  }

  // member: transition_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transition_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_duration, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: initiated_by
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initiated_by: ";
    rosidl_generator_traits::value_to_yaml(msg.initiated_by, out);
    out << "\n";
  }

  // member: failure_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << "\n";
  }

  // member: preconditions_checked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.preconditions_checked.size() == 0) {
      out << "preconditions_checked: []\n";
    } else {
      out << "preconditions_checked:\n";
      for (auto item : msg.preconditions_checked) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: entry_actions_executed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.entry_actions_executed.size() == 0) {
      out << "entry_actions_executed: []\n";
    } else {
      out << "entry_actions_executed:\n";
      for (auto item : msg.entry_actions_executed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: exit_actions_executed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.exit_actions_executed.size() == 0) {
      out << "exit_actions_executed: []\n";
    } else {
      out << "exit_actions_executed:\n";
      for (auto item : msg.exit_actions_executed) {
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

inline std::string to_yaml(const StateTransition & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::StateTransition & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::StateTransition & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::StateTransition>()
{
  return "autonomy_interfaces::msg::StateTransition";
}

template<>
inline const char * name<autonomy_interfaces::msg::StateTransition>()
{
  return "autonomy_interfaces/msg/StateTransition";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::StateTransition>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::StateTransition>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::StateTransition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__TRAITS_HPP_
