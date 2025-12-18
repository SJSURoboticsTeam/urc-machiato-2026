// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_system_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSystemState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: include_history
  {
    out << "include_history: ";
    rosidl_generator_traits::value_to_yaml(msg.include_history, out);
    out << ", ";
  }

  // member: include_subsystems
  {
    out << "include_subsystems: ";
    rosidl_generator_traits::value_to_yaml(msg.include_subsystems, out);
    out << ", ";
  }

  // member: history_limit
  {
    out << "history_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.history_limit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSystemState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: include_history
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_history: ";
    rosidl_generator_traits::value_to_yaml(msg.include_history, out);
    out << "\n";
  }

  // member: include_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_subsystems: ";
    rosidl_generator_traits::value_to_yaml(msg.include_subsystems, out);
    out << "\n";
  }

  // member: history_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "history_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.history_limit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSystemState_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::srv::GetSystemState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSystemState_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSystemState_Request>()
{
  return "autonomy_interfaces::srv::GetSystemState_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSystemState_Request>()
{
  return "autonomy_interfaces/srv/GetSystemState_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSystemState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSystemState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSystemState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'state_entered'
// Member 'state_timestamps'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSystemState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
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

  // member: state_entered
  {
    out << "state_entered: ";
    to_flow_style_yaml(msg.state_entered, out);
    out << ", ";
  }

  // member: recent_states
  {
    if (msg.recent_states.size() == 0) {
      out << "recent_states: []";
    } else {
      out << "recent_states: [";
      size_t pending_items = msg.recent_states.size();
      for (auto item : msg.recent_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: state_timestamps
  {
    if (msg.state_timestamps.size() == 0) {
      out << "state_timestamps: []";
    } else {
      out << "state_timestamps: [";
      size_t pending_items = msg.state_timestamps.size();
      for (auto item : msg.state_timestamps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: transition_reasons
  {
    if (msg.transition_reasons.size() == 0) {
      out << "transition_reasons: []";
    } else {
      out << "transition_reasons: [";
      size_t pending_items = msg.transition_reasons.size();
      for (auto item : msg.transition_reasons) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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

  // member: inactive_subsystems
  {
    if (msg.inactive_subsystems.size() == 0) {
      out << "inactive_subsystems: []";
    } else {
      out << "inactive_subsystems: [";
      size_t pending_items = msg.inactive_subsystems.size();
      for (auto item : msg.inactive_subsystems) {
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSystemState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
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

  // member: state_entered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_entered:\n";
    to_block_style_yaml(msg.state_entered, out, indentation + 2);
  }

  // member: recent_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.recent_states.size() == 0) {
      out << "recent_states: []\n";
    } else {
      out << "recent_states:\n";
      for (auto item : msg.recent_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: state_timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state_timestamps.size() == 0) {
      out << "state_timestamps: []\n";
    } else {
      out << "state_timestamps:\n";
      for (auto item : msg.state_timestamps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: transition_reasons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.transition_reasons.size() == 0) {
      out << "transition_reasons: []\n";
    } else {
      out << "transition_reasons:\n";
      for (auto item : msg.transition_reasons) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

  // member: inactive_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.inactive_subsystems.size() == 0) {
      out << "inactive_subsystems: []\n";
    } else {
      out << "inactive_subsystems:\n";
      for (auto item : msg.inactive_subsystems) {
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSystemState_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::srv::GetSystemState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSystemState_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSystemState_Response>()
{
  return "autonomy_interfaces::srv::GetSystemState_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSystemState_Response>()
{
  return "autonomy_interfaces/srv/GetSystemState_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSystemState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSystemState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSystemState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSystemState>()
{
  return "autonomy_interfaces::srv::GetSystemState";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSystemState>()
{
  return "autonomy_interfaces/srv/GetSystemState";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSystemState>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetSystemState_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetSystemState_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSystemState>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetSystemState_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetSystemState_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetSystemState>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetSystemState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetSystemState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__TRAITS_HPP_
