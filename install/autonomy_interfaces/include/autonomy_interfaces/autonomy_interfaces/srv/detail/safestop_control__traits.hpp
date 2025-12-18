// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/SafestopControl.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/safestop_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SafestopControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << ", ";
  }

  // member: acknowledge_risks
  {
    out << "acknowledge_risks: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_risks, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: affected_subsystems
  {
    if (msg.affected_subsystems.size() == 0) {
      out << "affected_subsystems: []";
    } else {
      out << "affected_subsystems: [";
      size_t pending_items = msg.affected_subsystems.size();
      for (auto item : msg.affected_subsystems) {
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
  const SafestopControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
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

  // member: acknowledge_risks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledge_risks: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_risks, out);
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

  // member: affected_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.affected_subsystems.size() == 0) {
      out << "affected_subsystems: []\n";
    } else {
      out << "affected_subsystems:\n";
      for (auto item : msg.affected_subsystems) {
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

inline std::string to_yaml(const SafestopControl_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::SafestopControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::SafestopControl_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::SafestopControl_Request>()
{
  return "autonomy_interfaces::srv::SafestopControl_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::SafestopControl_Request>()
{
  return "autonomy_interfaces/srv/SafestopControl_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SafestopControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SafestopControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::SafestopControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SafestopControl_Response & msg,
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

  // member: affected_subsystems
  {
    if (msg.affected_subsystems.size() == 0) {
      out << "affected_subsystems: []";
    } else {
      out << "affected_subsystems: [";
      size_t pending_items = msg.affected_subsystems.size();
      for (auto item : msg.affected_subsystems) {
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
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafestopControl_Response & msg,
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

  // member: affected_subsystems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.affected_subsystems.size() == 0) {
      out << "affected_subsystems: []\n";
    } else {
      out << "affected_subsystems:\n";
      for (auto item : msg.affected_subsystems) {
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
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafestopControl_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::SafestopControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::SafestopControl_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::SafestopControl_Response>()
{
  return "autonomy_interfaces::srv::SafestopControl_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::SafestopControl_Response>()
{
  return "autonomy_interfaces/srv/SafestopControl_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SafestopControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SafestopControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::SafestopControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::SafestopControl>()
{
  return "autonomy_interfaces::srv::SafestopControl";
}

template<>
inline const char * name<autonomy_interfaces::srv::SafestopControl>()
{
  return "autonomy_interfaces/srv/SafestopControl";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SafestopControl>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::SafestopControl_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::SafestopControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SafestopControl>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::SafestopControl_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::SafestopControl_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::SafestopControl>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::SafestopControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::SafestopControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__TRAITS_HPP_
