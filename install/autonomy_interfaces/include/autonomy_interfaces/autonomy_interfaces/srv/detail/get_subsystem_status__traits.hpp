// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetSubsystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_subsystem_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_subsystem_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSubsystemStatus_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: subsystem_name
  {
    out << "subsystem_name: ";
    rosidl_generator_traits::value_to_yaml(msg.subsystem_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSubsystemStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: subsystem_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subsystem_name: ";
    rosidl_generator_traits::value_to_yaml(msg.subsystem_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSubsystemStatus_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSubsystemStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSubsystemStatus_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSubsystemStatus_Request>()
{
  return "autonomy_interfaces::srv::GetSubsystemStatus_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSubsystemStatus_Request>()
{
  return "autonomy_interfaces/srv/GetSubsystemStatus_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSubsystemStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSubsystemStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: subsystem_names
  {
    if (msg.subsystem_names.size() == 0) {
      out << "subsystem_names: []";
    } else {
      out << "subsystem_names: [";
      size_t pending_items = msg.subsystem_names.size();
      for (auto item : msg.subsystem_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: subsystem_states
  {
    if (msg.subsystem_states.size() == 0) {
      out << "subsystem_states: []";
    } else {
      out << "subsystem_states: [";
      size_t pending_items = msg.subsystem_states.size();
      for (auto item : msg.subsystem_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: subsystem_health
  {
    if (msg.subsystem_health.size() == 0) {
      out << "subsystem_health: []";
    } else {
      out << "subsystem_health: [";
      size_t pending_items = msg.subsystem_health.size();
      for (auto item : msg.subsystem_health) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: status_messages
  {
    if (msg.status_messages.size() == 0) {
      out << "status_messages: []";
    } else {
      out << "status_messages: [";
      size_t pending_items = msg.status_messages.size();
      for (auto item : msg.status_messages) {
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
  const GetSubsystemStatus_Response & msg,
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

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }

  // member: subsystem_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.subsystem_names.size() == 0) {
      out << "subsystem_names: []\n";
    } else {
      out << "subsystem_names:\n";
      for (auto item : msg.subsystem_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: subsystem_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.subsystem_states.size() == 0) {
      out << "subsystem_states: []\n";
    } else {
      out << "subsystem_states:\n";
      for (auto item : msg.subsystem_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: subsystem_health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.subsystem_health.size() == 0) {
      out << "subsystem_health: []\n";
    } else {
      out << "subsystem_health:\n";
      for (auto item : msg.subsystem_health) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: status_messages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.status_messages.size() == 0) {
      out << "status_messages: []\n";
    } else {
      out << "status_messages:\n";
      for (auto item : msg.status_messages) {
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

inline std::string to_yaml(const GetSubsystemStatus_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSubsystemStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSubsystemStatus_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSubsystemStatus_Response>()
{
  return "autonomy_interfaces::srv::GetSubsystemStatus_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSubsystemStatus_Response>()
{
  return "autonomy_interfaces/srv/GetSubsystemStatus_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSubsystemStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSubsystemStatus_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const GetSubsystemStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSubsystemStatus_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSubsystemStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSubsystemStatus_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSubsystemStatus_Event>()
{
  return "autonomy_interfaces::srv::GetSubsystemStatus_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSubsystemStatus_Event>()
{
  return "autonomy_interfaces/srv/GetSubsystemStatus_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Request>::value && has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSubsystemStatus_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSubsystemStatus>()
{
  return "autonomy_interfaces::srv::GetSubsystemStatus";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSubsystemStatus>()
{
  return "autonomy_interfaces/srv/GetSubsystemStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetSubsystemStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetSubsystemStatus_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetSubsystemStatus>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetSubsystemStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetSubsystemStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__TRAITS_HPP_
