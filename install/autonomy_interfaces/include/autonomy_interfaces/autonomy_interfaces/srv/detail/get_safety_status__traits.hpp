// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_safety_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_safety_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafetyStatus_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafetyStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafetyStatus_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSafetyStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSafetyStatus_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSafetyStatus_Request>()
{
  return "autonomy_interfaces::srv::GetSafetyStatus_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSafetyStatus_Request>()
{
  return "autonomy_interfaces/srv/GetSafetyStatus_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSafetyStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'active_alerts'
#include "autonomy_interfaces/msg/detail/safety_alert__traits.hpp"
// Member 'monitoring_stats'
#include "autonomy_interfaces/msg/detail/monitoring_stats__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafetyStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: overall_safety
  {
    out << "overall_safety: ";
    rosidl_generator_traits::value_to_yaml(msg.overall_safety, out);
    out << ", ";
  }

  // member: active_alerts
  {
    if (msg.active_alerts.size() == 0) {
      out << "active_alerts: []";
    } else {
      out << "active_alerts: [";
      size_t pending_items = msg.active_alerts.size();
      for (auto item : msg.active_alerts) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: monitoring_stats
  {
    out << "monitoring_stats: ";
    to_flow_style_yaml(msg.monitoring_stats, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafetyStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: overall_safety
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overall_safety: ";
    rosidl_generator_traits::value_to_yaml(msg.overall_safety, out);
    out << "\n";
  }

  // member: active_alerts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_alerts.size() == 0) {
      out << "active_alerts: []\n";
    } else {
      out << "active_alerts:\n";
      for (auto item : msg.active_alerts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: monitoring_stats
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "monitoring_stats:\n";
    to_block_style_yaml(msg.monitoring_stats, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafetyStatus_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSafetyStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSafetyStatus_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSafetyStatus_Response>()
{
  return "autonomy_interfaces::srv::GetSafetyStatus_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSafetyStatus_Response>()
{
  return "autonomy_interfaces/srv/GetSafetyStatus_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSafetyStatus_Response>
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
  const GetSafetyStatus_Event & msg,
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
  const GetSafetyStatus_Event & msg,
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

inline std::string to_yaml(const GetSafetyStatus_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetSafetyStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetSafetyStatus_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSafetyStatus_Event>()
{
  return "autonomy_interfaces::srv::GetSafetyStatus_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSafetyStatus_Event>()
{
  return "autonomy_interfaces/srv/GetSafetyStatus_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Request>::value && has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::GetSafetyStatus_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetSafetyStatus>()
{
  return "autonomy_interfaces::srv::GetSafetyStatus";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetSafetyStatus>()
{
  return "autonomy_interfaces/srv/GetSafetyStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetSafetyStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetSafetyStatus_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetSafetyStatus>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetSafetyStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetSafetyStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__TRAITS_HPP_
