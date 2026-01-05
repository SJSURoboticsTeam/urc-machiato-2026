// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetAdaptationHistory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_adaptation_history.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_adaptation_history__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAdaptationHistory_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: limit
  {
    out << "limit: ";
    rosidl_generator_traits::value_to_yaml(msg.limit, out);
    out << ", ";
  }

  // member: include_context
  {
    out << "include_context: ";
    rosidl_generator_traits::value_to_yaml(msg.include_context, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAdaptationHistory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "limit: ";
    rosidl_generator_traits::value_to_yaml(msg.limit, out);
    out << "\n";
  }

  // member: include_context
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_context: ";
    rosidl_generator_traits::value_to_yaml(msg.include_context, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAdaptationHistory_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetAdaptationHistory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetAdaptationHistory_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAdaptationHistory_Request>()
{
  return "autonomy_interfaces::srv::GetAdaptationHistory_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAdaptationHistory_Request>()
{
  return "autonomy_interfaces/srv/GetAdaptationHistory_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::srv::GetAdaptationHistory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'actions'
#include "autonomy_interfaces/msg/detail/adaptive_action__traits.hpp"
// Member 'contexts'
#include "autonomy_interfaces/msg/detail/context_state__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAdaptationHistory_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: actions
  {
    if (msg.actions.size() == 0) {
      out << "actions: []";
    } else {
      out << "actions: [";
      size_t pending_items = msg.actions.size();
      for (auto item : msg.actions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: contexts
  {
    if (msg.contexts.size() == 0) {
      out << "contexts: []";
    } else {
      out << "contexts: [";
      size_t pending_items = msg.contexts.size();
      for (auto item : msg.contexts) {
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
  const GetAdaptationHistory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: actions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actions.size() == 0) {
      out << "actions: []\n";
    } else {
      out << "actions:\n";
      for (auto item : msg.actions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: contexts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.contexts.size() == 0) {
      out << "contexts: []\n";
    } else {
      out << "contexts:\n";
      for (auto item : msg.contexts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAdaptationHistory_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetAdaptationHistory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetAdaptationHistory_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAdaptationHistory_Response>()
{
  return "autonomy_interfaces::srv::GetAdaptationHistory_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAdaptationHistory_Response>()
{
  return "autonomy_interfaces/srv/GetAdaptationHistory_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetAdaptationHistory_Response>
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
  const GetAdaptationHistory_Event & msg,
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
  const GetAdaptationHistory_Event & msg,
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

inline std::string to_yaml(const GetAdaptationHistory_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetAdaptationHistory_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetAdaptationHistory_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAdaptationHistory_Event>()
{
  return "autonomy_interfaces::srv::GetAdaptationHistory_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAdaptationHistory_Event>()
{
  return "autonomy_interfaces/srv/GetAdaptationHistory_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Request>::value && has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::GetAdaptationHistory_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAdaptationHistory>()
{
  return "autonomy_interfaces::srv::GetAdaptationHistory";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAdaptationHistory>()
{
  return "autonomy_interfaces/srv/GetAdaptationHistory";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetAdaptationHistory_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetAdaptationHistory_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetAdaptationHistory>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetAdaptationHistory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetAdaptationHistory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_ADAPTATION_HISTORY__TRAITS_HPP_
