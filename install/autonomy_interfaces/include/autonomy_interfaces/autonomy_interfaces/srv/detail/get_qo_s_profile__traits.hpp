// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetQoSProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_qo_s_profile.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_qo_s_profile__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetQoSProfile_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetQoSProfile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetQoSProfile_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetQoSProfile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetQoSProfile_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetQoSProfile_Request>()
{
  return "autonomy_interfaces::srv::GetQoSProfile_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetQoSProfile_Request>()
{
  return "autonomy_interfaces/srv/GetQoSProfile_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetQoSProfile_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::srv::GetQoSProfile_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'topic_profiles'
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__traits.hpp"
// Member 'network_stats'
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetQoSProfile_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_profiles
  {
    if (msg.topic_profiles.size() == 0) {
      out << "topic_profiles: []";
    } else {
      out << "topic_profiles: [";
      size_t pending_items = msg.topic_profiles.size();
      for (auto item : msg.topic_profiles) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: network_stats
  {
    out << "network_stats: ";
    to_flow_style_yaml(msg.network_stats, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetQoSProfile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_profiles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topic_profiles.size() == 0) {
      out << "topic_profiles: []\n";
    } else {
      out << "topic_profiles:\n";
      for (auto item : msg.topic_profiles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: network_stats
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "network_stats:\n";
    to_block_style_yaml(msg.network_stats, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetQoSProfile_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetQoSProfile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetQoSProfile_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetQoSProfile_Response>()
{
  return "autonomy_interfaces::srv::GetQoSProfile_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetQoSProfile_Response>()
{
  return "autonomy_interfaces/srv/GetQoSProfile_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetQoSProfile_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetQoSProfile_Response>
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
  const GetQoSProfile_Event & msg,
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
  const GetQoSProfile_Event & msg,
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

inline std::string to_yaml(const GetQoSProfile_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetQoSProfile_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetQoSProfile_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetQoSProfile_Event>()
{
  return "autonomy_interfaces::srv::GetQoSProfile_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetQoSProfile_Event>()
{
  return "autonomy_interfaces/srv/GetQoSProfile_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetQoSProfile_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Request>::value && has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::GetQoSProfile_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetQoSProfile>()
{
  return "autonomy_interfaces::srv::GetQoSProfile";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetQoSProfile>()
{
  return "autonomy_interfaces/srv/GetQoSProfile";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetQoSProfile>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetQoSProfile_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetQoSProfile_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetQoSProfile>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetQoSProfile_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetQoSProfile>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetQoSProfile_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetQoSProfile_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__TRAITS_HPP_
