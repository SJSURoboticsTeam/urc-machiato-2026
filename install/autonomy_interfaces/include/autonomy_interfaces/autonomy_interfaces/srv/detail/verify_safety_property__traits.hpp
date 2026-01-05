// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/VerifySafetyProperty.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/verify_safety_property.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/verify_safety_property__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const VerifySafetyProperty_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: property_name
  {
    out << "property_name: ";
    rosidl_generator_traits::value_to_yaml(msg.property_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VerifySafetyProperty_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: property_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "property_name: ";
    rosidl_generator_traits::value_to_yaml(msg.property_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VerifySafetyProperty_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::VerifySafetyProperty_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::VerifySafetyProperty_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::VerifySafetyProperty_Request>()
{
  return "autonomy_interfaces::srv::VerifySafetyProperty_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::VerifySafetyProperty_Request>()
{
  return "autonomy_interfaces/srv/VerifySafetyProperty_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::VerifySafetyProperty_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const VerifySafetyProperty_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: property_name
  {
    out << "property_name: ";
    rosidl_generator_traits::value_to_yaml(msg.property_name, out);
    out << ", ";
  }

  // member: satisfied
  {
    out << "satisfied: ";
    rosidl_generator_traits::value_to_yaml(msg.satisfied, out);
    out << ", ";
  }

  // member: details
  {
    out << "details: ";
    rosidl_generator_traits::value_to_yaml(msg.details, out);
    out << ", ";
  }

  // member: violation_count
  {
    out << "violation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.violation_count, out);
    out << ", ";
  }

  // member: last_violation
  {
    out << "last_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.last_violation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VerifySafetyProperty_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: property_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "property_name: ";
    rosidl_generator_traits::value_to_yaml(msg.property_name, out);
    out << "\n";
  }

  // member: satisfied
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "satisfied: ";
    rosidl_generator_traits::value_to_yaml(msg.satisfied, out);
    out << "\n";
  }

  // member: details
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "details: ";
    rosidl_generator_traits::value_to_yaml(msg.details, out);
    out << "\n";
  }

  // member: violation_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "violation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.violation_count, out);
    out << "\n";
  }

  // member: last_violation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.last_violation, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VerifySafetyProperty_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::VerifySafetyProperty_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::VerifySafetyProperty_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::VerifySafetyProperty_Response>()
{
  return "autonomy_interfaces::srv::VerifySafetyProperty_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::VerifySafetyProperty_Response>()
{
  return "autonomy_interfaces/srv/VerifySafetyProperty_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::VerifySafetyProperty_Response>
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
  const VerifySafetyProperty_Event & msg,
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
  const VerifySafetyProperty_Event & msg,
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

inline std::string to_yaml(const VerifySafetyProperty_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::VerifySafetyProperty_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::VerifySafetyProperty_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::VerifySafetyProperty_Event>()
{
  return "autonomy_interfaces::srv::VerifySafetyProperty_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::VerifySafetyProperty_Event>()
{
  return "autonomy_interfaces/srv/VerifySafetyProperty_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Request>::value && has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::VerifySafetyProperty_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::VerifySafetyProperty>()
{
  return "autonomy_interfaces::srv::VerifySafetyProperty";
}

template<>
inline const char * name<autonomy_interfaces::srv::VerifySafetyProperty>()
{
  return "autonomy_interfaces/srv/VerifySafetyProperty";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::VerifySafetyProperty_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::VerifySafetyProperty_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::VerifySafetyProperty>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::VerifySafetyProperty_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::VerifySafetyProperty_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__TRAITS_HPP_
