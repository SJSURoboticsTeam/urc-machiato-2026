// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/software_estop__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SoftwareEstop_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: acknowledge_criticality
  {
    out << "acknowledge_criticality: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_criticality, out);
    out << ", ";
  }

  // member: force_immediate
  {
    out << "force_immediate: ";
    rosidl_generator_traits::value_to_yaml(msg.force_immediate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SoftwareEstop_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: operator_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
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

  // member: acknowledge_criticality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledge_criticality: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_criticality, out);
    out << "\n";
  }

  // member: force_immediate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force_immediate: ";
    rosidl_generator_traits::value_to_yaml(msg.force_immediate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SoftwareEstop_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::SoftwareEstop_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::SoftwareEstop_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::SoftwareEstop_Request>()
{
  return "autonomy_interfaces::srv::SoftwareEstop_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::SoftwareEstop_Request>()
{
  return "autonomy_interfaces/srv/SoftwareEstop_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SoftwareEstop_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SoftwareEstop_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::SoftwareEstop_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SoftwareEstop_Response & msg,
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

  // member: estop_id
  {
    out << "estop_id: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_id, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: triggered_by
  {
    out << "triggered_by: ";
    rosidl_generator_traits::value_to_yaml(msg.triggered_by, out);
    out << ", ";
  }

  // member: coordination_started
  {
    out << "coordination_started: ";
    rosidl_generator_traits::value_to_yaml(msg.coordination_started, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SoftwareEstop_Response & msg,
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

  // member: estop_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop_id: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_id, out);
    out << "\n";
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

  // member: triggered_by
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "triggered_by: ";
    rosidl_generator_traits::value_to_yaml(msg.triggered_by, out);
    out << "\n";
  }

  // member: coordination_started
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coordination_started: ";
    rosidl_generator_traits::value_to_yaml(msg.coordination_started, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SoftwareEstop_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::SoftwareEstop_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::SoftwareEstop_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::SoftwareEstop_Response>()
{
  return "autonomy_interfaces::srv::SoftwareEstop_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::SoftwareEstop_Response>()
{
  return "autonomy_interfaces/srv/SoftwareEstop_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SoftwareEstop_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SoftwareEstop_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::SoftwareEstop_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::SoftwareEstop>()
{
  return "autonomy_interfaces::srv::SoftwareEstop";
}

template<>
inline const char * name<autonomy_interfaces::srv::SoftwareEstop>()
{
  return "autonomy_interfaces/srv/SoftwareEstop";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::SoftwareEstop>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::SoftwareEstop_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::SoftwareEstop_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::SoftwareEstop>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::SoftwareEstop_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::SoftwareEstop_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::SoftwareEstop>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::SoftwareEstop_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::SoftwareEstop_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__TRAITS_HPP_
