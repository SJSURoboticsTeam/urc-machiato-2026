// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetContext.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_context__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetContext_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetContext_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetContext_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetContext_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetContext_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetContext_Request>()
{
  return "autonomy_interfaces::srv::GetContext_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetContext_Request>()
{
  return "autonomy_interfaces/srv/GetContext_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetContext_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetContext_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonomy_interfaces::srv::GetContext_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'context'
#include "autonomy_interfaces/msg/detail/context_state__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetContext_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: context
  {
    out << "context: ";
    to_flow_style_yaml(msg.context, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetContext_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: context
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "context:\n";
    to_block_style_yaml(msg.context, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetContext_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetContext_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetContext_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetContext_Response>()
{
  return "autonomy_interfaces::srv::GetContext_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetContext_Response>()
{
  return "autonomy_interfaces/srv/GetContext_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetContext_Response>
  : std::integral_constant<bool, has_fixed_size<autonomy_interfaces::msg::ContextState>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetContext_Response>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::msg::ContextState>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::GetContext_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetContext>()
{
  return "autonomy_interfaces::srv::GetContext";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetContext>()
{
  return "autonomy_interfaces/srv/GetContext";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetContext>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetContext_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetContext_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetContext>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetContext_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetContext_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetContext>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetContext_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetContext_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__TRAITS_HPP_
