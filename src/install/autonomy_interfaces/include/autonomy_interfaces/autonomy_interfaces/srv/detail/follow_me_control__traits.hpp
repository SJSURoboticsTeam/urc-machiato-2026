// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/follow_me_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FollowMeControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_tag_id
  {
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << ", ";
  }

  // member: safety_distance
  {
    out << "safety_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_distance, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << ", ";
  }

  // member: enable_following
  {
    out << "enable_following: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_following, out);
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
  const FollowMeControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << "\n";
  }

  // member: safety_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_distance, out);
    out << "\n";
  }

  // member: max_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << "\n";
  }

  // member: enable_following
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_following: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_following, out);
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

inline std::string to_yaml(const FollowMeControl_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::FollowMeControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::FollowMeControl_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::FollowMeControl_Request>()
{
  return "autonomy_interfaces::srv::FollowMeControl_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::FollowMeControl_Request>()
{
  return "autonomy_interfaces/srv/FollowMeControl_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::FollowMeControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::FollowMeControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::FollowMeControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FollowMeControl_Response & msg,
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

  // member: is_following
  {
    out << "is_following: ";
    rosidl_generator_traits::value_to_yaml(msg.is_following, out);
    out << ", ";
  }

  // member: current_target_tag
  {
    out << "current_target_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.current_target_tag, out);
    out << ", ";
  }

  // member: current_distance
  {
    out << "current_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.current_distance, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    to_flow_style_yaml(msg.target_position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FollowMeControl_Response & msg,
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

  // member: is_following
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_following: ";
    rosidl_generator_traits::value_to_yaml(msg.is_following, out);
    out << "\n";
  }

  // member: current_target_tag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_target_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.current_target_tag, out);
    out << "\n";
  }

  // member: current_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.current_distance, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position:\n";
    to_block_style_yaml(msg.target_position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FollowMeControl_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::FollowMeControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::FollowMeControl_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::FollowMeControl_Response>()
{
  return "autonomy_interfaces::srv::FollowMeControl_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::FollowMeControl_Response>()
{
  return "autonomy_interfaces/srv/FollowMeControl_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::FollowMeControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::FollowMeControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::FollowMeControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::FollowMeControl>()
{
  return "autonomy_interfaces::srv::FollowMeControl";
}

template<>
inline const char * name<autonomy_interfaces::srv::FollowMeControl>()
{
  return "autonomy_interfaces/srv/FollowMeControl";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::FollowMeControl>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::FollowMeControl_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::FollowMeControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::FollowMeControl>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::FollowMeControl_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::FollowMeControl_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::FollowMeControl>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::FollowMeControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::FollowMeControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__TRAITS_HPP_
