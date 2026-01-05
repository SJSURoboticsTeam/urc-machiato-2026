// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/FollowMeStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/follow_me_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/follow_me_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FollowMeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_following
  {
    out << "is_following: ";
    rosidl_generator_traits::value_to_yaml(msg.is_following, out);
    out << ", ";
  }

  // member: target_tag_id
  {
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << ", ";
  }

  // member: target_distance
  {
    out << "target_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.target_distance, out);
    out << ", ";
  }

  // member: target_angle
  {
    out << "target_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.target_angle, out);
    out << ", ";
  }

  // member: safety_distance
  {
    out << "safety_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_distance, out);
    out << ", ";
  }

  // member: safety_violation
  {
    out << "safety_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_violation, out);
    out << ", ";
  }

  // member: current_speed
  {
    out << "current_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.current_speed, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    to_flow_style_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: target_visible
  {
    out << "target_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.target_visible, out);
    out << ", ";
  }

  // member: last_detection_time
  {
    out << "last_detection_time: ";
    rosidl_generator_traits::value_to_yaml(msg.last_detection_time, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
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
  const FollowMeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
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

  // member: target_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << "\n";
  }

  // member: target_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.target_distance, out);
    out << "\n";
  }

  // member: target_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.target_angle, out);
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

  // member: safety_violation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_violation, out);
    out << "\n";
  }

  // member: current_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.current_speed, out);
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

  // member: target_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.target_visible, out);
    out << "\n";
  }

  // member: last_detection_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_detection_time: ";
    rosidl_generator_traits::value_to_yaml(msg.last_detection_time, out);
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

inline std::string to_yaml(const FollowMeStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::msg::FollowMeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::FollowMeStatus & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::FollowMeStatus>()
{
  return "autonomy_interfaces::msg::FollowMeStatus";
}

template<>
inline const char * name<autonomy_interfaces::msg::FollowMeStatus>()
{
  return "autonomy_interfaces/msg/FollowMeStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::FollowMeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::FollowMeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::FollowMeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__FOLLOW_ME_STATUS__TRAITS_HPP_
