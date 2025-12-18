// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/typing_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'keyboard_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'key_dimensions'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TypingGoal & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: target_text
  {
    out << "target_text: ";
    rosidl_generator_traits::value_to_yaml(msg.target_text, out);
    out << ", ";
  }

  // member: keyboard_pose
  {
    out << "keyboard_pose: ";
    to_flow_style_yaml(msg.keyboard_pose, out);
    out << ", ";
  }

  // member: accuracy_requirement
  {
    out << "accuracy_requirement: ";
    rosidl_generator_traits::value_to_yaml(msg.accuracy_requirement, out);
    out << ", ";
  }

  // member: speed_requirement
  {
    out << "speed_requirement: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_requirement, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: keyboard_layout
  {
    out << "keyboard_layout: ";
    rosidl_generator_traits::value_to_yaml(msg.keyboard_layout, out);
    out << ", ";
  }

  // member: key_spacing_m
  {
    out << "key_spacing_m: ";
    rosidl_generator_traits::value_to_yaml(msg.key_spacing_m, out);
    out << ", ";
  }

  // member: key_dimensions
  {
    out << "key_dimensions: ";
    to_flow_style_yaml(msg.key_dimensions, out);
    out << ", ";
  }

  // member: standoff_distance
  {
    out << "standoff_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.standoff_distance, out);
    out << ", ";
  }

  // member: contact_force
  {
    out << "contact_force: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TypingGoal & msg,
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

  // member: target_text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_text: ";
    rosidl_generator_traits::value_to_yaml(msg.target_text, out);
    out << "\n";
  }

  // member: keyboard_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyboard_pose:\n";
    to_block_style_yaml(msg.keyboard_pose, out, indentation + 2);
  }

  // member: accuracy_requirement
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accuracy_requirement: ";
    rosidl_generator_traits::value_to_yaml(msg.accuracy_requirement, out);
    out << "\n";
  }

  // member: speed_requirement
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_requirement: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_requirement, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

  // member: keyboard_layout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyboard_layout: ";
    rosidl_generator_traits::value_to_yaml(msg.keyboard_layout, out);
    out << "\n";
  }

  // member: key_spacing_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_spacing_m: ";
    rosidl_generator_traits::value_to_yaml(msg.key_spacing_m, out);
    out << "\n";
  }

  // member: key_dimensions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_dimensions:\n";
    to_block_style_yaml(msg.key_dimensions, out, indentation + 2);
  }

  // member: standoff_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "standoff_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.standoff_distance, out);
    out << "\n";
  }

  // member: contact_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_force: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TypingGoal & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::TypingGoal & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::TypingGoal & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::TypingGoal>()
{
  return "autonomy_interfaces::msg::TypingGoal";
}

template<>
inline const char * name<autonomy_interfaces::msg::TypingGoal>()
{
  return "autonomy_interfaces/msg/TypingGoal";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::TypingGoal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::TypingGoal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::TypingGoal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__TRAITS_HPP_
