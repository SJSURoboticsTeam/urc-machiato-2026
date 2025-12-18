// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/camera_command__struct.hpp"
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
  const CameraCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: command_type
  {
    out << "command_type: ";
    rosidl_generator_traits::value_to_yaml(msg.command_type, out);
    out << ", ";
  }

  // member: pan_angle
  {
    out << "pan_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_angle, out);
    out << ", ";
  }

  // member: tilt_angle
  {
    out << "tilt_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_angle, out);
    out << ", ";
  }

  // member: pan_speed
  {
    out << "pan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_speed, out);
    out << ", ";
  }

  // member: tilt_speed
  {
    out << "tilt_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_speed, out);
    out << ", ";
  }

  // member: zoom_level
  {
    out << "zoom_level: ";
    rosidl_generator_traits::value_to_yaml(msg.zoom_level, out);
    out << ", ";
  }

  // member: autofocus
  {
    out << "autofocus: ";
    rosidl_generator_traits::value_to_yaml(msg.autofocus, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    to_flow_style_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: tracking_timeout
  {
    out << "tracking_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_timeout, out);
    out << ", ";
  }

  // member: scan_pattern
  {
    out << "scan_pattern: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_pattern, out);
    out << ", ";
  }

  // member: scan_speed
  {
    out << "scan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_speed, out);
    out << ", ";
  }

  // member: scan_range
  {
    out << "scan_range: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_range, out);
    out << ", ";
  }

  // member: max_pan_speed
  {
    out << "max_pan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pan_speed, out);
    out << ", ";
  }

  // member: max_tilt_speed
  {
    out << "max_tilt_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_tilt_speed, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraCommand & msg,
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

  // member: command_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_type: ";
    rosidl_generator_traits::value_to_yaml(msg.command_type, out);
    out << "\n";
  }

  // member: pan_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_angle, out);
    out << "\n";
  }

  // member: tilt_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_angle, out);
    out << "\n";
  }

  // member: pan_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_speed, out);
    out << "\n";
  }

  // member: tilt_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_speed, out);
    out << "\n";
  }

  // member: zoom_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zoom_level: ";
    rosidl_generator_traits::value_to_yaml(msg.zoom_level, out);
    out << "\n";
  }

  // member: autofocus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autofocus: ";
    rosidl_generator_traits::value_to_yaml(msg.autofocus, out);
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

  // member: tracking_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_timeout, out);
    out << "\n";
  }

  // member: scan_pattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_pattern: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_pattern, out);
    out << "\n";
  }

  // member: scan_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_speed, out);
    out << "\n";
  }

  // member: scan_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_range: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_range, out);
    out << "\n";
  }

  // member: max_pan_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_pan_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pan_speed, out);
    out << "\n";
  }

  // member: max_tilt_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_tilt_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_tilt_speed, out);
    out << "\n";
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraCommand & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::CameraCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::CameraCommand & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::CameraCommand>()
{
  return "autonomy_interfaces::msg::CameraCommand";
}

template<>
inline const char * name<autonomy_interfaces::msg::CameraCommand>()
{
  return "autonomy_interfaces/msg/CameraCommand";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::CameraCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::CameraCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::CameraCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__TRAITS_HPP_
