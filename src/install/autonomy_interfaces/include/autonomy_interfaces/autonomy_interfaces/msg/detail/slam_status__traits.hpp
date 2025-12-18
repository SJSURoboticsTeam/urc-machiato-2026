// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/SlamStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/slam_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__traits.hpp"
// Member 'local_map'
#include "nav_msgs/msg/detail/occupancy_grid__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SlamStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: local_map
  {
    out << "local_map: ";
    to_flow_style_yaml(msg.local_map, out);
    out << ", ";
  }

  // member: map_width
  {
    out << "map_width: ";
    rosidl_generator_traits::value_to_yaml(msg.map_width, out);
    out << ", ";
  }

  // member: map_height
  {
    out << "map_height: ";
    rosidl_generator_traits::value_to_yaml(msg.map_height, out);
    out << ", ";
  }

  // member: map_resolution
  {
    out << "map_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.map_resolution, out);
    out << ", ";
  }

  // member: loop_closure_confidence
  {
    out << "loop_closure_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_closure_confidence, out);
    out << ", ";
  }

  // member: keyframes_tracked
  {
    out << "keyframes_tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframes_tracked, out);
    out << ", ";
  }

  // member: landmarks_tracked
  {
    out << "landmarks_tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.landmarks_tracked, out);
    out << ", ";
  }

  // member: tracking_quality
  {
    out << "tracking_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_quality, out);
    out << ", ";
  }

  // member: loop_closure_detected
  {
    out << "loop_closure_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_closure_detected, out);
    out << ", ";
  }

  // member: drift_estimate
  {
    out << "drift_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.drift_estimate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SlamStatus & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: local_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_map:\n";
    to_block_style_yaml(msg.local_map, out, indentation + 2);
  }

  // member: map_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_width: ";
    rosidl_generator_traits::value_to_yaml(msg.map_width, out);
    out << "\n";
  }

  // member: map_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_height: ";
    rosidl_generator_traits::value_to_yaml(msg.map_height, out);
    out << "\n";
  }

  // member: map_resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.map_resolution, out);
    out << "\n";
  }

  // member: loop_closure_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_closure_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_closure_confidence, out);
    out << "\n";
  }

  // member: keyframes_tracked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyframes_tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframes_tracked, out);
    out << "\n";
  }

  // member: landmarks_tracked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landmarks_tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.landmarks_tracked, out);
    out << "\n";
  }

  // member: tracking_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_quality, out);
    out << "\n";
  }

  // member: loop_closure_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_closure_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_closure_detected, out);
    out << "\n";
  }

  // member: drift_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drift_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.drift_estimate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SlamStatus & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::SlamStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::SlamStatus & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::SlamStatus>()
{
  return "autonomy_interfaces::msg::SlamStatus";
}

template<>
inline const char * name<autonomy_interfaces::msg::SlamStatus>()
{
  return "autonomy_interfaces/msg/SlamStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::SlamStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::SlamStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::SlamStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__TRAITS_HPP_
