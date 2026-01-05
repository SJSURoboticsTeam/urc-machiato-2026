// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/arm_alignment_command.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/arm_alignment_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'target_orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmAlignmentCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: mission_type
  {
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
    out << ", ";
  }

  // member: alignment_id
  {
    out << "alignment_id: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_id, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    to_flow_style_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: target_orientation
  {
    out << "target_orientation: ";
    to_flow_style_yaml(msg.target_orientation, out);
    out << ", ";
  }

  // member: approach_distance
  {
    out << "approach_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_distance, out);
    out << ", ";
  }

  // member: final_distance
  {
    out << "final_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.final_distance, out);
    out << ", ";
  }

  // member: alignment_quality
  {
    out << "alignment_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_quality, out);
    out << ", ";
  }

  // member: max_position_error
  {
    out << "max_position_error: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position_error, out);
    out << ", ";
  }

  // member: max_orientation_error
  {
    out << "max_orientation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.max_orientation_error, out);
    out << ", ";
  }

  // member: alignment_timeout
  {
    out << "alignment_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_timeout, out);
    out << ", ";
  }

  // member: max_approach_speed
  {
    out << "max_approach_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_approach_speed, out);
    out << ", ";
  }

  // member: max_rotation_speed
  {
    out << "max_rotation_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_rotation_speed, out);
    out << ", ";
  }

  // member: enable_collision_avoidance
  {
    out << "enable_collision_avoidance: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_collision_avoidance, out);
    out << ", ";
  }

  // member: safety_zones
  {
    if (msg.safety_zones.size() == 0) {
      out << "safety_zones: []";
    } else {
      out << "safety_zones: [";
      size_t pending_items = msg.safety_zones.size();
      for (auto item : msg.safety_zones) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: require_position_feedback
  {
    out << "require_position_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.require_position_feedback, out);
    out << ", ";
  }

  // member: require_force_feedback
  {
    out << "require_force_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.require_force_feedback, out);
    out << ", ";
  }

  // member: feedback_rate
  {
    out << "feedback_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_rate, out);
    out << ", ";
  }

  // member: required_aruco_tags
  {
    if (msg.required_aruco_tags.size() == 0) {
      out << "required_aruco_tags: []";
    } else {
      out << "required_aruco_tags: [";
      size_t pending_items = msg.required_aruco_tags.size();
      for (auto item : msg.required_aruco_tags) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tag_visibility_timeout
  {
    out << "tag_visibility_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_visibility_timeout, out);
    out << ", ";
  }

  // member: allow_realignment
  {
    out << "allow_realignment: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_realignment, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmAlignmentCommand & msg,
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

  // member: mission_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
    out << "\n";
  }

  // member: alignment_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_id: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_id, out);
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

  // member: target_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_orientation:\n";
    to_block_style_yaml(msg.target_orientation, out, indentation + 2);
  }

  // member: approach_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "approach_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_distance, out);
    out << "\n";
  }

  // member: final_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.final_distance, out);
    out << "\n";
  }

  // member: alignment_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_quality, out);
    out << "\n";
  }

  // member: max_position_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_position_error: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position_error, out);
    out << "\n";
  }

  // member: max_orientation_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_orientation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.max_orientation_error, out);
    out << "\n";
  }

  // member: alignment_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_timeout, out);
    out << "\n";
  }

  // member: max_approach_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_approach_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_approach_speed, out);
    out << "\n";
  }

  // member: max_rotation_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_rotation_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_rotation_speed, out);
    out << "\n";
  }

  // member: enable_collision_avoidance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_collision_avoidance: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_collision_avoidance, out);
    out << "\n";
  }

  // member: safety_zones
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.safety_zones.size() == 0) {
      out << "safety_zones: []\n";
    } else {
      out << "safety_zones:\n";
      for (auto item : msg.safety_zones) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: require_position_feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "require_position_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.require_position_feedback, out);
    out << "\n";
  }

  // member: require_force_feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "require_force_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.require_force_feedback, out);
    out << "\n";
  }

  // member: feedback_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_rate, out);
    out << "\n";
  }

  // member: required_aruco_tags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.required_aruco_tags.size() == 0) {
      out << "required_aruco_tags: []\n";
    } else {
      out << "required_aruco_tags:\n";
      for (auto item : msg.required_aruco_tags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tag_visibility_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag_visibility_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_visibility_timeout, out);
    out << "\n";
  }

  // member: allow_realignment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "allow_realignment: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_realignment, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmAlignmentCommand & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::ArmAlignmentCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::ArmAlignmentCommand & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::ArmAlignmentCommand>()
{
  return "autonomy_interfaces::msg::ArmAlignmentCommand";
}

template<>
inline const char * name<autonomy_interfaces::msg::ArmAlignmentCommand>()
{
  return "autonomy_interfaces/msg/ArmAlignmentCommand";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::ArmAlignmentCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::ArmAlignmentCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::ArmAlignmentCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__TRAITS_HPP_
