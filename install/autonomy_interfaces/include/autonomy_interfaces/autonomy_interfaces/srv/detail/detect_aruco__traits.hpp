// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/detect_aruco__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DetectAruco_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_tag_ids
  {
    if (msg.target_tag_ids.size() == 0) {
      out << "target_tag_ids: []";
    } else {
      out << "target_tag_ids: [";
      size_t pending_items = msg.target_tag_ids.size();
      for (auto item : msg.target_tag_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: detection_timeout
  {
    out << "detection_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_timeout, out);
    out << ", ";
  }

  // member: require_distance_estimate
  {
    out << "require_distance_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.require_distance_estimate, out);
    out << ", ";
  }

  // member: max_detection_distance
  {
    out << "max_detection_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.max_detection_distance, out);
    out << ", ";
  }

  // member: calculate_alignment
  {
    out << "calculate_alignment: ";
    rosidl_generator_traits::value_to_yaml(msg.calculate_alignment, out);
    out << ", ";
  }

  // member: target_depth
  {
    out << "target_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.target_depth, out);
    out << ", ";
  }

  // member: mission_type
  {
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectAruco_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_tag_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_tag_ids.size() == 0) {
      out << "target_tag_ids: []\n";
    } else {
      out << "target_tag_ids:\n";
      for (auto item : msg.target_tag_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: detection_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_timeout, out);
    out << "\n";
  }

  // member: require_distance_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "require_distance_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.require_distance_estimate, out);
    out << "\n";
  }

  // member: max_detection_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_detection_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.max_detection_distance, out);
    out << "\n";
  }

  // member: calculate_alignment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calculate_alignment: ";
    rosidl_generator_traits::value_to_yaml(msg.calculate_alignment, out);
    out << "\n";
  }

  // member: target_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.target_depth, out);
    out << "\n";
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectAruco_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::DetectAruco_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::DetectAruco_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::DetectAruco_Request>()
{
  return "autonomy_interfaces::srv::DetectAruco_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::DetectAruco_Request>()
{
  return "autonomy_interfaces/srv/DetectAruco_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::DetectAruco_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::DetectAruco_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::DetectAruco_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'tag_positions'
// Member 'alignment_center'
// Member 'arm_target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'detection_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'alignment_orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DetectAruco_Response & msg,
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

  // member: detected_tag_ids
  {
    if (msg.detected_tag_ids.size() == 0) {
      out << "detected_tag_ids: []";
    } else {
      out << "detected_tag_ids: [";
      size_t pending_items = msg.detected_tag_ids.size();
      for (auto item : msg.detected_tag_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tag_positions
  {
    if (msg.tag_positions.size() == 0) {
      out << "tag_positions: []";
    } else {
      out << "tag_positions: [";
      size_t pending_items = msg.tag_positions.size();
      for (auto item : msg.tag_positions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tag_distances
  {
    if (msg.tag_distances.size() == 0) {
      out << "tag_distances: []";
    } else {
      out << "tag_distances: [";
      size_t pending_items = msg.tag_distances.size();
      for (auto item : msg.tag_distances) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tag_angles
  {
    if (msg.tag_angles.size() == 0) {
      out << "tag_angles: []";
    } else {
      out << "tag_angles: [";
      size_t pending_items = msg.tag_angles.size();
      for (auto item : msg.tag_angles) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: detection_time
  {
    out << "detection_time: ";
    to_flow_style_yaml(msg.detection_time, out);
    out << ", ";
  }

  // member: alignment_available
  {
    out << "alignment_available: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_available, out);
    out << ", ";
  }

  // member: alignment_center
  {
    out << "alignment_center: ";
    to_flow_style_yaml(msg.alignment_center, out);
    out << ", ";
  }

  // member: alignment_orientation
  {
    out << "alignment_orientation: ";
    to_flow_style_yaml(msg.alignment_orientation, out);
    out << ", ";
  }

  // member: arm_target_position
  {
    out << "arm_target_position: ";
    to_flow_style_yaml(msg.arm_target_position, out);
    out << ", ";
  }

  // member: alignment_quality
  {
    out << "alignment_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_quality, out);
    out << ", ";
  }

  // member: alignment_warnings
  {
    if (msg.alignment_warnings.size() == 0) {
      out << "alignment_warnings: []";
    } else {
      out << "alignment_warnings: [";
      size_t pending_items = msg.alignment_warnings.size();
      for (auto item : msg.alignment_warnings) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const DetectAruco_Response & msg,
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

  // member: detected_tag_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detected_tag_ids.size() == 0) {
      out << "detected_tag_ids: []\n";
    } else {
      out << "detected_tag_ids:\n";
      for (auto item : msg.detected_tag_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tag_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tag_positions.size() == 0) {
      out << "tag_positions: []\n";
    } else {
      out << "tag_positions:\n";
      for (auto item : msg.tag_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: tag_distances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tag_distances.size() == 0) {
      out << "tag_distances: []\n";
    } else {
      out << "tag_distances:\n";
      for (auto item : msg.tag_distances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tag_angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tag_angles.size() == 0) {
      out << "tag_angles: []\n";
    } else {
      out << "tag_angles:\n";
      for (auto item : msg.tag_angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: detection_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_time:\n";
    to_block_style_yaml(msg.detection_time, out, indentation + 2);
  }

  // member: alignment_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_available: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_available, out);
    out << "\n";
  }

  // member: alignment_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_center:\n";
    to_block_style_yaml(msg.alignment_center, out, indentation + 2);
  }

  // member: alignment_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_orientation:\n";
    to_block_style_yaml(msg.alignment_orientation, out, indentation + 2);
  }

  // member: arm_target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_target_position:\n";
    to_block_style_yaml(msg.arm_target_position, out, indentation + 2);
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

  // member: alignment_warnings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.alignment_warnings.size() == 0) {
      out << "alignment_warnings: []\n";
    } else {
      out << "alignment_warnings:\n";
      for (auto item : msg.alignment_warnings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectAruco_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::DetectAruco_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::DetectAruco_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::DetectAruco_Response>()
{
  return "autonomy_interfaces::srv::DetectAruco_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::DetectAruco_Response>()
{
  return "autonomy_interfaces/srv/DetectAruco_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::DetectAruco_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::DetectAruco_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::DetectAruco_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::DetectAruco>()
{
  return "autonomy_interfaces::srv::DetectAruco";
}

template<>
inline const char * name<autonomy_interfaces::srv::DetectAruco>()
{
  return "autonomy_interfaces/srv/DetectAruco";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::DetectAruco>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::DetectAruco_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::DetectAruco_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::DetectAruco>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::DetectAruco_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::DetectAruco_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::DetectAruco>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::DetectAruco_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::DetectAruco_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__TRAITS_HPP_
