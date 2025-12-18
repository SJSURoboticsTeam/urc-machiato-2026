// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/configure_mission__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'waypoints'
// Member 'typing_location'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConfigureMission_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mission_name
  {
    out << "mission_name: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_name, out);
    out << ", ";
  }

  // member: objectives
  {
    if (msg.objectives.size() == 0) {
      out << "objectives: []";
    } else {
      out << "objectives: [";
      size_t pending_items = msg.objectives.size();
      for (auto item : msg.objectives) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: waypoint_names
  {
    if (msg.waypoint_names.size() == 0) {
      out << "waypoint_names: []";
    } else {
      out << "waypoint_names: [";
      size_t pending_items = msg.waypoint_names.size();
      for (auto item : msg.waypoint_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: precision_required
  {
    if (msg.precision_required.size() == 0) {
      out << "precision_required: []";
    } else {
      out << "precision_required: [";
      size_t pending_items = msg.precision_required.size();
      for (auto item : msg.precision_required) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: waypoint_tolerances
  {
    if (msg.waypoint_tolerances.size() == 0) {
      out << "waypoint_tolerances: []";
    } else {
      out << "waypoint_tolerances: [";
      size_t pending_items = msg.waypoint_tolerances.size();
      for (auto item : msg.waypoint_tolerances) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: time_limit
  {
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
    out << ", ";
  }

  // member: waypoint_timeout
  {
    out << "waypoint_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_timeout, out);
    out << ", ";
  }

  // member: max_linear_velocity
  {
    out << "max_linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_linear_velocity, out);
    out << ", ";
  }

  // member: max_angular_velocity
  {
    out << "max_angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_angular_velocity, out);
    out << ", ";
  }

  // member: waypoint_approach_tolerance
  {
    out << "waypoint_approach_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_approach_tolerance, out);
    out << ", ";
  }

  // member: typing_text
  {
    out << "typing_text: ";
    rosidl_generator_traits::value_to_yaml(msg.typing_text, out);
    out << ", ";
  }

  // member: typing_location
  {
    out << "typing_location: ";
    to_flow_style_yaml(msg.typing_location, out);
    out << ", ";
  }

  // member: terrain_type
  {
    out << "terrain_type: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_type, out);
    out << ", ";
  }

  // member: max_incline
  {
    out << "max_incline: ";
    rosidl_generator_traits::value_to_yaml(msg.max_incline, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConfigureMission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mission_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_name: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_name, out);
    out << "\n";
  }

  // member: objectives
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objectives.size() == 0) {
      out << "objectives: []\n";
    } else {
      out << "objectives:\n";
      for (auto item : msg.objectives) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: waypoint_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoint_names.size() == 0) {
      out << "waypoint_names: []\n";
    } else {
      out << "waypoint_names:\n";
      for (auto item : msg.waypoint_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: precision_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.precision_required.size() == 0) {
      out << "precision_required: []\n";
    } else {
      out << "precision_required:\n";
      for (auto item : msg.precision_required) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: waypoint_tolerances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoint_tolerances.size() == 0) {
      out << "waypoint_tolerances: []\n";
    } else {
      out << "waypoint_tolerances:\n";
      for (auto item : msg.waypoint_tolerances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: time_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
    out << "\n";
  }

  // member: waypoint_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoint_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_timeout, out);
    out << "\n";
  }

  // member: max_linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_linear_velocity, out);
    out << "\n";
  }

  // member: max_angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_angular_velocity, out);
    out << "\n";
  }

  // member: waypoint_approach_tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoint_approach_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoint_approach_tolerance, out);
    out << "\n";
  }

  // member: typing_text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "typing_text: ";
    rosidl_generator_traits::value_to_yaml(msg.typing_text, out);
    out << "\n";
  }

  // member: typing_location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "typing_location:\n";
    to_block_style_yaml(msg.typing_location, out, indentation + 2);
  }

  // member: terrain_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "terrain_type: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_type, out);
    out << "\n";
  }

  // member: max_incline
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_incline: ";
    rosidl_generator_traits::value_to_yaml(msg.max_incline, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConfigureMission_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ConfigureMission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ConfigureMission_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ConfigureMission_Request>()
{
  return "autonomy_interfaces::srv::ConfigureMission_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::ConfigureMission_Request>()
{
  return "autonomy_interfaces/srv/ConfigureMission_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ConfigureMission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ConfigureMission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ConfigureMission_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConfigureMission_Response & msg,
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

  // member: mission_id
  {
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << ", ";
  }

  // member: estimated_duration
  {
    out << "estimated_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_duration, out);
    out << ", ";
  }

  // member: total_waypoints
  {
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << ", ";
  }

  // member: configured_objectives
  {
    if (msg.configured_objectives.size() == 0) {
      out << "configured_objectives: []";
    } else {
      out << "configured_objectives: [";
      size_t pending_items = msg.configured_objectives.size();
      for (auto item : msg.configured_objectives) {
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
  const ConfigureMission_Response & msg,
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

  // member: mission_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << "\n";
  }

  // member: estimated_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_duration, out);
    out << "\n";
  }

  // member: total_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << "\n";
  }

  // member: configured_objectives
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.configured_objectives.size() == 0) {
      out << "configured_objectives: []\n";
    } else {
      out << "configured_objectives:\n";
      for (auto item : msg.configured_objectives) {
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

inline std::string to_yaml(const ConfigureMission_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ConfigureMission_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ConfigureMission_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ConfigureMission_Response>()
{
  return "autonomy_interfaces::srv::ConfigureMission_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::ConfigureMission_Response>()
{
  return "autonomy_interfaces/srv/ConfigureMission_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ConfigureMission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ConfigureMission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ConfigureMission_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::ConfigureMission>()
{
  return "autonomy_interfaces::srv::ConfigureMission";
}

template<>
inline const char * name<autonomy_interfaces::srv::ConfigureMission>()
{
  return "autonomy_interfaces/srv/ConfigureMission";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ConfigureMission>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::ConfigureMission_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::ConfigureMission_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ConfigureMission>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::ConfigureMission_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::ConfigureMission_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::ConfigureMission>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::ConfigureMission_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::ConfigureMission_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__TRAITS_HPP_
