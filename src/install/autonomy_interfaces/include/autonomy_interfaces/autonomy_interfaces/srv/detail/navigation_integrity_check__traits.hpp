// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/navigation_integrity_check__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const NavigationIntegrityCheck_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: detailed_check
  {
    out << "detailed_check: ";
    rosidl_generator_traits::value_to_yaml(msg.detailed_check, out);
    out << ", ";
  }

  // member: check_components
  {
    if (msg.check_components.size() == 0) {
      out << "check_components: []";
    } else {
      out << "check_components: [";
      size_t pending_items = msg.check_components.size();
      for (auto item : msg.check_components) {
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
  const NavigationIntegrityCheck_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: detailed_check
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detailed_check: ";
    rosidl_generator_traits::value_to_yaml(msg.detailed_check, out);
    out << "\n";
  }

  // member: check_components
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.check_components.size() == 0) {
      out << "check_components: []\n";
    } else {
      out << "check_components:\n";
      for (auto item : msg.check_components) {
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

inline std::string to_yaml(const NavigationIntegrityCheck_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::NavigationIntegrityCheck_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::NavigationIntegrityCheck_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>()
{
  return "autonomy_interfaces::srv::NavigationIntegrityCheck_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>()
{
  return "autonomy_interfaces/srv/NavigationIntegrityCheck_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const NavigationIntegrityCheck_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: integrity_ok
  {
    out << "integrity_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_ok, out);
    out << ", ";
  }

  // member: integrity_score
  {
    out << "integrity_score: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_score, out);
    out << ", ";
  }

  // member: integrity_level
  {
    out << "integrity_level: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_level, out);
    out << ", ";
  }

  // member: checked_components
  {
    if (msg.checked_components.size() == 0) {
      out << "checked_components: []";
    } else {
      out << "checked_components: [";
      size_t pending_items = msg.checked_components.size();
      for (auto item : msg.checked_components) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: component_status
  {
    if (msg.component_status.size() == 0) {
      out << "component_status: []";
    } else {
      out << "component_status: [";
      size_t pending_items = msg.component_status.size();
      for (auto item : msg.component_status) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: component_details
  {
    if (msg.component_details.size() == 0) {
      out << "component_details: []";
    } else {
      out << "component_details: [";
      size_t pending_items = msg.component_details.size();
      for (auto item : msg.component_details) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position_accuracy
  {
    out << "position_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.position_accuracy, out);
    out << ", ";
  }

  // member: heading_accuracy
  {
    out << "heading_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_accuracy, out);
    out << ", ";
  }

  // member: velocity_consistency
  {
    out << "velocity_consistency: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_consistency, out);
    out << ", ";
  }

  // member: satellite_count
  {
    out << "satellite_count: ";
    rosidl_generator_traits::value_to_yaml(msg.satellite_count, out);
    out << ", ";
  }

  // member: hdop
  {
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << ", ";
  }

  // member: recommendations
  {
    if (msg.recommendations.size() == 0) {
      out << "recommendations: []";
    } else {
      out << "recommendations: [";
      size_t pending_items = msg.recommendations.size();
      for (auto item : msg.recommendations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigationIntegrityCheck_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: integrity_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrity_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_ok, out);
    out << "\n";
  }

  // member: integrity_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrity_score: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_score, out);
    out << "\n";
  }

  // member: integrity_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrity_level: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_level, out);
    out << "\n";
  }

  // member: checked_components
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.checked_components.size() == 0) {
      out << "checked_components: []\n";
    } else {
      out << "checked_components:\n";
      for (auto item : msg.checked_components) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: component_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.component_status.size() == 0) {
      out << "component_status: []\n";
    } else {
      out << "component_status:\n";
      for (auto item : msg.component_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: component_details
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.component_details.size() == 0) {
      out << "component_details: []\n";
    } else {
      out << "component_details:\n";
      for (auto item : msg.component_details) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.position_accuracy, out);
    out << "\n";
  }

  // member: heading_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_accuracy, out);
    out << "\n";
  }

  // member: velocity_consistency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_consistency: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_consistency, out);
    out << "\n";
  }

  // member: satellite_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "satellite_count: ";
    rosidl_generator_traits::value_to_yaml(msg.satellite_count, out);
    out << "\n";
  }

  // member: hdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << "\n";
  }

  // member: recommendations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.recommendations.size() == 0) {
      out << "recommendations: []\n";
    } else {
      out << "recommendations:\n";
      for (auto item : msg.recommendations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigationIntegrityCheck_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::NavigationIntegrityCheck_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>()
{
  return "autonomy_interfaces::srv::NavigationIntegrityCheck_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>()
{
  return "autonomy_interfaces/srv/NavigationIntegrityCheck_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::NavigationIntegrityCheck>()
{
  return "autonomy_interfaces::srv::NavigationIntegrityCheck";
}

template<>
inline const char * name<autonomy_interfaces::srv::NavigationIntegrityCheck>()
{
  return "autonomy_interfaces/srv/NavigationIntegrityCheck";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::NavigationIntegrityCheck>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::NavigationIntegrityCheck>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::NavigationIntegrityCheck>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::NavigationIntegrityCheck_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::NavigationIntegrityCheck_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__TRAITS_HPP_
