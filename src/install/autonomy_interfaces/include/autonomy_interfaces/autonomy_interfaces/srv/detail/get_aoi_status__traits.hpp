// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/get_aoi_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAOIStatus_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: sensor_name
  {
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << ", ";
  }

  // member: include_history
  {
    out << "include_history: ";
    rosidl_generator_traits::value_to_yaml(msg.include_history, out);
    out << ", ";
  }

  // member: history_samples
  {
    out << "history_samples: ";
    rosidl_generator_traits::value_to_yaml(msg.history_samples, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAOIStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sensor_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << "\n";
  }

  // member: include_history
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_history: ";
    rosidl_generator_traits::value_to_yaml(msg.include_history, out);
    out << "\n";
  }

  // member: history_samples
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "history_samples: ";
    rosidl_generator_traits::value_to_yaml(msg.history_samples, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAOIStatus_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetAOIStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetAOIStatus_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAOIStatus_Request>()
{
  return "autonomy_interfaces::srv::GetAOIStatus_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAOIStatus_Request>()
{
  return "autonomy_interfaces/srv/GetAOIStatus_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAOIStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAOIStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetAOIStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'sensor_status'
#include "autonomy_interfaces/msg/detail/aoi_status__traits.hpp"
// Member 'timestamp_history'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'system_metrics'
#include "autonomy_interfaces/msg/detail/aoi_metrics__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAOIStatus_Response & msg,
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

  // member: sensor_status
  {
    if (msg.sensor_status.size() == 0) {
      out << "sensor_status: []";
    } else {
      out << "sensor_status: [";
      size_t pending_items = msg.sensor_status.size();
      for (auto item : msg.sensor_status) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: aoi_history
  {
    if (msg.aoi_history.size() == 0) {
      out << "aoi_history: []";
    } else {
      out << "aoi_history: [";
      size_t pending_items = msg.aoi_history.size();
      for (auto item : msg.aoi_history) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timestamp_history
  {
    if (msg.timestamp_history.size() == 0) {
      out << "timestamp_history: []";
    } else {
      out << "timestamp_history: [";
      size_t pending_items = msg.timestamp_history.size();
      for (auto item : msg.timestamp_history) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: system_metrics
  {
    out << "system_metrics: ";
    to_flow_style_yaml(msg.system_metrics, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAOIStatus_Response & msg,
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

  // member: sensor_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sensor_status.size() == 0) {
      out << "sensor_status: []\n";
    } else {
      out << "sensor_status:\n";
      for (auto item : msg.sensor_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: aoi_history
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.aoi_history.size() == 0) {
      out << "aoi_history: []\n";
    } else {
      out << "aoi_history:\n";
      for (auto item : msg.aoi_history) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timestamp_history
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.timestamp_history.size() == 0) {
      out << "timestamp_history: []\n";
    } else {
      out << "timestamp_history:\n";
      for (auto item : msg.timestamp_history) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: system_metrics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_metrics:\n";
    to_block_style_yaml(msg.system_metrics, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAOIStatus_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::GetAOIStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::GetAOIStatus_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAOIStatus_Response>()
{
  return "autonomy_interfaces::srv::GetAOIStatus_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAOIStatus_Response>()
{
  return "autonomy_interfaces/srv/GetAOIStatus_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAOIStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAOIStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::GetAOIStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::GetAOIStatus>()
{
  return "autonomy_interfaces::srv::GetAOIStatus";
}

template<>
inline const char * name<autonomy_interfaces::srv::GetAOIStatus>()
{
  return "autonomy_interfaces/srv/GetAOIStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::GetAOIStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::GetAOIStatus_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::GetAOIStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::GetAOIStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::GetAOIStatus_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::GetAOIStatus_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::GetAOIStatus>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::GetAOIStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::GetAOIStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__TRAITS_HPP_
