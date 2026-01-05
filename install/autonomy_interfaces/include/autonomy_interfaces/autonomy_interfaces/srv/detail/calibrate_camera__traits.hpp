// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/calibrate_camera.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/calibrate_camera__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCamera_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: image_directory
  {
    out << "image_directory: ";
    rosidl_generator_traits::value_to_yaml(msg.image_directory, out);
    out << ", ";
  }

  // member: board_type
  {
    out << "board_type: ";
    rosidl_generator_traits::value_to_yaml(msg.board_type, out);
    out << ", ";
  }

  // member: squares_x
  {
    out << "squares_x: ";
    rosidl_generator_traits::value_to_yaml(msg.squares_x, out);
    out << ", ";
  }

  // member: squares_y
  {
    out << "squares_y: ";
    rosidl_generator_traits::value_to_yaml(msg.squares_y, out);
    out << ", ";
  }

  // member: square_size
  {
    out << "square_size: ";
    rosidl_generator_traits::value_to_yaml(msg.square_size, out);
    out << ", ";
  }

  // member: marker_size
  {
    out << "marker_size: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCamera_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image_directory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_directory: ";
    rosidl_generator_traits::value_to_yaml(msg.image_directory, out);
    out << "\n";
  }

  // member: board_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_type: ";
    rosidl_generator_traits::value_to_yaml(msg.board_type, out);
    out << "\n";
  }

  // member: squares_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "squares_x: ";
    rosidl_generator_traits::value_to_yaml(msg.squares_x, out);
    out << "\n";
  }

  // member: squares_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "squares_y: ";
    rosidl_generator_traits::value_to_yaml(msg.squares_y, out);
    out << "\n";
  }

  // member: square_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "square_size: ";
    rosidl_generator_traits::value_to_yaml(msg.square_size, out);
    out << "\n";
  }

  // member: marker_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "marker_size: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_size, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCamera_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::CalibrateCamera_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::CalibrateCamera_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::CalibrateCamera_Request>()
{
  return "autonomy_interfaces::srv::CalibrateCamera_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::CalibrateCamera_Request>()
{
  return "autonomy_interfaces/srv/CalibrateCamera_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::CalibrateCamera_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::CalibrateCamera_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCamera_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result_file
  {
    out << "result_file: ";
    rosidl_generator_traits::value_to_yaml(msg.result_file, out);
    out << ", ";
  }

  // member: calibration_summary
  {
    out << "calibration_summary: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_summary, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCamera_Response & msg,
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

  // member: result_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_file: ";
    rosidl_generator_traits::value_to_yaml(msg.result_file, out);
    out << "\n";
  }

  // member: calibration_summary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_summary: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_summary, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCamera_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::CalibrateCamera_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::CalibrateCamera_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::CalibrateCamera_Response>()
{
  return "autonomy_interfaces::srv::CalibrateCamera_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::CalibrateCamera_Response>()
{
  return "autonomy_interfaces/srv/CalibrateCamera_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::CalibrateCamera_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::CalibrateCamera_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCamera_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const CalibrateCamera_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCamera_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::CalibrateCamera_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::CalibrateCamera_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::CalibrateCamera_Event>()
{
  return "autonomy_interfaces::srv::CalibrateCamera_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::CalibrateCamera_Event>()
{
  return "autonomy_interfaces/srv/CalibrateCamera_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::CalibrateCamera_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Request>::value && has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::CalibrateCamera_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::CalibrateCamera>()
{
  return "autonomy_interfaces::srv::CalibrateCamera";
}

template<>
inline const char * name<autonomy_interfaces::srv::CalibrateCamera>()
{
  return "autonomy_interfaces/srv/CalibrateCamera";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::CalibrateCamera>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::CalibrateCamera_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::CalibrateCamera_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::CalibrateCamera>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::CalibrateCamera_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::CalibrateCamera>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::CalibrateCamera_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::CalibrateCamera_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_
