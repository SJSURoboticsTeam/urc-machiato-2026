// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/LoadCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/load_calibration_parameters__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadCalibrationParameters_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: calibration_file
  {
    out << "calibration_file: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_file, out);
    out << ", ";
  }

  // member: parameter_namespace
  {
    out << "parameter_namespace: ";
    rosidl_generator_traits::value_to_yaml(msg.parameter_namespace, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadCalibrationParameters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: calibration_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_file: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_file, out);
    out << "\n";
  }

  // member: parameter_namespace
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parameter_namespace: ";
    rosidl_generator_traits::value_to_yaml(msg.parameter_namespace, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadCalibrationParameters_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::LoadCalibrationParameters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::LoadCalibrationParameters_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::LoadCalibrationParameters_Request>()
{
  return "autonomy_interfaces::srv::LoadCalibrationParameters_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::LoadCalibrationParameters_Request>()
{
  return "autonomy_interfaces/srv/LoadCalibrationParameters_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::LoadCalibrationParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::LoadCalibrationParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::LoadCalibrationParameters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadCalibrationParameters_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
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
  const LoadCalibrationParameters_Response & msg,
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

inline std::string to_yaml(const LoadCalibrationParameters_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::LoadCalibrationParameters_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::LoadCalibrationParameters_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::LoadCalibrationParameters_Response>()
{
  return "autonomy_interfaces::srv::LoadCalibrationParameters_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::LoadCalibrationParameters_Response>()
{
  return "autonomy_interfaces/srv/LoadCalibrationParameters_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::LoadCalibrationParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::LoadCalibrationParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::LoadCalibrationParameters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::LoadCalibrationParameters>()
{
  return "autonomy_interfaces::srv::LoadCalibrationParameters";
}

template<>
inline const char * name<autonomy_interfaces::srv::LoadCalibrationParameters>()
{
  return "autonomy_interfaces/srv/LoadCalibrationParameters";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::LoadCalibrationParameters>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::LoadCalibrationParameters_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::LoadCalibrationParameters_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::LoadCalibrationParameters>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::LoadCalibrationParameters_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::LoadCalibrationParameters_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::LoadCalibrationParameters>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::LoadCalibrationParameters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::LoadCalibrationParameters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_PARAMETERS__TRAITS_HPP_
