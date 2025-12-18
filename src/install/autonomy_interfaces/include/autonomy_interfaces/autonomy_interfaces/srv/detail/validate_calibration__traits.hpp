// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/validate_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidateCalibration_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: calibration_file
  {
    out << "calibration_file: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_file, out);
    out << ", ";
  }

  // member: test_images
  {
    if (msg.test_images.size() == 0) {
      out << "test_images: []";
    } else {
      out << "test_images: [";
      size_t pending_items = msg.test_images.size();
      for (auto item : msg.test_images) {
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
  const ValidateCalibration_Request & msg,
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

  // member: test_images
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.test_images.size() == 0) {
      out << "test_images: []\n";
    } else {
      out << "test_images:\n";
      for (auto item : msg.test_images) {
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

inline std::string to_yaml(const ValidateCalibration_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ValidateCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ValidateCalibration_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ValidateCalibration_Request>()
{
  return "autonomy_interfaces::srv::ValidateCalibration_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::ValidateCalibration_Request>()
{
  return "autonomy_interfaces/srv/ValidateCalibration_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ValidateCalibration_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ValidateCalibration_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ValidateCalibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidateCalibration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: reprojection_error
  {
    out << "reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.reprojection_error, out);
    out << ", ";
  }

  // member: quality_assessment
  {
    out << "quality_assessment: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_assessment, out);
    out << ", ";
  }

  // member: recommendations
  {
    out << "recommendations: ";
    rosidl_generator_traits::value_to_yaml(msg.recommendations, out);
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
  const ValidateCalibration_Response & msg,
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

  // member: reprojection_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.reprojection_error, out);
    out << "\n";
  }

  // member: quality_assessment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality_assessment: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_assessment, out);
    out << "\n";
  }

  // member: recommendations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recommendations: ";
    rosidl_generator_traits::value_to_yaml(msg.recommendations, out);
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

inline std::string to_yaml(const ValidateCalibration_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ValidateCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ValidateCalibration_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ValidateCalibration_Response>()
{
  return "autonomy_interfaces::srv::ValidateCalibration_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::ValidateCalibration_Response>()
{
  return "autonomy_interfaces/srv/ValidateCalibration_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ValidateCalibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ValidateCalibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ValidateCalibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::ValidateCalibration>()
{
  return "autonomy_interfaces::srv::ValidateCalibration";
}

template<>
inline const char * name<autonomy_interfaces::srv::ValidateCalibration>()
{
  return "autonomy_interfaces/srv/ValidateCalibration";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ValidateCalibration>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::ValidateCalibration_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::ValidateCalibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ValidateCalibration>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::ValidateCalibration_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::ValidateCalibration_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::ValidateCalibration>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::ValidateCalibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::ValidateCalibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__TRAITS_HPP_
