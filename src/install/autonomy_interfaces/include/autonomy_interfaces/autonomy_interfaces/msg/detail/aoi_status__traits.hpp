// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/aoi_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AOIStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: sensor_name
  {
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << ", ";
  }

  // member: sensor_type
  {
    out << "sensor_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_type, out);
    out << ", ";
  }

  // member: current_aoi
  {
    out << "current_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.current_aoi, out);
    out << ", ";
  }

  // member: average_aoi
  {
    out << "average_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.average_aoi, out);
    out << ", ";
  }

  // member: max_aoi
  {
    out << "max_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.max_aoi, out);
    out << ", ";
  }

  // member: min_aoi
  {
    out << "min_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.min_aoi, out);
    out << ", ";
  }

  // member: is_fresh
  {
    out << "is_fresh: ";
    rosidl_generator_traits::value_to_yaml(msg.is_fresh, out);
    out << ", ";
  }

  // member: quality_score
  {
    out << "quality_score: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_score, out);
    out << ", ";
  }

  // member: freshness_status
  {
    out << "freshness_status: ";
    rosidl_generator_traits::value_to_yaml(msg.freshness_status, out);
    out << ", ";
  }

  // member: acceptable_threshold
  {
    out << "acceptable_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.acceptable_threshold, out);
    out << ", ";
  }

  // member: optimal_threshold
  {
    out << "optimal_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.optimal_threshold, out);
    out << ", ";
  }

  // member: sample_count
  {
    out << "sample_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_count, out);
    out << ", ";
  }

  // member: freshness_ratio
  {
    out << "freshness_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.freshness_ratio, out);
    out << ", ";
  }

  // member: transport_type
  {
    out << "transport_type: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_type, out);
    out << ", ";
  }

  // member: network_latency
  {
    out << "network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.network_latency, out);
    out << ", ";
  }

  // member: transport_latency
  {
    out << "transport_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_latency, out);
    out << ", ";
  }

  // member: congestion_detected
  {
    out << "congestion_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.congestion_detected, out);
    out << ", ";
  }

  // member: congestion_factor
  {
    out << "congestion_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.congestion_factor, out);
    out << ", ";
  }

  // member: predicted_aoi
  {
    out << "predicted_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.predicted_aoi, out);
    out << ", ";
  }

  // member: aoi_trend
  {
    out << "aoi_trend: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_trend, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AOIStatus & msg,
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

  // member: sensor_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << "\n";
  }

  // member: sensor_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_type, out);
    out << "\n";
  }

  // member: current_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.current_aoi, out);
    out << "\n";
  }

  // member: average_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.average_aoi, out);
    out << "\n";
  }

  // member: max_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.max_aoi, out);
    out << "\n";
  }

  // member: min_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.min_aoi, out);
    out << "\n";
  }

  // member: is_fresh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_fresh: ";
    rosidl_generator_traits::value_to_yaml(msg.is_fresh, out);
    out << "\n";
  }

  // member: quality_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality_score: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_score, out);
    out << "\n";
  }

  // member: freshness_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freshness_status: ";
    rosidl_generator_traits::value_to_yaml(msg.freshness_status, out);
    out << "\n";
  }

  // member: acceptable_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceptable_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.acceptable_threshold, out);
    out << "\n";
  }

  // member: optimal_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "optimal_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.optimal_threshold, out);
    out << "\n";
  }

  // member: sample_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_count, out);
    out << "\n";
  }

  // member: freshness_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freshness_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.freshness_ratio, out);
    out << "\n";
  }

  // member: transport_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transport_type: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_type, out);
    out << "\n";
  }

  // member: network_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.network_latency, out);
    out << "\n";
  }

  // member: transport_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transport_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_latency, out);
    out << "\n";
  }

  // member: congestion_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "congestion_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.congestion_detected, out);
    out << "\n";
  }

  // member: congestion_factor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "congestion_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.congestion_factor, out);
    out << "\n";
  }

  // member: predicted_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "predicted_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.predicted_aoi, out);
    out << "\n";
  }

  // member: aoi_trend
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoi_trend: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_trend, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AOIStatus & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::AOIStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::AOIStatus & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::AOIStatus>()
{
  return "autonomy_interfaces::msg::AOIStatus";
}

template<>
inline const char * name<autonomy_interfaces::msg::AOIStatus>()
{
  return "autonomy_interfaces/msg/AOIStatus";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::AOIStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::AOIStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::AOIStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__TRAITS_HPP_
