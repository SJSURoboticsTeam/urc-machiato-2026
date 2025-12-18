// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__TRAITS_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/msg/detail/aoi_metrics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'last_alert_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AOIMetrics & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: system_average_aoi
  {
    out << "system_average_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.system_average_aoi, out);
    out << ", ";
  }

  // member: total_sensors
  {
    out << "total_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.total_sensors, out);
    out << ", ";
  }

  // member: fresh_sensors
  {
    out << "fresh_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.fresh_sensors, out);
    out << ", ";
  }

  // member: stale_sensors
  {
    out << "stale_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.stale_sensors, out);
    out << ", ";
  }

  // member: critical_sensors
  {
    out << "critical_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.critical_sensors, out);
    out << ", ";
  }

  // member: aoi_p50
  {
    out << "aoi_p50: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p50, out);
    out << ", ";
  }

  // member: aoi_p90
  {
    out << "aoi_p90: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p90, out);
    out << ", ";
  }

  // member: aoi_p95
  {
    out << "aoi_p95: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p95, out);
    out << ", ";
  }

  // member: aoi_p99
  {
    out << "aoi_p99: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p99, out);
    out << ", ";
  }

  // member: system_healthy
  {
    out << "system_healthy: ";
    rosidl_generator_traits::value_to_yaml(msg.system_healthy, out);
    out << ", ";
  }

  // member: health_status
  {
    out << "health_status: ";
    rosidl_generator_traits::value_to_yaml(msg.health_status, out);
    out << ", ";
  }

  // member: health_score
  {
    out << "health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.health_score, out);
    out << ", ";
  }

  // member: update_rate_hz
  {
    out << "update_rate_hz: ";
    rosidl_generator_traits::value_to_yaml(msg.update_rate_hz, out);
    out << ", ";
  }

  // member: dropped_updates
  {
    out << "dropped_updates: ";
    rosidl_generator_traits::value_to_yaml(msg.dropped_updates, out);
    out << ", ";
  }

  // member: processing_latency
  {
    out << "processing_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.processing_latency, out);
    out << ", ";
  }

  // member: active_alerts
  {
    if (msg.active_alerts.size() == 0) {
      out << "active_alerts: []";
    } else {
      out << "active_alerts: [";
      size_t pending_items = msg.active_alerts.size();
      for (auto item : msg.active_alerts) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: alert_count
  {
    out << "alert_count: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_count, out);
    out << ", ";
  }

  // member: last_alert_time
  {
    out << "last_alert_time: ";
    to_flow_style_yaml(msg.last_alert_time, out);
    out << ", ";
  }

  // member: serial_sensors
  {
    out << "serial_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_sensors, out);
    out << ", ";
  }

  // member: can_sensors
  {
    out << "can_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.can_sensors, out);
    out << ", ";
  }

  // member: ethernet_sensors
  {
    out << "ethernet_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.ethernet_sensors, out);
    out << ", ";
  }

  // member: local_sensors
  {
    out << "local_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.local_sensors, out);
    out << ", ";
  }

  // member: avg_network_latency
  {
    out << "avg_network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_network_latency, out);
    out << ", ";
  }

  // member: max_network_latency
  {
    out << "max_network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.max_network_latency, out);
    out << ", ";
  }

  // member: congested_links
  {
    out << "congested_links: ";
    rosidl_generator_traits::value_to_yaml(msg.congested_links, out);
    out << ", ";
  }

  // member: network_recommendations
  {
    if (msg.network_recommendations.size() == 0) {
      out << "network_recommendations: []";
    } else {
      out << "network_recommendations: [";
      size_t pending_items = msg.network_recommendations.size();
      for (auto item : msg.network_recommendations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: network_health_score
  {
    out << "network_health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.network_health_score, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AOIMetrics & msg,
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

  // member: system_average_aoi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_average_aoi: ";
    rosidl_generator_traits::value_to_yaml(msg.system_average_aoi, out);
    out << "\n";
  }

  // member: total_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.total_sensors, out);
    out << "\n";
  }

  // member: fresh_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fresh_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.fresh_sensors, out);
    out << "\n";
  }

  // member: stale_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stale_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.stale_sensors, out);
    out << "\n";
  }

  // member: critical_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "critical_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.critical_sensors, out);
    out << "\n";
  }

  // member: aoi_p50
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoi_p50: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p50, out);
    out << "\n";
  }

  // member: aoi_p90
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoi_p90: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p90, out);
    out << "\n";
  }

  // member: aoi_p95
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoi_p95: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p95, out);
    out << "\n";
  }

  // member: aoi_p99
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoi_p99: ";
    rosidl_generator_traits::value_to_yaml(msg.aoi_p99, out);
    out << "\n";
  }

  // member: system_healthy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_healthy: ";
    rosidl_generator_traits::value_to_yaml(msg.system_healthy, out);
    out << "\n";
  }

  // member: health_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health_status: ";
    rosidl_generator_traits::value_to_yaml(msg.health_status, out);
    out << "\n";
  }

  // member: health_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.health_score, out);
    out << "\n";
  }

  // member: update_rate_hz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "update_rate_hz: ";
    rosidl_generator_traits::value_to_yaml(msg.update_rate_hz, out);
    out << "\n";
  }

  // member: dropped_updates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dropped_updates: ";
    rosidl_generator_traits::value_to_yaml(msg.dropped_updates, out);
    out << "\n";
  }

  // member: processing_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "processing_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.processing_latency, out);
    out << "\n";
  }

  // member: active_alerts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_alerts.size() == 0) {
      out << "active_alerts: []\n";
    } else {
      out << "active_alerts:\n";
      for (auto item : msg.active_alerts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: alert_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alert_count: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_count, out);
    out << "\n";
  }

  // member: last_alert_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_alert_time:\n";
    to_block_style_yaml(msg.last_alert_time, out, indentation + 2);
  }

  // member: serial_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_sensors, out);
    out << "\n";
  }

  // member: can_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.can_sensors, out);
    out << "\n";
  }

  // member: ethernet_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ethernet_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.ethernet_sensors, out);
    out << "\n";
  }

  // member: local_sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_sensors: ";
    rosidl_generator_traits::value_to_yaml(msg.local_sensors, out);
    out << "\n";
  }

  // member: avg_network_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_network_latency, out);
    out << "\n";
  }

  // member: max_network_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_network_latency: ";
    rosidl_generator_traits::value_to_yaml(msg.max_network_latency, out);
    out << "\n";
  }

  // member: congested_links
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "congested_links: ";
    rosidl_generator_traits::value_to_yaml(msg.congested_links, out);
    out << "\n";
  }

  // member: network_recommendations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.network_recommendations.size() == 0) {
      out << "network_recommendations: []\n";
    } else {
      out << "network_recommendations:\n";
      for (auto item : msg.network_recommendations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: network_health_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "network_health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.network_health_score, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AOIMetrics & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::msg::AOIMetrics & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::msg::AOIMetrics & msg)
{
  return autonomy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::msg::AOIMetrics>()
{
  return "autonomy_interfaces::msg::AOIMetrics";
}

template<>
inline const char * name<autonomy_interfaces::msg::AOIMetrics>()
{
  return "autonomy_interfaces/msg/AOIMetrics";
}

template<>
struct has_fixed_size<autonomy_interfaces::msg::AOIMetrics>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::msg::AOIMetrics>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::msg::AOIMetrics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__TRAITS_HPP_
