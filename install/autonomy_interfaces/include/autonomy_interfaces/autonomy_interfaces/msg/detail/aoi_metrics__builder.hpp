// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/aoi_metrics.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/aoi_metrics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_AOIMetrics_network_health_score
{
public:
  explicit Init_AOIMetrics_network_health_score(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::AOIMetrics network_health_score(::autonomy_interfaces::msg::AOIMetrics::_network_health_score_type arg)
  {
    msg_.network_health_score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_network_recommendations
{
public:
  explicit Init_AOIMetrics_network_recommendations(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_network_health_score network_recommendations(::autonomy_interfaces::msg::AOIMetrics::_network_recommendations_type arg)
  {
    msg_.network_recommendations = std::move(arg);
    return Init_AOIMetrics_network_health_score(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_congested_links
{
public:
  explicit Init_AOIMetrics_congested_links(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_network_recommendations congested_links(::autonomy_interfaces::msg::AOIMetrics::_congested_links_type arg)
  {
    msg_.congested_links = std::move(arg);
    return Init_AOIMetrics_network_recommendations(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_max_network_latency
{
public:
  explicit Init_AOIMetrics_max_network_latency(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_congested_links max_network_latency(::autonomy_interfaces::msg::AOIMetrics::_max_network_latency_type arg)
  {
    msg_.max_network_latency = std::move(arg);
    return Init_AOIMetrics_congested_links(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_avg_network_latency
{
public:
  explicit Init_AOIMetrics_avg_network_latency(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_max_network_latency avg_network_latency(::autonomy_interfaces::msg::AOIMetrics::_avg_network_latency_type arg)
  {
    msg_.avg_network_latency = std::move(arg);
    return Init_AOIMetrics_max_network_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_local_sensors
{
public:
  explicit Init_AOIMetrics_local_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_avg_network_latency local_sensors(::autonomy_interfaces::msg::AOIMetrics::_local_sensors_type arg)
  {
    msg_.local_sensors = std::move(arg);
    return Init_AOIMetrics_avg_network_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_ethernet_sensors
{
public:
  explicit Init_AOIMetrics_ethernet_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_local_sensors ethernet_sensors(::autonomy_interfaces::msg::AOIMetrics::_ethernet_sensors_type arg)
  {
    msg_.ethernet_sensors = std::move(arg);
    return Init_AOIMetrics_local_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_can_sensors
{
public:
  explicit Init_AOIMetrics_can_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_ethernet_sensors can_sensors(::autonomy_interfaces::msg::AOIMetrics::_can_sensors_type arg)
  {
    msg_.can_sensors = std::move(arg);
    return Init_AOIMetrics_ethernet_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_serial_sensors
{
public:
  explicit Init_AOIMetrics_serial_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_can_sensors serial_sensors(::autonomy_interfaces::msg::AOIMetrics::_serial_sensors_type arg)
  {
    msg_.serial_sensors = std::move(arg);
    return Init_AOIMetrics_can_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_last_alert_time
{
public:
  explicit Init_AOIMetrics_last_alert_time(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_serial_sensors last_alert_time(::autonomy_interfaces::msg::AOIMetrics::_last_alert_time_type arg)
  {
    msg_.last_alert_time = std::move(arg);
    return Init_AOIMetrics_serial_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_alert_count
{
public:
  explicit Init_AOIMetrics_alert_count(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_last_alert_time alert_count(::autonomy_interfaces::msg::AOIMetrics::_alert_count_type arg)
  {
    msg_.alert_count = std::move(arg);
    return Init_AOIMetrics_last_alert_time(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_active_alerts
{
public:
  explicit Init_AOIMetrics_active_alerts(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_alert_count active_alerts(::autonomy_interfaces::msg::AOIMetrics::_active_alerts_type arg)
  {
    msg_.active_alerts = std::move(arg);
    return Init_AOIMetrics_alert_count(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_processing_latency
{
public:
  explicit Init_AOIMetrics_processing_latency(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_active_alerts processing_latency(::autonomy_interfaces::msg::AOIMetrics::_processing_latency_type arg)
  {
    msg_.processing_latency = std::move(arg);
    return Init_AOIMetrics_active_alerts(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_dropped_updates
{
public:
  explicit Init_AOIMetrics_dropped_updates(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_processing_latency dropped_updates(::autonomy_interfaces::msg::AOIMetrics::_dropped_updates_type arg)
  {
    msg_.dropped_updates = std::move(arg);
    return Init_AOIMetrics_processing_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_update_rate_hz
{
public:
  explicit Init_AOIMetrics_update_rate_hz(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_dropped_updates update_rate_hz(::autonomy_interfaces::msg::AOIMetrics::_update_rate_hz_type arg)
  {
    msg_.update_rate_hz = std::move(arg);
    return Init_AOIMetrics_dropped_updates(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_health_score
{
public:
  explicit Init_AOIMetrics_health_score(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_update_rate_hz health_score(::autonomy_interfaces::msg::AOIMetrics::_health_score_type arg)
  {
    msg_.health_score = std::move(arg);
    return Init_AOIMetrics_update_rate_hz(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_health_status
{
public:
  explicit Init_AOIMetrics_health_status(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_health_score health_status(::autonomy_interfaces::msg::AOIMetrics::_health_status_type arg)
  {
    msg_.health_status = std::move(arg);
    return Init_AOIMetrics_health_score(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_system_healthy
{
public:
  explicit Init_AOIMetrics_system_healthy(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_health_status system_healthy(::autonomy_interfaces::msg::AOIMetrics::_system_healthy_type arg)
  {
    msg_.system_healthy = std::move(arg);
    return Init_AOIMetrics_health_status(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_aoi_p99
{
public:
  explicit Init_AOIMetrics_aoi_p99(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_system_healthy aoi_p99(::autonomy_interfaces::msg::AOIMetrics::_aoi_p99_type arg)
  {
    msg_.aoi_p99 = std::move(arg);
    return Init_AOIMetrics_system_healthy(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_aoi_p95
{
public:
  explicit Init_AOIMetrics_aoi_p95(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_aoi_p99 aoi_p95(::autonomy_interfaces::msg::AOIMetrics::_aoi_p95_type arg)
  {
    msg_.aoi_p95 = std::move(arg);
    return Init_AOIMetrics_aoi_p99(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_aoi_p90
{
public:
  explicit Init_AOIMetrics_aoi_p90(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_aoi_p95 aoi_p90(::autonomy_interfaces::msg::AOIMetrics::_aoi_p90_type arg)
  {
    msg_.aoi_p90 = std::move(arg);
    return Init_AOIMetrics_aoi_p95(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_aoi_p50
{
public:
  explicit Init_AOIMetrics_aoi_p50(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_aoi_p90 aoi_p50(::autonomy_interfaces::msg::AOIMetrics::_aoi_p50_type arg)
  {
    msg_.aoi_p50 = std::move(arg);
    return Init_AOIMetrics_aoi_p90(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_critical_sensors
{
public:
  explicit Init_AOIMetrics_critical_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_aoi_p50 critical_sensors(::autonomy_interfaces::msg::AOIMetrics::_critical_sensors_type arg)
  {
    msg_.critical_sensors = std::move(arg);
    return Init_AOIMetrics_aoi_p50(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_stale_sensors
{
public:
  explicit Init_AOIMetrics_stale_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_critical_sensors stale_sensors(::autonomy_interfaces::msg::AOIMetrics::_stale_sensors_type arg)
  {
    msg_.stale_sensors = std::move(arg);
    return Init_AOIMetrics_critical_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_fresh_sensors
{
public:
  explicit Init_AOIMetrics_fresh_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_stale_sensors fresh_sensors(::autonomy_interfaces::msg::AOIMetrics::_fresh_sensors_type arg)
  {
    msg_.fresh_sensors = std::move(arg);
    return Init_AOIMetrics_stale_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_total_sensors
{
public:
  explicit Init_AOIMetrics_total_sensors(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_fresh_sensors total_sensors(::autonomy_interfaces::msg::AOIMetrics::_total_sensors_type arg)
  {
    msg_.total_sensors = std::move(arg);
    return Init_AOIMetrics_fresh_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_system_average_aoi
{
public:
  explicit Init_AOIMetrics_system_average_aoi(::autonomy_interfaces::msg::AOIMetrics & msg)
  : msg_(msg)
  {}
  Init_AOIMetrics_total_sensors system_average_aoi(::autonomy_interfaces::msg::AOIMetrics::_system_average_aoi_type arg)
  {
    msg_.system_average_aoi = std::move(arg);
    return Init_AOIMetrics_total_sensors(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

class Init_AOIMetrics_header
{
public:
  Init_AOIMetrics_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AOIMetrics_system_average_aoi header(::autonomy_interfaces::msg::AOIMetrics::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AOIMetrics_system_average_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIMetrics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::AOIMetrics>()
{
  return autonomy_interfaces::msg::builder::Init_AOIMetrics_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__BUILDER_HPP_
