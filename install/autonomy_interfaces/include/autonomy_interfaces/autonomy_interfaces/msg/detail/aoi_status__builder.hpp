// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/aoi_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/aoi_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_AOIStatus_aoi_trend
{
public:
  explicit Init_AOIStatus_aoi_trend(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::AOIStatus aoi_trend(::autonomy_interfaces::msg::AOIStatus::_aoi_trend_type arg)
  {
    msg_.aoi_trend = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_predicted_aoi
{
public:
  explicit Init_AOIStatus_predicted_aoi(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_aoi_trend predicted_aoi(::autonomy_interfaces::msg::AOIStatus::_predicted_aoi_type arg)
  {
    msg_.predicted_aoi = std::move(arg);
    return Init_AOIStatus_aoi_trend(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_congestion_factor
{
public:
  explicit Init_AOIStatus_congestion_factor(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_predicted_aoi congestion_factor(::autonomy_interfaces::msg::AOIStatus::_congestion_factor_type arg)
  {
    msg_.congestion_factor = std::move(arg);
    return Init_AOIStatus_predicted_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_congestion_detected
{
public:
  explicit Init_AOIStatus_congestion_detected(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_congestion_factor congestion_detected(::autonomy_interfaces::msg::AOIStatus::_congestion_detected_type arg)
  {
    msg_.congestion_detected = std::move(arg);
    return Init_AOIStatus_congestion_factor(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_transport_latency
{
public:
  explicit Init_AOIStatus_transport_latency(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_congestion_detected transport_latency(::autonomy_interfaces::msg::AOIStatus::_transport_latency_type arg)
  {
    msg_.transport_latency = std::move(arg);
    return Init_AOIStatus_congestion_detected(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_network_latency
{
public:
  explicit Init_AOIStatus_network_latency(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_transport_latency network_latency(::autonomy_interfaces::msg::AOIStatus::_network_latency_type arg)
  {
    msg_.network_latency = std::move(arg);
    return Init_AOIStatus_transport_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_transport_type
{
public:
  explicit Init_AOIStatus_transport_type(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_network_latency transport_type(::autonomy_interfaces::msg::AOIStatus::_transport_type_type arg)
  {
    msg_.transport_type = std::move(arg);
    return Init_AOIStatus_network_latency(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_freshness_ratio
{
public:
  explicit Init_AOIStatus_freshness_ratio(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_transport_type freshness_ratio(::autonomy_interfaces::msg::AOIStatus::_freshness_ratio_type arg)
  {
    msg_.freshness_ratio = std::move(arg);
    return Init_AOIStatus_transport_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_sample_count
{
public:
  explicit Init_AOIStatus_sample_count(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_freshness_ratio sample_count(::autonomy_interfaces::msg::AOIStatus::_sample_count_type arg)
  {
    msg_.sample_count = std::move(arg);
    return Init_AOIStatus_freshness_ratio(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_optimal_threshold
{
public:
  explicit Init_AOIStatus_optimal_threshold(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_sample_count optimal_threshold(::autonomy_interfaces::msg::AOIStatus::_optimal_threshold_type arg)
  {
    msg_.optimal_threshold = std::move(arg);
    return Init_AOIStatus_sample_count(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_acceptable_threshold
{
public:
  explicit Init_AOIStatus_acceptable_threshold(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_optimal_threshold acceptable_threshold(::autonomy_interfaces::msg::AOIStatus::_acceptable_threshold_type arg)
  {
    msg_.acceptable_threshold = std::move(arg);
    return Init_AOIStatus_optimal_threshold(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_freshness_status
{
public:
  explicit Init_AOIStatus_freshness_status(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_acceptable_threshold freshness_status(::autonomy_interfaces::msg::AOIStatus::_freshness_status_type arg)
  {
    msg_.freshness_status = std::move(arg);
    return Init_AOIStatus_acceptable_threshold(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_quality_score
{
public:
  explicit Init_AOIStatus_quality_score(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_freshness_status quality_score(::autonomy_interfaces::msg::AOIStatus::_quality_score_type arg)
  {
    msg_.quality_score = std::move(arg);
    return Init_AOIStatus_freshness_status(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_is_fresh
{
public:
  explicit Init_AOIStatus_is_fresh(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_quality_score is_fresh(::autonomy_interfaces::msg::AOIStatus::_is_fresh_type arg)
  {
    msg_.is_fresh = std::move(arg);
    return Init_AOIStatus_quality_score(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_min_aoi
{
public:
  explicit Init_AOIStatus_min_aoi(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_is_fresh min_aoi(::autonomy_interfaces::msg::AOIStatus::_min_aoi_type arg)
  {
    msg_.min_aoi = std::move(arg);
    return Init_AOIStatus_is_fresh(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_max_aoi
{
public:
  explicit Init_AOIStatus_max_aoi(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_min_aoi max_aoi(::autonomy_interfaces::msg::AOIStatus::_max_aoi_type arg)
  {
    msg_.max_aoi = std::move(arg);
    return Init_AOIStatus_min_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_average_aoi
{
public:
  explicit Init_AOIStatus_average_aoi(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_max_aoi average_aoi(::autonomy_interfaces::msg::AOIStatus::_average_aoi_type arg)
  {
    msg_.average_aoi = std::move(arg);
    return Init_AOIStatus_max_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_current_aoi
{
public:
  explicit Init_AOIStatus_current_aoi(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_average_aoi current_aoi(::autonomy_interfaces::msg::AOIStatus::_current_aoi_type arg)
  {
    msg_.current_aoi = std::move(arg);
    return Init_AOIStatus_average_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_sensor_type
{
public:
  explicit Init_AOIStatus_sensor_type(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_current_aoi sensor_type(::autonomy_interfaces::msg::AOIStatus::_sensor_type_type arg)
  {
    msg_.sensor_type = std::move(arg);
    return Init_AOIStatus_current_aoi(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_sensor_name
{
public:
  explicit Init_AOIStatus_sensor_name(::autonomy_interfaces::msg::AOIStatus & msg)
  : msg_(msg)
  {}
  Init_AOIStatus_sensor_type sensor_name(::autonomy_interfaces::msg::AOIStatus::_sensor_name_type arg)
  {
    msg_.sensor_name = std::move(arg);
    return Init_AOIStatus_sensor_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

class Init_AOIStatus_header
{
public:
  Init_AOIStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AOIStatus_sensor_name header(::autonomy_interfaces::msg::AOIStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AOIStatus_sensor_name(msg_);
  }

private:
  ::autonomy_interfaces::msg::AOIStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::AOIStatus>()
{
  return autonomy_interfaces::msg::builder::Init_AOIStatus_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__BUILDER_HPP_
