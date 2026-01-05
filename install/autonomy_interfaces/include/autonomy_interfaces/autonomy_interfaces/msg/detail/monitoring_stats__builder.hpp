// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/monitoring_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/monitoring_stats__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_MonitoringStats_evaluation_rate
{
public:
  explicit Init_MonitoringStats_evaluation_rate(::autonomy_interfaces::msg::MonitoringStats & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::MonitoringStats evaluation_rate(::autonomy_interfaces::msg::MonitoringStats::_evaluation_rate_type arg)
  {
    msg_.evaluation_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::MonitoringStats msg_;
};

class Init_MonitoringStats_total_violations
{
public:
  explicit Init_MonitoringStats_total_violations(::autonomy_interfaces::msg::MonitoringStats & msg)
  : msg_(msg)
  {}
  Init_MonitoringStats_evaluation_rate total_violations(::autonomy_interfaces::msg::MonitoringStats::_total_violations_type arg)
  {
    msg_.total_violations = std::move(arg);
    return Init_MonitoringStats_evaluation_rate(msg_);
  }

private:
  ::autonomy_interfaces::msg::MonitoringStats msg_;
};

class Init_MonitoringStats_total_evaluations
{
public:
  Init_MonitoringStats_total_evaluations()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MonitoringStats_total_violations total_evaluations(::autonomy_interfaces::msg::MonitoringStats::_total_evaluations_type arg)
  {
    msg_.total_evaluations = std::move(arg);
    return Init_MonitoringStats_total_violations(msg_);
  }

private:
  ::autonomy_interfaces::msg::MonitoringStats msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::MonitoringStats>()
{
  return autonomy_interfaces::msg::builder::Init_MonitoringStats_total_evaluations();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__BUILDER_HPP_
