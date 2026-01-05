// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/timing_safety_check.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/timing_safety_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_TimingSafetyCheck_Request_monitored_components
{
public:
  explicit Init_TimingSafetyCheck_Request_monitored_components(::autonomy_interfaces::srv::TimingSafetyCheck_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::TimingSafetyCheck_Request monitored_components(::autonomy_interfaces::srv::TimingSafetyCheck_Request::_monitored_components_type arg)
  {
    msg_.monitored_components = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Request msg_;
};

class Init_TimingSafetyCheck_Request_time_window
{
public:
  explicit Init_TimingSafetyCheck_Request_time_window(::autonomy_interfaces::srv::TimingSafetyCheck_Request & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Request_monitored_components time_window(::autonomy_interfaces::srv::TimingSafetyCheck_Request::_time_window_type arg)
  {
    msg_.time_window = std::move(arg);
    return Init_TimingSafetyCheck_Request_monitored_components(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Request msg_;
};

class Init_TimingSafetyCheck_Request_real_time_check
{
public:
  Init_TimingSafetyCheck_Request_real_time_check()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimingSafetyCheck_Request_time_window real_time_check(::autonomy_interfaces::srv::TimingSafetyCheck_Request::_real_time_check_type arg)
  {
    msg_.real_time_check = std::move(arg);
    return Init_TimingSafetyCheck_Request_time_window(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::TimingSafetyCheck_Request>()
{
  return autonomy_interfaces::srv::builder::Init_TimingSafetyCheck_Request_real_time_check();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_TimingSafetyCheck_Response_timestamp
{
public:
  explicit Init_TimingSafetyCheck_Response_timestamp(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response timestamp(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_timing_recommendations
{
public:
  explicit Init_TimingSafetyCheck_Response_timing_recommendations(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_timestamp timing_recommendations(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_timing_recommendations_type arg)
  {
    msg_.timing_recommendations = std::move(arg);
    return Init_TimingSafetyCheck_Response_timestamp(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_real_time_scheduling
{
public:
  explicit Init_TimingSafetyCheck_Response_real_time_scheduling(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_timing_recommendations real_time_scheduling(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_real_time_scheduling_type arg)
  {
    msg_.real_time_scheduling = std::move(arg);
    return Init_TimingSafetyCheck_Response_timing_recommendations(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_thread_count
{
public:
  explicit Init_TimingSafetyCheck_Response_thread_count(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_real_time_scheduling thread_count(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_thread_count_type arg)
  {
    msg_.thread_count = std::move(arg);
    return Init_TimingSafetyCheck_Response_real_time_scheduling(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_memory_utilization
{
public:
  explicit Init_TimingSafetyCheck_Response_memory_utilization(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_thread_count memory_utilization(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_memory_utilization_type arg)
  {
    msg_.memory_utilization = std::move(arg);
    return Init_TimingSafetyCheck_Response_thread_count(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_cpu_utilization
{
public:
  explicit Init_TimingSafetyCheck_Response_cpu_utilization(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_memory_utilization cpu_utilization(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_cpu_utilization_type arg)
  {
    msg_.cpu_utilization = std::move(arg);
    return Init_TimingSafetyCheck_Response_memory_utilization(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_component_deadlines_missed
{
public:
  explicit Init_TimingSafetyCheck_Response_component_deadlines_missed(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_cpu_utilization component_deadlines_missed(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_component_deadlines_missed_type arg)
  {
    msg_.component_deadlines_missed = std::move(arg);
    return Init_TimingSafetyCheck_Response_cpu_utilization(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_component_avg_times
{
public:
  explicit Init_TimingSafetyCheck_Response_component_avg_times(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_component_deadlines_missed component_avg_times(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_component_avg_times_type arg)
  {
    msg_.component_avg_times = std::move(arg);
    return Init_TimingSafetyCheck_Response_component_deadlines_missed(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_components_checked
{
public:
  explicit Init_TimingSafetyCheck_Response_components_checked(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_component_avg_times components_checked(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_components_checked_type arg)
  {
    msg_.components_checked = std::move(arg);
    return Init_TimingSafetyCheck_Response_component_avg_times(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_deadline_miss_rate
{
public:
  explicit Init_TimingSafetyCheck_Response_deadline_miss_rate(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_components_checked deadline_miss_rate(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_deadline_miss_rate_type arg)
  {
    msg_.deadline_miss_rate = std::move(arg);
    return Init_TimingSafetyCheck_Response_components_checked(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_deadline_misses
{
public:
  explicit Init_TimingSafetyCheck_Response_deadline_misses(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_deadline_miss_rate deadline_misses(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_deadline_misses_type arg)
  {
    msg_.deadline_misses = std::move(arg);
    return Init_TimingSafetyCheck_Response_deadline_miss_rate(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_jitter
{
public:
  explicit Init_TimingSafetyCheck_Response_jitter(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_deadline_misses jitter(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_jitter_type arg)
  {
    msg_.jitter = std::move(arg);
    return Init_TimingSafetyCheck_Response_deadline_misses(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_min_response_time
{
public:
  explicit Init_TimingSafetyCheck_Response_min_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_jitter min_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_min_response_time_type arg)
  {
    msg_.min_response_time = std::move(arg);
    return Init_TimingSafetyCheck_Response_jitter(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_max_response_time
{
public:
  explicit Init_TimingSafetyCheck_Response_max_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_min_response_time max_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_max_response_time_type arg)
  {
    msg_.max_response_time = std::move(arg);
    return Init_TimingSafetyCheck_Response_min_response_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_avg_response_time
{
public:
  explicit Init_TimingSafetyCheck_Response_avg_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_max_response_time avg_response_time(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_avg_response_time_type arg)
  {
    msg_.avg_response_time = std::move(arg);
    return Init_TimingSafetyCheck_Response_max_response_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_timing_status
{
public:
  explicit Init_TimingSafetyCheck_Response_timing_status(::autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Response_avg_response_time timing_status(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_timing_status_type arg)
  {
    msg_.timing_status = std::move(arg);
    return Init_TimingSafetyCheck_Response_avg_response_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

class Init_TimingSafetyCheck_Response_timing_safe
{
public:
  Init_TimingSafetyCheck_Response_timing_safe()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimingSafetyCheck_Response_timing_status timing_safe(::autonomy_interfaces::srv::TimingSafetyCheck_Response::_timing_safe_type arg)
  {
    msg_.timing_safe = std::move(arg);
    return Init_TimingSafetyCheck_Response_timing_status(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::TimingSafetyCheck_Response>()
{
  return autonomy_interfaces::srv::builder::Init_TimingSafetyCheck_Response_timing_safe();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_TimingSafetyCheck_Event_response
{
public:
  explicit Init_TimingSafetyCheck_Event_response(::autonomy_interfaces::srv::TimingSafetyCheck_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::TimingSafetyCheck_Event response(::autonomy_interfaces::srv::TimingSafetyCheck_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Event msg_;
};

class Init_TimingSafetyCheck_Event_request
{
public:
  explicit Init_TimingSafetyCheck_Event_request(::autonomy_interfaces::srv::TimingSafetyCheck_Event & msg)
  : msg_(msg)
  {}
  Init_TimingSafetyCheck_Event_response request(::autonomy_interfaces::srv::TimingSafetyCheck_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TimingSafetyCheck_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Event msg_;
};

class Init_TimingSafetyCheck_Event_info
{
public:
  Init_TimingSafetyCheck_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimingSafetyCheck_Event_request info(::autonomy_interfaces::srv::TimingSafetyCheck_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TimingSafetyCheck_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::TimingSafetyCheck_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::TimingSafetyCheck_Event>()
{
  return autonomy_interfaces::srv::builder::Init_TimingSafetyCheck_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__BUILDER_HPP_
