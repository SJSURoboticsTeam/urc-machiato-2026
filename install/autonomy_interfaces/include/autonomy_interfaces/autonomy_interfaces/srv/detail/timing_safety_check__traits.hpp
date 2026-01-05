// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/timing_safety_check.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/timing_safety_check__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TimingSafetyCheck_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: real_time_check
  {
    out << "real_time_check: ";
    rosidl_generator_traits::value_to_yaml(msg.real_time_check, out);
    out << ", ";
  }

  // member: time_window
  {
    out << "time_window: ";
    rosidl_generator_traits::value_to_yaml(msg.time_window, out);
    out << ", ";
  }

  // member: monitored_components
  {
    if (msg.monitored_components.size() == 0) {
      out << "monitored_components: []";
    } else {
      out << "monitored_components: [";
      size_t pending_items = msg.monitored_components.size();
      for (auto item : msg.monitored_components) {
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
  const TimingSafetyCheck_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: real_time_check
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_time_check: ";
    rosidl_generator_traits::value_to_yaml(msg.real_time_check, out);
    out << "\n";
  }

  // member: time_window
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_window: ";
    rosidl_generator_traits::value_to_yaml(msg.time_window, out);
    out << "\n";
  }

  // member: monitored_components
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitored_components.size() == 0) {
      out << "monitored_components: []\n";
    } else {
      out << "monitored_components:\n";
      for (auto item : msg.monitored_components) {
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

inline std::string to_yaml(const TimingSafetyCheck_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::TimingSafetyCheck_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::TimingSafetyCheck_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::TimingSafetyCheck_Request>()
{
  return "autonomy_interfaces::srv::TimingSafetyCheck_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::TimingSafetyCheck_Request>()
{
  return "autonomy_interfaces/srv/TimingSafetyCheck_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::TimingSafetyCheck_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TimingSafetyCheck_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: timing_safe
  {
    out << "timing_safe: ";
    rosidl_generator_traits::value_to_yaml(msg.timing_safe, out);
    out << ", ";
  }

  // member: timing_status
  {
    out << "timing_status: ";
    rosidl_generator_traits::value_to_yaml(msg.timing_status, out);
    out << ", ";
  }

  // member: avg_response_time
  {
    out << "avg_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_response_time, out);
    out << ", ";
  }

  // member: max_response_time
  {
    out << "max_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.max_response_time, out);
    out << ", ";
  }

  // member: min_response_time
  {
    out << "min_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.min_response_time, out);
    out << ", ";
  }

  // member: jitter
  {
    out << "jitter: ";
    rosidl_generator_traits::value_to_yaml(msg.jitter, out);
    out << ", ";
  }

  // member: deadline_misses
  {
    out << "deadline_misses: ";
    rosidl_generator_traits::value_to_yaml(msg.deadline_misses, out);
    out << ", ";
  }

  // member: deadline_miss_rate
  {
    out << "deadline_miss_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.deadline_miss_rate, out);
    out << ", ";
  }

  // member: components_checked
  {
    if (msg.components_checked.size() == 0) {
      out << "components_checked: []";
    } else {
      out << "components_checked: [";
      size_t pending_items = msg.components_checked.size();
      for (auto item : msg.components_checked) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: component_avg_times
  {
    if (msg.component_avg_times.size() == 0) {
      out << "component_avg_times: []";
    } else {
      out << "component_avg_times: [";
      size_t pending_items = msg.component_avg_times.size();
      for (auto item : msg.component_avg_times) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: component_deadlines_missed
  {
    if (msg.component_deadlines_missed.size() == 0) {
      out << "component_deadlines_missed: []";
    } else {
      out << "component_deadlines_missed: [";
      size_t pending_items = msg.component_deadlines_missed.size();
      for (auto item : msg.component_deadlines_missed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cpu_utilization
  {
    out << "cpu_utilization: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_utilization, out);
    out << ", ";
  }

  // member: memory_utilization
  {
    out << "memory_utilization: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_utilization, out);
    out << ", ";
  }

  // member: thread_count
  {
    out << "thread_count: ";
    rosidl_generator_traits::value_to_yaml(msg.thread_count, out);
    out << ", ";
  }

  // member: real_time_scheduling
  {
    out << "real_time_scheduling: ";
    rosidl_generator_traits::value_to_yaml(msg.real_time_scheduling, out);
    out << ", ";
  }

  // member: timing_recommendations
  {
    if (msg.timing_recommendations.size() == 0) {
      out << "timing_recommendations: []";
    } else {
      out << "timing_recommendations: [";
      size_t pending_items = msg.timing_recommendations.size();
      for (auto item : msg.timing_recommendations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TimingSafetyCheck_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timing_safe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timing_safe: ";
    rosidl_generator_traits::value_to_yaml(msg.timing_safe, out);
    out << "\n";
  }

  // member: timing_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timing_status: ";
    rosidl_generator_traits::value_to_yaml(msg.timing_status, out);
    out << "\n";
  }

  // member: avg_response_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_response_time, out);
    out << "\n";
  }

  // member: max_response_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.max_response_time, out);
    out << "\n";
  }

  // member: min_response_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_response_time: ";
    rosidl_generator_traits::value_to_yaml(msg.min_response_time, out);
    out << "\n";
  }

  // member: jitter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jitter: ";
    rosidl_generator_traits::value_to_yaml(msg.jitter, out);
    out << "\n";
  }

  // member: deadline_misses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deadline_misses: ";
    rosidl_generator_traits::value_to_yaml(msg.deadline_misses, out);
    out << "\n";
  }

  // member: deadline_miss_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deadline_miss_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.deadline_miss_rate, out);
    out << "\n";
  }

  // member: components_checked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.components_checked.size() == 0) {
      out << "components_checked: []\n";
    } else {
      out << "components_checked:\n";
      for (auto item : msg.components_checked) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: component_avg_times
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.component_avg_times.size() == 0) {
      out << "component_avg_times: []\n";
    } else {
      out << "component_avg_times:\n";
      for (auto item : msg.component_avg_times) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: component_deadlines_missed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.component_deadlines_missed.size() == 0) {
      out << "component_deadlines_missed: []\n";
    } else {
      out << "component_deadlines_missed:\n";
      for (auto item : msg.component_deadlines_missed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cpu_utilization
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_utilization: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_utilization, out);
    out << "\n";
  }

  // member: memory_utilization
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "memory_utilization: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_utilization, out);
    out << "\n";
  }

  // member: thread_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thread_count: ";
    rosidl_generator_traits::value_to_yaml(msg.thread_count, out);
    out << "\n";
  }

  // member: real_time_scheduling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_time_scheduling: ";
    rosidl_generator_traits::value_to_yaml(msg.real_time_scheduling, out);
    out << "\n";
  }

  // member: timing_recommendations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.timing_recommendations.size() == 0) {
      out << "timing_recommendations: []\n";
    } else {
      out << "timing_recommendations:\n";
      for (auto item : msg.timing_recommendations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TimingSafetyCheck_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::TimingSafetyCheck_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::TimingSafetyCheck_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::TimingSafetyCheck_Response>()
{
  return "autonomy_interfaces::srv::TimingSafetyCheck_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::TimingSafetyCheck_Response>()
{
  return "autonomy_interfaces/srv/TimingSafetyCheck_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::TimingSafetyCheck_Response>
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
  const TimingSafetyCheck_Event & msg,
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
  const TimingSafetyCheck_Event & msg,
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

inline std::string to_yaml(const TimingSafetyCheck_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::TimingSafetyCheck_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::TimingSafetyCheck_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::TimingSafetyCheck_Event>()
{
  return "autonomy_interfaces::srv::TimingSafetyCheck_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::TimingSafetyCheck_Event>()
{
  return "autonomy_interfaces/srv/TimingSafetyCheck_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Request>::value && has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::TimingSafetyCheck_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::TimingSafetyCheck>()
{
  return "autonomy_interfaces::srv::TimingSafetyCheck";
}

template<>
inline const char * name<autonomy_interfaces::srv::TimingSafetyCheck>()
{
  return "autonomy_interfaces/srv/TimingSafetyCheck";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::TimingSafetyCheck_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::TimingSafetyCheck_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::TimingSafetyCheck>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::TimingSafetyCheck_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::TimingSafetyCheck_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__TRAITS_HPP_
