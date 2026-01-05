// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:action/ExecuteMission.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/action/execute_mission.hpp"


#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__TRAITS_HPP_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/action/detail/execute_mission__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: mission_type
  {
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
    out << ", ";
  }

  // member: mission_id
  {
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << ", ";
  }

  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mission_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_type: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_type, out);
    out << "\n";
  }

  // member: mission_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << "\n";
  }

  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_Goal & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_Goal>()
{
  return "autonomy_interfaces::action::ExecuteMission_Goal";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_Goal>()
{
  return "autonomy_interfaces/action/ExecuteMission_Goal";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_phase
  {
    out << "current_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.current_phase, out);
    out << ", ";
  }

  // member: progress
  {
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << ", ";
  }

  // member: waypoints_completed
  {
    out << "waypoints_completed: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_completed, out);
    out << ", ";
  }

  // member: estimated_time_remaining
  {
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.current_phase, out);
    out << "\n";
  }

  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << "\n";
  }

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }

  // member: waypoints_completed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoints_completed: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_completed, out);
    out << "\n";
  }

  // member: estimated_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_Result & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_Result>()
{
  return "autonomy_interfaces::action::ExecuteMission_Result";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_Result>()
{
  return "autonomy_interfaces/action/ExecuteMission_Result";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: completion_status
  {
    out << "completion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_status, out);
    out << ", ";
  }

  // member: completed_tasks
  {
    if (msg.completed_tasks.size() == 0) {
      out << "completed_tasks: []";
    } else {
      out << "completed_tasks: [";
      size_t pending_items = msg.completed_tasks.size();
      for (auto item : msg.completed_tasks) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: total_time
  {
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
    out << ", ";
  }

  // member: waypoints_visited
  {
    out << "waypoints_visited: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_visited, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_Feedback & msg,
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

  // member: completion_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "completion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_status, out);
    out << "\n";
  }

  // member: completed_tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.completed_tasks.size() == 0) {
      out << "completed_tasks: []\n";
    } else {
      out << "completed_tasks:\n";
      for (auto item : msg.completed_tasks) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: total_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
    out << "\n";
  }

  // member: waypoints_visited
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoints_visited: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_visited, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_Feedback & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_Feedback>()
{
  return "autonomy_interfaces::action::ExecuteMission_Feedback";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_Feedback>()
{
  return "autonomy_interfaces/action/ExecuteMission_Feedback";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "autonomy_interfaces/action/detail/execute_mission__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_SendGoal_Request & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>()
{
  return "autonomy_interfaces::action::ExecuteMission_SendGoal_Request";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>()
{
  return "autonomy_interfaces/action/ExecuteMission_SendGoal_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<autonomy_interfaces::action::ExecuteMission_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::action::ExecuteMission_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_SendGoal_Response & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>()
{
  return "autonomy_interfaces::action::ExecuteMission_SendGoal_Response";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>()
{
  return "autonomy_interfaces/action/ExecuteMission_SendGoal_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_SendGoal_Event & msg,
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
  const ExecuteMission_SendGoal_Event & msg,
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

inline std::string to_yaml(const ExecuteMission_SendGoal_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_SendGoal_Event & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_SendGoal_Event>()
{
  return "autonomy_interfaces::action::ExecuteMission_SendGoal_Event";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_SendGoal_Event>()
{
  return "autonomy_interfaces/action/ExecuteMission_SendGoal_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>::value && has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_SendGoal>()
{
  return "autonomy_interfaces::action::ExecuteMission_SendGoal";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_SendGoal>()
{
  return "autonomy_interfaces/action/ExecuteMission_SendGoal";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>::value &&
    has_fixed_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>::value &&
    has_bounded_size<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::action::ExecuteMission_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::action::ExecuteMission_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::action::ExecuteMission_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_GetResult_Request & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_GetResult_Request>()
{
  return "autonomy_interfaces::action::ExecuteMission_GetResult_Request";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_GetResult_Request>()
{
  return "autonomy_interfaces/action/ExecuteMission_GetResult_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_GetResult_Response & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_GetResult_Response>()
{
  return "autonomy_interfaces::action::ExecuteMission_GetResult_Response";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_GetResult_Response>()
{
  return "autonomy_interfaces/action/ExecuteMission_GetResult_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<autonomy_interfaces::action::ExecuteMission_Result>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::action::ExecuteMission_Result>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_GetResult_Event & msg,
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
  const ExecuteMission_GetResult_Event & msg,
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

inline std::string to_yaml(const ExecuteMission_GetResult_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_GetResult_Event & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_GetResult_Event>()
{
  return "autonomy_interfaces::action::ExecuteMission_GetResult_Event";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_GetResult_Event>()
{
  return "autonomy_interfaces/action/ExecuteMission_GetResult_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Request>::value && has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_GetResult>()
{
  return "autonomy_interfaces::action::ExecuteMission_GetResult";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_GetResult>()
{
  return "autonomy_interfaces/action/ExecuteMission_GetResult";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult_Request>::value &&
    has_fixed_size<autonomy_interfaces::action::ExecuteMission_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Request>::value &&
    has_bounded_size<autonomy_interfaces::action::ExecuteMission_GetResult_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::action::ExecuteMission_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::action::ExecuteMission_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::action::ExecuteMission_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__traits.hpp"

namespace autonomy_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMission_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMission_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMission_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace autonomy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonomy_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonomy_interfaces::action::ExecuteMission_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::action::ExecuteMission_FeedbackMessage & msg)
{
  return autonomy_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::action::ExecuteMission_FeedbackMessage>()
{
  return "autonomy_interfaces::action::ExecuteMission_FeedbackMessage";
}

template<>
inline const char * name<autonomy_interfaces::action::ExecuteMission_FeedbackMessage>()
{
  return "autonomy_interfaces/action/ExecuteMission_FeedbackMessage";
}

template<>
struct has_fixed_size<autonomy_interfaces::action::ExecuteMission_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<autonomy_interfaces::action::ExecuteMission_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<autonomy_interfaces::action::ExecuteMission_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::action::ExecuteMission_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<autonomy_interfaces::action::ExecuteMission_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<autonomy_interfaces::action::ExecuteMission>
  : std::true_type
{
};

template<>
struct is_action_goal<autonomy_interfaces::action::ExecuteMission_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<autonomy_interfaces::action::ExecuteMission_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<autonomy_interfaces::action::ExecuteMission_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__TRAITS_HPP_
