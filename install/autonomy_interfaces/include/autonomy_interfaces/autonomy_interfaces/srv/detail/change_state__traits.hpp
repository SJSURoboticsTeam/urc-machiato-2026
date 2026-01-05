// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/change_state.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/change_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChangeState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: desired_state
  {
    out << "desired_state: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_state, out);
    out << ", ";
  }

  // member: desired_substate
  {
    out << "desired_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_substate, out);
    out << ", ";
  }

  // member: desired_calibration_substate
  {
    out << "desired_calibration_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_calibration_substate, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << ", ";
  }

  // member: metadata
  {
    if (msg.metadata.size() == 0) {
      out << "metadata: []";
    } else {
      out << "metadata: [";
      size_t pending_items = msg.metadata.size();
      for (auto item : msg.metadata) {
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
  const ChangeState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: desired_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_state: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_state, out);
    out << "\n";
  }

  // member: desired_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_substate, out);
    out << "\n";
  }

  // member: desired_calibration_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_calibration_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_calibration_substate, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: operator_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << "\n";
  }

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }

  // member: metadata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.metadata.size() == 0) {
      out << "metadata: []\n";
    } else {
      out << "metadata:\n";
      for (auto item : msg.metadata) {
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

inline std::string to_yaml(const ChangeState_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ChangeState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ChangeState_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ChangeState_Request>()
{
  return "autonomy_interfaces::srv::ChangeState_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::ChangeState_Request>()
{
  return "autonomy_interfaces/srv/ChangeState_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ChangeState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ChangeState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ChangeState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChangeState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: actual_state
  {
    out << "actual_state: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_state, out);
    out << ", ";
  }

  // member: actual_substate
  {
    out << "actual_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_substate, out);
    out << ", ";
  }

  // member: actual_calibration_substate
  {
    out << "actual_calibration_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_calibration_substate, out);
    out << ", ";
  }

  // member: transition_time
  {
    out << "transition_time: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_time, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: preconditions_met
  {
    out << "preconditions_met: ";
    rosidl_generator_traits::value_to_yaml(msg.preconditions_met, out);
    out << ", ";
  }

  // member: failed_preconditions
  {
    if (msg.failed_preconditions.size() == 0) {
      out << "failed_preconditions: []";
    } else {
      out << "failed_preconditions: [";
      size_t pending_items = msg.failed_preconditions.size();
      for (auto item : msg.failed_preconditions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: warnings
  {
    if (msg.warnings.size() == 0) {
      out << "warnings: []";
    } else {
      out << "warnings: [";
      size_t pending_items = msg.warnings.size();
      for (auto item : msg.warnings) {
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
  const ChangeState_Response & msg,
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

  // member: actual_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual_state: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_state, out);
    out << "\n";
  }

  // member: actual_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_substate, out);
    out << "\n";
  }

  // member: actual_calibration_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual_calibration_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_calibration_substate, out);
    out << "\n";
  }

  // member: transition_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transition_time: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_time, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: preconditions_met
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "preconditions_met: ";
    rosidl_generator_traits::value_to_yaml(msg.preconditions_met, out);
    out << "\n";
  }

  // member: failed_preconditions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.failed_preconditions.size() == 0) {
      out << "failed_preconditions: []\n";
    } else {
      out << "failed_preconditions:\n";
      for (auto item : msg.failed_preconditions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: warnings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.warnings.size() == 0) {
      out << "warnings: []\n";
    } else {
      out << "warnings:\n";
      for (auto item : msg.warnings) {
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

inline std::string to_yaml(const ChangeState_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ChangeState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ChangeState_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ChangeState_Response>()
{
  return "autonomy_interfaces::srv::ChangeState_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::ChangeState_Response>()
{
  return "autonomy_interfaces/srv/ChangeState_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ChangeState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ChangeState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::ChangeState_Response>
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
  const ChangeState_Event & msg,
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
  const ChangeState_Event & msg,
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

inline std::string to_yaml(const ChangeState_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::ChangeState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::ChangeState_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::ChangeState_Event>()
{
  return "autonomy_interfaces::srv::ChangeState_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::ChangeState_Event>()
{
  return "autonomy_interfaces/srv/ChangeState_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ChangeState_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ChangeState_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::ChangeState_Request>::value && has_bounded_size<autonomy_interfaces::srv::ChangeState_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::ChangeState_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::ChangeState>()
{
  return "autonomy_interfaces::srv::ChangeState";
}

template<>
inline const char * name<autonomy_interfaces::srv::ChangeState>()
{
  return "autonomy_interfaces/srv/ChangeState";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::ChangeState>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::ChangeState_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::ChangeState_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::ChangeState>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::ChangeState_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::ChangeState_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::ChangeState>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::ChangeState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::ChangeState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_
