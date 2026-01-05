// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/recover_from_safety.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__TRAITS_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonomy_interfaces/srv/detail/recover_from_safety__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RecoverFromSafety_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: recovery_method
  {
    out << "recovery_method: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_method, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << ", ";
  }

  // member: acknowledge_risks
  {
    out << "acknowledge_risks: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_risks, out);
    out << ", ";
  }

  // member: completed_steps
  {
    if (msg.completed_steps.size() == 0) {
      out << "completed_steps: []";
    } else {
      out << "completed_steps: [";
      size_t pending_items = msg.completed_steps.size();
      for (auto item : msg.completed_steps) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: notes
  {
    out << "notes: ";
    rosidl_generator_traits::value_to_yaml(msg.notes, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RecoverFromSafety_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: recovery_method
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_method: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_method, out);
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

  // member: acknowledge_risks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledge_risks: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledge_risks, out);
    out << "\n";
  }

  // member: completed_steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.completed_steps.size() == 0) {
      out << "completed_steps: []\n";
    } else {
      out << "completed_steps:\n";
      for (auto item : msg.completed_steps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: notes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "notes: ";
    rosidl_generator_traits::value_to_yaml(msg.notes, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RecoverFromSafety_Request & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::RecoverFromSafety_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::RecoverFromSafety_Request & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::RecoverFromSafety_Request>()
{
  return "autonomy_interfaces::srv::RecoverFromSafety_Request";
}

template<>
inline const char * name<autonomy_interfaces::srv::RecoverFromSafety_Request>()
{
  return "autonomy_interfaces/srv/RecoverFromSafety_Request";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::RecoverFromSafety_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autonomy_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RecoverFromSafety_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: recovery_state
  {
    out << "recovery_state: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_state, out);
    out << ", ";
  }

  // member: is_safe_to_proceed
  {
    out << "is_safe_to_proceed: ";
    rosidl_generator_traits::value_to_yaml(msg.is_safe_to_proceed, out);
    out << ", ";
  }

  // member: remaining_steps
  {
    if (msg.remaining_steps.size() == 0) {
      out << "remaining_steps: []";
    } else {
      out << "remaining_steps: [";
      size_t pending_items = msg.remaining_steps.size();
      for (auto item : msg.remaining_steps) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: verified_systems
  {
    if (msg.verified_systems.size() == 0) {
      out << "verified_systems: []";
    } else {
      out << "verified_systems: [";
      size_t pending_items = msg.verified_systems.size();
      for (auto item : msg.verified_systems) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: failed_systems
  {
    if (msg.failed_systems.size() == 0) {
      out << "failed_systems: []";
    } else {
      out << "failed_systems: [";
      size_t pending_items = msg.failed_systems.size();
      for (auto item : msg.failed_systems) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: estimated_time
  {
    out << "estimated_time: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time, out);
    out << ", ";
  }

  // member: recommended_next_state
  {
    out << "recommended_next_state: ";
    rosidl_generator_traits::value_to_yaml(msg.recommended_next_state, out);
    out << ", ";
  }

  // member: restrictions
  {
    if (msg.restrictions.size() == 0) {
      out << "restrictions: []";
    } else {
      out << "restrictions: [";
      size_t pending_items = msg.restrictions.size();
      for (auto item : msg.restrictions) {
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
  const RecoverFromSafety_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: recovery_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_state: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_state, out);
    out << "\n";
  }

  // member: is_safe_to_proceed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_safe_to_proceed: ";
    rosidl_generator_traits::value_to_yaml(msg.is_safe_to_proceed, out);
    out << "\n";
  }

  // member: remaining_steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.remaining_steps.size() == 0) {
      out << "remaining_steps: []\n";
    } else {
      out << "remaining_steps:\n";
      for (auto item : msg.remaining_steps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: verified_systems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.verified_systems.size() == 0) {
      out << "verified_systems: []\n";
    } else {
      out << "verified_systems:\n";
      for (auto item : msg.verified_systems) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: failed_systems
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.failed_systems.size() == 0) {
      out << "failed_systems: []\n";
    } else {
      out << "failed_systems:\n";
      for (auto item : msg.failed_systems) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: estimated_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time, out);
    out << "\n";
  }

  // member: recommended_next_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recommended_next_state: ";
    rosidl_generator_traits::value_to_yaml(msg.recommended_next_state, out);
    out << "\n";
  }

  // member: restrictions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.restrictions.size() == 0) {
      out << "restrictions: []\n";
    } else {
      out << "restrictions:\n";
      for (auto item : msg.restrictions) {
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

inline std::string to_yaml(const RecoverFromSafety_Response & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::RecoverFromSafety_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::RecoverFromSafety_Response & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::RecoverFromSafety_Response>()
{
  return "autonomy_interfaces::srv::RecoverFromSafety_Response";
}

template<>
inline const char * name<autonomy_interfaces::srv::RecoverFromSafety_Response>()
{
  return "autonomy_interfaces/srv/RecoverFromSafety_Response";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonomy_interfaces::srv::RecoverFromSafety_Response>
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
  const RecoverFromSafety_Event & msg,
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
  const RecoverFromSafety_Event & msg,
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

inline std::string to_yaml(const RecoverFromSafety_Event & msg, bool use_flow_style = false)
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
  const autonomy_interfaces::srv::RecoverFromSafety_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonomy_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonomy_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const autonomy_interfaces::srv::RecoverFromSafety_Event & msg)
{
  return autonomy_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autonomy_interfaces::srv::RecoverFromSafety_Event>()
{
  return "autonomy_interfaces::srv::RecoverFromSafety_Event";
}

template<>
inline const char * name<autonomy_interfaces::srv::RecoverFromSafety_Event>()
{
  return "autonomy_interfaces/srv/RecoverFromSafety_Event";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Event>
  : std::integral_constant<bool, has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Request>::value && has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<autonomy_interfaces::srv::RecoverFromSafety_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autonomy_interfaces::srv::RecoverFromSafety>()
{
  return "autonomy_interfaces::srv::RecoverFromSafety";
}

template<>
inline const char * name<autonomy_interfaces::srv::RecoverFromSafety>()
{
  return "autonomy_interfaces/srv/RecoverFromSafety";
}

template<>
struct has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety>
  : std::integral_constant<
    bool,
    has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety_Request>::value &&
    has_fixed_size<autonomy_interfaces::srv::RecoverFromSafety_Response>::value
  >
{
};

template<>
struct has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety>
  : std::integral_constant<
    bool,
    has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Request>::value &&
    has_bounded_size<autonomy_interfaces::srv::RecoverFromSafety_Response>::value
  >
{
};

template<>
struct is_service<autonomy_interfaces::srv::RecoverFromSafety>
  : std::true_type
{
};

template<>
struct is_service_request<autonomy_interfaces::srv::RecoverFromSafety_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autonomy_interfaces::srv::RecoverFromSafety_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__TRAITS_HPP_
