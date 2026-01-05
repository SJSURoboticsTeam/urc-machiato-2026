// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/configure_mission.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/configure_mission__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConfigureMission_Request_max_incline
{
public:
  explicit Init_ConfigureMission_Request_max_incline(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ConfigureMission_Request max_incline(::autonomy_interfaces::srv::ConfigureMission_Request::_max_incline_type arg)
  {
    msg_.max_incline = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_terrain_type
{
public:
  explicit Init_ConfigureMission_Request_terrain_type(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_max_incline terrain_type(::autonomy_interfaces::srv::ConfigureMission_Request::_terrain_type_type arg)
  {
    msg_.terrain_type = std::move(arg);
    return Init_ConfigureMission_Request_max_incline(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_typing_location
{
public:
  explicit Init_ConfigureMission_Request_typing_location(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_terrain_type typing_location(::autonomy_interfaces::srv::ConfigureMission_Request::_typing_location_type arg)
  {
    msg_.typing_location = std::move(arg);
    return Init_ConfigureMission_Request_terrain_type(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_typing_text
{
public:
  explicit Init_ConfigureMission_Request_typing_text(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_typing_location typing_text(::autonomy_interfaces::srv::ConfigureMission_Request::_typing_text_type arg)
  {
    msg_.typing_text = std::move(arg);
    return Init_ConfigureMission_Request_typing_location(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_waypoint_approach_tolerance
{
public:
  explicit Init_ConfigureMission_Request_waypoint_approach_tolerance(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_typing_text waypoint_approach_tolerance(::autonomy_interfaces::srv::ConfigureMission_Request::_waypoint_approach_tolerance_type arg)
  {
    msg_.waypoint_approach_tolerance = std::move(arg);
    return Init_ConfigureMission_Request_typing_text(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_max_angular_velocity
{
public:
  explicit Init_ConfigureMission_Request_max_angular_velocity(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_waypoint_approach_tolerance max_angular_velocity(::autonomy_interfaces::srv::ConfigureMission_Request::_max_angular_velocity_type arg)
  {
    msg_.max_angular_velocity = std::move(arg);
    return Init_ConfigureMission_Request_waypoint_approach_tolerance(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_max_linear_velocity
{
public:
  explicit Init_ConfigureMission_Request_max_linear_velocity(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_max_angular_velocity max_linear_velocity(::autonomy_interfaces::srv::ConfigureMission_Request::_max_linear_velocity_type arg)
  {
    msg_.max_linear_velocity = std::move(arg);
    return Init_ConfigureMission_Request_max_angular_velocity(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_waypoint_timeout
{
public:
  explicit Init_ConfigureMission_Request_waypoint_timeout(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_max_linear_velocity waypoint_timeout(::autonomy_interfaces::srv::ConfigureMission_Request::_waypoint_timeout_type arg)
  {
    msg_.waypoint_timeout = std::move(arg);
    return Init_ConfigureMission_Request_max_linear_velocity(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_time_limit
{
public:
  explicit Init_ConfigureMission_Request_time_limit(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_waypoint_timeout time_limit(::autonomy_interfaces::srv::ConfigureMission_Request::_time_limit_type arg)
  {
    msg_.time_limit = std::move(arg);
    return Init_ConfigureMission_Request_waypoint_timeout(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_waypoint_tolerances
{
public:
  explicit Init_ConfigureMission_Request_waypoint_tolerances(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_time_limit waypoint_tolerances(::autonomy_interfaces::srv::ConfigureMission_Request::_waypoint_tolerances_type arg)
  {
    msg_.waypoint_tolerances = std::move(arg);
    return Init_ConfigureMission_Request_time_limit(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_precision_required
{
public:
  explicit Init_ConfigureMission_Request_precision_required(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_waypoint_tolerances precision_required(::autonomy_interfaces::srv::ConfigureMission_Request::_precision_required_type arg)
  {
    msg_.precision_required = std::move(arg);
    return Init_ConfigureMission_Request_waypoint_tolerances(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_waypoint_names
{
public:
  explicit Init_ConfigureMission_Request_waypoint_names(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_precision_required waypoint_names(::autonomy_interfaces::srv::ConfigureMission_Request::_waypoint_names_type arg)
  {
    msg_.waypoint_names = std::move(arg);
    return Init_ConfigureMission_Request_precision_required(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_waypoints
{
public:
  explicit Init_ConfigureMission_Request_waypoints(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_waypoint_names waypoints(::autonomy_interfaces::srv::ConfigureMission_Request::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return Init_ConfigureMission_Request_waypoint_names(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_objectives
{
public:
  explicit Init_ConfigureMission_Request_objectives(::autonomy_interfaces::srv::ConfigureMission_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Request_waypoints objectives(::autonomy_interfaces::srv::ConfigureMission_Request::_objectives_type arg)
  {
    msg_.objectives = std::move(arg);
    return Init_ConfigureMission_Request_waypoints(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

class Init_ConfigureMission_Request_mission_name
{
public:
  Init_ConfigureMission_Request_mission_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfigureMission_Request_objectives mission_name(::autonomy_interfaces::srv::ConfigureMission_Request::_mission_name_type arg)
  {
    msg_.mission_name = std::move(arg);
    return Init_ConfigureMission_Request_objectives(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ConfigureMission_Request>()
{
  return autonomy_interfaces::srv::builder::Init_ConfigureMission_Request_mission_name();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConfigureMission_Response_configured_objectives
{
public:
  explicit Init_ConfigureMission_Response_configured_objectives(::autonomy_interfaces::srv::ConfigureMission_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ConfigureMission_Response configured_objectives(::autonomy_interfaces::srv::ConfigureMission_Response::_configured_objectives_type arg)
  {
    msg_.configured_objectives = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

class Init_ConfigureMission_Response_total_waypoints
{
public:
  explicit Init_ConfigureMission_Response_total_waypoints(::autonomy_interfaces::srv::ConfigureMission_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Response_configured_objectives total_waypoints(::autonomy_interfaces::srv::ConfigureMission_Response::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_ConfigureMission_Response_configured_objectives(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

class Init_ConfigureMission_Response_estimated_duration
{
public:
  explicit Init_ConfigureMission_Response_estimated_duration(::autonomy_interfaces::srv::ConfigureMission_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Response_total_waypoints estimated_duration(::autonomy_interfaces::srv::ConfigureMission_Response::_estimated_duration_type arg)
  {
    msg_.estimated_duration = std::move(arg);
    return Init_ConfigureMission_Response_total_waypoints(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

class Init_ConfigureMission_Response_mission_id
{
public:
  explicit Init_ConfigureMission_Response_mission_id(::autonomy_interfaces::srv::ConfigureMission_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Response_estimated_duration mission_id(::autonomy_interfaces::srv::ConfigureMission_Response::_mission_id_type arg)
  {
    msg_.mission_id = std::move(arg);
    return Init_ConfigureMission_Response_estimated_duration(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

class Init_ConfigureMission_Response_message
{
public:
  explicit Init_ConfigureMission_Response_message(::autonomy_interfaces::srv::ConfigureMission_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Response_mission_id message(::autonomy_interfaces::srv::ConfigureMission_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ConfigureMission_Response_mission_id(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

class Init_ConfigureMission_Response_success
{
public:
  Init_ConfigureMission_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfigureMission_Response_message success(::autonomy_interfaces::srv::ConfigureMission_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ConfigureMission_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ConfigureMission_Response>()
{
  return autonomy_interfaces::srv::builder::Init_ConfigureMission_Response_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConfigureMission_Event_response
{
public:
  explicit Init_ConfigureMission_Event_response(::autonomy_interfaces::srv::ConfigureMission_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::ConfigureMission_Event response(::autonomy_interfaces::srv::ConfigureMission_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Event msg_;
};

class Init_ConfigureMission_Event_request
{
public:
  explicit Init_ConfigureMission_Event_request(::autonomy_interfaces::srv::ConfigureMission_Event & msg)
  : msg_(msg)
  {}
  Init_ConfigureMission_Event_response request(::autonomy_interfaces::srv::ConfigureMission_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ConfigureMission_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Event msg_;
};

class Init_ConfigureMission_Event_info
{
public:
  Init_ConfigureMission_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfigureMission_Event_request info(::autonomy_interfaces::srv::ConfigureMission_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ConfigureMission_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::srv::ConfigureMission_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::ConfigureMission_Event>()
{
  return autonomy_interfaces::srv::builder::Init_ConfigureMission_Event_info();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__BUILDER_HPP_
