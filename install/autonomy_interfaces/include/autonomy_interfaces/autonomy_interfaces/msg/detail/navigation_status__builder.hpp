// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/NavigationStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/navigation_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_NavigationStatus_status_message
{
public:
  explicit Init_NavigationStatus_status_message(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::NavigationStatus status_message(::autonomy_interfaces::msg::NavigationStatus::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_heading_error
{
public:
  explicit Init_NavigationStatus_heading_error(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_status_message heading_error(::autonomy_interfaces::msg::NavigationStatus::_heading_error_type arg)
  {
    msg_.heading_error = std::move(arg);
    return Init_NavigationStatus_status_message(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_speed
{
public:
  explicit Init_NavigationStatus_speed(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_heading_error speed(::autonomy_interfaces::msg::NavigationStatus::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_NavigationStatus_heading_error(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_distance_to_goal
{
public:
  explicit Init_NavigationStatus_distance_to_goal(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_speed distance_to_goal(::autonomy_interfaces::msg::NavigationStatus::_distance_to_goal_type arg)
  {
    msg_.distance_to_goal = std::move(arg);
    return Init_NavigationStatus_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_goal_pose
{
public:
  explicit Init_NavigationStatus_goal_pose(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_distance_to_goal goal_pose(::autonomy_interfaces::msg::NavigationStatus::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return Init_NavigationStatus_distance_to_goal(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_current_pose
{
public:
  explicit Init_NavigationStatus_current_pose(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_goal_pose current_pose(::autonomy_interfaces::msg::NavigationStatus::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_NavigationStatus_goal_pose(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_total_waypoints
{
public:
  explicit Init_NavigationStatus_total_waypoints(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_current_pose total_waypoints(::autonomy_interfaces::msg::NavigationStatus::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_NavigationStatus_current_pose(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_current_waypoint
{
public:
  explicit Init_NavigationStatus_current_waypoint(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_total_waypoints current_waypoint(::autonomy_interfaces::msg::NavigationStatus::_current_waypoint_type arg)
  {
    msg_.current_waypoint = std::move(arg);
    return Init_NavigationStatus_total_waypoints(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_mission_progress
{
public:
  explicit Init_NavigationStatus_mission_progress(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_current_waypoint mission_progress(::autonomy_interfaces::msg::NavigationStatus::_mission_progress_type arg)
  {
    msg_.mission_progress = std::move(arg);
    return Init_NavigationStatus_current_waypoint(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_state
{
public:
  explicit Init_NavigationStatus_state(::autonomy_interfaces::msg::NavigationStatus & msg)
  : msg_(msg)
  {}
  Init_NavigationStatus_mission_progress state(::autonomy_interfaces::msg::NavigationStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_NavigationStatus_mission_progress(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

class Init_NavigationStatus_header
{
public:
  Init_NavigationStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationStatus_state header(::autonomy_interfaces::msg::NavigationStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavigationStatus_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::NavigationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::NavigationStatus>()
{
  return autonomy_interfaces::msg::builder::Init_NavigationStatus_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__NAVIGATION_STATUS__BUILDER_HPP_
