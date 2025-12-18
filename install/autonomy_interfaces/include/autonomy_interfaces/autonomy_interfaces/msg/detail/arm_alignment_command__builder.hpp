// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/arm_alignment_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_ArmAlignmentCommand_allow_realignment
{
public:
  explicit Init_ArmAlignmentCommand_allow_realignment(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::ArmAlignmentCommand allow_realignment(::autonomy_interfaces::msg::ArmAlignmentCommand::_allow_realignment_type arg)
  {
    msg_.allow_realignment = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_tag_visibility_timeout
{
public:
  explicit Init_ArmAlignmentCommand_tag_visibility_timeout(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_allow_realignment tag_visibility_timeout(::autonomy_interfaces::msg::ArmAlignmentCommand::_tag_visibility_timeout_type arg)
  {
    msg_.tag_visibility_timeout = std::move(arg);
    return Init_ArmAlignmentCommand_allow_realignment(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_required_aruco_tags
{
public:
  explicit Init_ArmAlignmentCommand_required_aruco_tags(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_tag_visibility_timeout required_aruco_tags(::autonomy_interfaces::msg::ArmAlignmentCommand::_required_aruco_tags_type arg)
  {
    msg_.required_aruco_tags = std::move(arg);
    return Init_ArmAlignmentCommand_tag_visibility_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_feedback_rate
{
public:
  explicit Init_ArmAlignmentCommand_feedback_rate(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_required_aruco_tags feedback_rate(::autonomy_interfaces::msg::ArmAlignmentCommand::_feedback_rate_type arg)
  {
    msg_.feedback_rate = std::move(arg);
    return Init_ArmAlignmentCommand_required_aruco_tags(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_require_force_feedback
{
public:
  explicit Init_ArmAlignmentCommand_require_force_feedback(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_feedback_rate require_force_feedback(::autonomy_interfaces::msg::ArmAlignmentCommand::_require_force_feedback_type arg)
  {
    msg_.require_force_feedback = std::move(arg);
    return Init_ArmAlignmentCommand_feedback_rate(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_require_position_feedback
{
public:
  explicit Init_ArmAlignmentCommand_require_position_feedback(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_require_force_feedback require_position_feedback(::autonomy_interfaces::msg::ArmAlignmentCommand::_require_position_feedback_type arg)
  {
    msg_.require_position_feedback = std::move(arg);
    return Init_ArmAlignmentCommand_require_force_feedback(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_safety_zones
{
public:
  explicit Init_ArmAlignmentCommand_safety_zones(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_require_position_feedback safety_zones(::autonomy_interfaces::msg::ArmAlignmentCommand::_safety_zones_type arg)
  {
    msg_.safety_zones = std::move(arg);
    return Init_ArmAlignmentCommand_require_position_feedback(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_enable_collision_avoidance
{
public:
  explicit Init_ArmAlignmentCommand_enable_collision_avoidance(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_safety_zones enable_collision_avoidance(::autonomy_interfaces::msg::ArmAlignmentCommand::_enable_collision_avoidance_type arg)
  {
    msg_.enable_collision_avoidance = std::move(arg);
    return Init_ArmAlignmentCommand_safety_zones(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_max_rotation_speed
{
public:
  explicit Init_ArmAlignmentCommand_max_rotation_speed(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_enable_collision_avoidance max_rotation_speed(::autonomy_interfaces::msg::ArmAlignmentCommand::_max_rotation_speed_type arg)
  {
    msg_.max_rotation_speed = std::move(arg);
    return Init_ArmAlignmentCommand_enable_collision_avoidance(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_max_approach_speed
{
public:
  explicit Init_ArmAlignmentCommand_max_approach_speed(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_max_rotation_speed max_approach_speed(::autonomy_interfaces::msg::ArmAlignmentCommand::_max_approach_speed_type arg)
  {
    msg_.max_approach_speed = std::move(arg);
    return Init_ArmAlignmentCommand_max_rotation_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_alignment_timeout
{
public:
  explicit Init_ArmAlignmentCommand_alignment_timeout(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_max_approach_speed alignment_timeout(::autonomy_interfaces::msg::ArmAlignmentCommand::_alignment_timeout_type arg)
  {
    msg_.alignment_timeout = std::move(arg);
    return Init_ArmAlignmentCommand_max_approach_speed(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_max_orientation_error
{
public:
  explicit Init_ArmAlignmentCommand_max_orientation_error(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_alignment_timeout max_orientation_error(::autonomy_interfaces::msg::ArmAlignmentCommand::_max_orientation_error_type arg)
  {
    msg_.max_orientation_error = std::move(arg);
    return Init_ArmAlignmentCommand_alignment_timeout(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_max_position_error
{
public:
  explicit Init_ArmAlignmentCommand_max_position_error(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_max_orientation_error max_position_error(::autonomy_interfaces::msg::ArmAlignmentCommand::_max_position_error_type arg)
  {
    msg_.max_position_error = std::move(arg);
    return Init_ArmAlignmentCommand_max_orientation_error(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_alignment_quality
{
public:
  explicit Init_ArmAlignmentCommand_alignment_quality(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_max_position_error alignment_quality(::autonomy_interfaces::msg::ArmAlignmentCommand::_alignment_quality_type arg)
  {
    msg_.alignment_quality = std::move(arg);
    return Init_ArmAlignmentCommand_max_position_error(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_final_distance
{
public:
  explicit Init_ArmAlignmentCommand_final_distance(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_alignment_quality final_distance(::autonomy_interfaces::msg::ArmAlignmentCommand::_final_distance_type arg)
  {
    msg_.final_distance = std::move(arg);
    return Init_ArmAlignmentCommand_alignment_quality(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_approach_distance
{
public:
  explicit Init_ArmAlignmentCommand_approach_distance(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_final_distance approach_distance(::autonomy_interfaces::msg::ArmAlignmentCommand::_approach_distance_type arg)
  {
    msg_.approach_distance = std::move(arg);
    return Init_ArmAlignmentCommand_final_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_target_orientation
{
public:
  explicit Init_ArmAlignmentCommand_target_orientation(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_approach_distance target_orientation(::autonomy_interfaces::msg::ArmAlignmentCommand::_target_orientation_type arg)
  {
    msg_.target_orientation = std::move(arg);
    return Init_ArmAlignmentCommand_approach_distance(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_target_position
{
public:
  explicit Init_ArmAlignmentCommand_target_position(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_target_orientation target_position(::autonomy_interfaces::msg::ArmAlignmentCommand::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_ArmAlignmentCommand_target_orientation(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_alignment_id
{
public:
  explicit Init_ArmAlignmentCommand_alignment_id(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_target_position alignment_id(::autonomy_interfaces::msg::ArmAlignmentCommand::_alignment_id_type arg)
  {
    msg_.alignment_id = std::move(arg);
    return Init_ArmAlignmentCommand_target_position(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_mission_type
{
public:
  explicit Init_ArmAlignmentCommand_mission_type(::autonomy_interfaces::msg::ArmAlignmentCommand & msg)
  : msg_(msg)
  {}
  Init_ArmAlignmentCommand_alignment_id mission_type(::autonomy_interfaces::msg::ArmAlignmentCommand::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return Init_ArmAlignmentCommand_alignment_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

class Init_ArmAlignmentCommand_header
{
public:
  Init_ArmAlignmentCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmAlignmentCommand_mission_type header(::autonomy_interfaces::msg::ArmAlignmentCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArmAlignmentCommand_mission_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::ArmAlignmentCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::ArmAlignmentCommand>()
{
  return autonomy_interfaces::msg::builder::Init_ArmAlignmentCommand_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__BUILDER_HPP_
