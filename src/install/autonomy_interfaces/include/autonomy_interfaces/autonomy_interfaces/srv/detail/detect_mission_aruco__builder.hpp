// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/detect_mission_aruco__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectMissionAruco_Request_min_alignment_quality
{
public:
  explicit Init_DetectMissionAruco_Request_min_alignment_quality(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::DetectMissionAruco_Request min_alignment_quality(::autonomy_interfaces::srv::DetectMissionAruco_Request::_min_alignment_quality_type arg)
  {
    msg_.min_alignment_quality = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_require_all_tags
{
public:
  explicit Init_DetectMissionAruco_Request_require_all_tags(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_min_alignment_quality require_all_tags(::autonomy_interfaces::srv::DetectMissionAruco_Request::_require_all_tags_type arg)
  {
    msg_.require_all_tags = std::move(arg);
    return Init_DetectMissionAruco_Request_min_alignment_quality(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_max_detection_distance
{
public:
  explicit Init_DetectMissionAruco_Request_max_detection_distance(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_require_all_tags max_detection_distance(::autonomy_interfaces::srv::DetectMissionAruco_Request::_max_detection_distance_type arg)
  {
    msg_.max_detection_distance = std::move(arg);
    return Init_DetectMissionAruco_Request_require_all_tags(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_target_depth
{
public:
  explicit Init_DetectMissionAruco_Request_target_depth(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_max_detection_distance target_depth(::autonomy_interfaces::srv::DetectMissionAruco_Request::_target_depth_type arg)
  {
    msg_.target_depth = std::move(arg);
    return Init_DetectMissionAruco_Request_max_detection_distance(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_detection_timeout
{
public:
  explicit Init_DetectMissionAruco_Request_detection_timeout(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_target_depth detection_timeout(::autonomy_interfaces::srv::DetectMissionAruco_Request::_detection_timeout_type arg)
  {
    msg_.detection_timeout = std::move(arg);
    return Init_DetectMissionAruco_Request_target_depth(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_optional_tag_ids
{
public:
  explicit Init_DetectMissionAruco_Request_optional_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_detection_timeout optional_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Request::_optional_tag_ids_type arg)
  {
    msg_.optional_tag_ids = std::move(arg);
    return Init_DetectMissionAruco_Request_detection_timeout(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_required_tag_ids
{
public:
  explicit Init_DetectMissionAruco_Request_required_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Request_optional_tag_ids required_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Request::_required_tag_ids_type arg)
  {
    msg_.required_tag_ids = std::move(arg);
    return Init_DetectMissionAruco_Request_optional_tag_ids(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

class Init_DetectMissionAruco_Request_mission_type
{
public:
  Init_DetectMissionAruco_Request_mission_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectMissionAruco_Request_required_tag_ids mission_type(::autonomy_interfaces::srv::DetectMissionAruco_Request::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return Init_DetectMissionAruco_Request_required_tag_ids(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::DetectMissionAruco_Request>()
{
  return autonomy_interfaces::srv::builder::Init_DetectMissionAruco_Request_mission_type();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectMissionAruco_Response_mission_recommendations
{
public:
  explicit Init_DetectMissionAruco_Response_mission_recommendations(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::DetectMissionAruco_Response mission_recommendations(::autonomy_interfaces::srv::DetectMissionAruco_Response::_mission_recommendations_type arg)
  {
    msg_.mission_recommendations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_warnings
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_warnings(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_mission_recommendations alignment_warnings(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_warnings_type arg)
  {
    msg_.alignment_warnings = std::move(arg);
    return Init_DetectMissionAruco_Response_mission_recommendations(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_detected_optional_tags
{
public:
  explicit Init_DetectMissionAruco_Response_detected_optional_tags(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_warnings detected_optional_tags(::autonomy_interfaces::srv::DetectMissionAruco_Response::_detected_optional_tags_type arg)
  {
    msg_.detected_optional_tags = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_warnings(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_missing_required_tags
{
public:
  explicit Init_DetectMissionAruco_Response_missing_required_tags(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_detected_optional_tags missing_required_tags(::autonomy_interfaces::srv::DetectMissionAruco_Response::_missing_required_tags_type arg)
  {
    msg_.missing_required_tags = std::move(arg);
    return Init_DetectMissionAruco_Response_detected_optional_tags(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_mission_ready
{
public:
  explicit Init_DetectMissionAruco_Response_mission_ready(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_missing_required_tags mission_ready(::autonomy_interfaces::srv::DetectMissionAruco_Response::_mission_ready_type arg)
  {
    msg_.mission_ready = std::move(arg);
    return Init_DetectMissionAruco_Response_missing_required_tags(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_errors
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_errors(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_mission_ready alignment_errors(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_errors_type arg)
  {
    msg_.alignment_errors = std::move(arg);
    return Init_DetectMissionAruco_Response_mission_ready(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_quality
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_quality(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_errors alignment_quality(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_quality_type arg)
  {
    msg_.alignment_quality = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_errors(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_arm_target_position
{
public:
  explicit Init_DetectMissionAruco_Response_arm_target_position(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_quality arm_target_position(::autonomy_interfaces::srv::DetectMissionAruco_Response::_arm_target_position_type arg)
  {
    msg_.arm_target_position = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_quality(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_orientation
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_orientation(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_arm_target_position alignment_orientation(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_orientation_type arg)
  {
    msg_.alignment_orientation = std::move(arg);
    return Init_DetectMissionAruco_Response_arm_target_position(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_center
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_center(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_orientation alignment_center(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_center_type arg)
  {
    msg_.alignment_center = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_orientation(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_alignment_available
{
public:
  explicit Init_DetectMissionAruco_Response_alignment_available(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_center alignment_available(::autonomy_interfaces::srv::DetectMissionAruco_Response::_alignment_available_type arg)
  {
    msg_.alignment_available = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_center(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_detection_time
{
public:
  explicit Init_DetectMissionAruco_Response_detection_time(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_alignment_available detection_time(::autonomy_interfaces::srv::DetectMissionAruco_Response::_detection_time_type arg)
  {
    msg_.detection_time = std::move(arg);
    return Init_DetectMissionAruco_Response_alignment_available(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_tag_angles
{
public:
  explicit Init_DetectMissionAruco_Response_tag_angles(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_detection_time tag_angles(::autonomy_interfaces::srv::DetectMissionAruco_Response::_tag_angles_type arg)
  {
    msg_.tag_angles = std::move(arg);
    return Init_DetectMissionAruco_Response_detection_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_tag_distances
{
public:
  explicit Init_DetectMissionAruco_Response_tag_distances(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_tag_angles tag_distances(::autonomy_interfaces::srv::DetectMissionAruco_Response::_tag_distances_type arg)
  {
    msg_.tag_distances = std::move(arg);
    return Init_DetectMissionAruco_Response_tag_angles(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_tag_positions
{
public:
  explicit Init_DetectMissionAruco_Response_tag_positions(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_tag_distances tag_positions(::autonomy_interfaces::srv::DetectMissionAruco_Response::_tag_positions_type arg)
  {
    msg_.tag_positions = std::move(arg);
    return Init_DetectMissionAruco_Response_tag_distances(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_detected_tag_ids
{
public:
  explicit Init_DetectMissionAruco_Response_detected_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_tag_positions detected_tag_ids(::autonomy_interfaces::srv::DetectMissionAruco_Response::_detected_tag_ids_type arg)
  {
    msg_.detected_tag_ids = std::move(arg);
    return Init_DetectMissionAruco_Response_tag_positions(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_mission_type
{
public:
  explicit Init_DetectMissionAruco_Response_mission_type(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_detected_tag_ids mission_type(::autonomy_interfaces::srv::DetectMissionAruco_Response::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return Init_DetectMissionAruco_Response_detected_tag_ids(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_message
{
public:
  explicit Init_DetectMissionAruco_Response_message(::autonomy_interfaces::srv::DetectMissionAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectMissionAruco_Response_mission_type message(::autonomy_interfaces::srv::DetectMissionAruco_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_DetectMissionAruco_Response_mission_type(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

class Init_DetectMissionAruco_Response_success
{
public:
  Init_DetectMissionAruco_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectMissionAruco_Response_message success(::autonomy_interfaces::srv::DetectMissionAruco_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DetectMissionAruco_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectMissionAruco_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::DetectMissionAruco_Response>()
{
  return autonomy_interfaces::srv::builder::Init_DetectMissionAruco_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__BUILDER_HPP_
