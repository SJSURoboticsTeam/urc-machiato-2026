// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/detect_aruco__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectAruco_Request_mission_type
{
public:
  explicit Init_DetectAruco_Request_mission_type(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::DetectAruco_Request mission_type(::autonomy_interfaces::srv::DetectAruco_Request::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_target_depth
{
public:
  explicit Init_DetectAruco_Request_target_depth(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Request_mission_type target_depth(::autonomy_interfaces::srv::DetectAruco_Request::_target_depth_type arg)
  {
    msg_.target_depth = std::move(arg);
    return Init_DetectAruco_Request_mission_type(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_calculate_alignment
{
public:
  explicit Init_DetectAruco_Request_calculate_alignment(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Request_target_depth calculate_alignment(::autonomy_interfaces::srv::DetectAruco_Request::_calculate_alignment_type arg)
  {
    msg_.calculate_alignment = std::move(arg);
    return Init_DetectAruco_Request_target_depth(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_max_detection_distance
{
public:
  explicit Init_DetectAruco_Request_max_detection_distance(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Request_calculate_alignment max_detection_distance(::autonomy_interfaces::srv::DetectAruco_Request::_max_detection_distance_type arg)
  {
    msg_.max_detection_distance = std::move(arg);
    return Init_DetectAruco_Request_calculate_alignment(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_require_distance_estimate
{
public:
  explicit Init_DetectAruco_Request_require_distance_estimate(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Request_max_detection_distance require_distance_estimate(::autonomy_interfaces::srv::DetectAruco_Request::_require_distance_estimate_type arg)
  {
    msg_.require_distance_estimate = std::move(arg);
    return Init_DetectAruco_Request_max_detection_distance(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_detection_timeout
{
public:
  explicit Init_DetectAruco_Request_detection_timeout(::autonomy_interfaces::srv::DetectAruco_Request & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Request_require_distance_estimate detection_timeout(::autonomy_interfaces::srv::DetectAruco_Request::_detection_timeout_type arg)
  {
    msg_.detection_timeout = std::move(arg);
    return Init_DetectAruco_Request_require_distance_estimate(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

class Init_DetectAruco_Request_target_tag_ids
{
public:
  Init_DetectAruco_Request_target_tag_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectAruco_Request_detection_timeout target_tag_ids(::autonomy_interfaces::srv::DetectAruco_Request::_target_tag_ids_type arg)
  {
    msg_.target_tag_ids = std::move(arg);
    return Init_DetectAruco_Request_detection_timeout(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::DetectAruco_Request>()
{
  return autonomy_interfaces::srv::builder::Init_DetectAruco_Request_target_tag_ids();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectAruco_Response_alignment_warnings
{
public:
  explicit Init_DetectAruco_Response_alignment_warnings(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::srv::DetectAruco_Response alignment_warnings(::autonomy_interfaces::srv::DetectAruco_Response::_alignment_warnings_type arg)
  {
    msg_.alignment_warnings = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_alignment_quality
{
public:
  explicit Init_DetectAruco_Response_alignment_quality(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_alignment_warnings alignment_quality(::autonomy_interfaces::srv::DetectAruco_Response::_alignment_quality_type arg)
  {
    msg_.alignment_quality = std::move(arg);
    return Init_DetectAruco_Response_alignment_warnings(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_arm_target_position
{
public:
  explicit Init_DetectAruco_Response_arm_target_position(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_alignment_quality arm_target_position(::autonomy_interfaces::srv::DetectAruco_Response::_arm_target_position_type arg)
  {
    msg_.arm_target_position = std::move(arg);
    return Init_DetectAruco_Response_alignment_quality(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_alignment_orientation
{
public:
  explicit Init_DetectAruco_Response_alignment_orientation(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_arm_target_position alignment_orientation(::autonomy_interfaces::srv::DetectAruco_Response::_alignment_orientation_type arg)
  {
    msg_.alignment_orientation = std::move(arg);
    return Init_DetectAruco_Response_arm_target_position(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_alignment_center
{
public:
  explicit Init_DetectAruco_Response_alignment_center(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_alignment_orientation alignment_center(::autonomy_interfaces::srv::DetectAruco_Response::_alignment_center_type arg)
  {
    msg_.alignment_center = std::move(arg);
    return Init_DetectAruco_Response_alignment_orientation(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_alignment_available
{
public:
  explicit Init_DetectAruco_Response_alignment_available(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_alignment_center alignment_available(::autonomy_interfaces::srv::DetectAruco_Response::_alignment_available_type arg)
  {
    msg_.alignment_available = std::move(arg);
    return Init_DetectAruco_Response_alignment_center(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_detection_time
{
public:
  explicit Init_DetectAruco_Response_detection_time(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_alignment_available detection_time(::autonomy_interfaces::srv::DetectAruco_Response::_detection_time_type arg)
  {
    msg_.detection_time = std::move(arg);
    return Init_DetectAruco_Response_alignment_available(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_tag_angles
{
public:
  explicit Init_DetectAruco_Response_tag_angles(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_detection_time tag_angles(::autonomy_interfaces::srv::DetectAruco_Response::_tag_angles_type arg)
  {
    msg_.tag_angles = std::move(arg);
    return Init_DetectAruco_Response_detection_time(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_tag_distances
{
public:
  explicit Init_DetectAruco_Response_tag_distances(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_tag_angles tag_distances(::autonomy_interfaces::srv::DetectAruco_Response::_tag_distances_type arg)
  {
    msg_.tag_distances = std::move(arg);
    return Init_DetectAruco_Response_tag_angles(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_tag_positions
{
public:
  explicit Init_DetectAruco_Response_tag_positions(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_tag_distances tag_positions(::autonomy_interfaces::srv::DetectAruco_Response::_tag_positions_type arg)
  {
    msg_.tag_positions = std::move(arg);
    return Init_DetectAruco_Response_tag_distances(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_detected_tag_ids
{
public:
  explicit Init_DetectAruco_Response_detected_tag_ids(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_tag_positions detected_tag_ids(::autonomy_interfaces::srv::DetectAruco_Response::_detected_tag_ids_type arg)
  {
    msg_.detected_tag_ids = std::move(arg);
    return Init_DetectAruco_Response_tag_positions(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_message
{
public:
  explicit Init_DetectAruco_Response_message(::autonomy_interfaces::srv::DetectAruco_Response & msg)
  : msg_(msg)
  {}
  Init_DetectAruco_Response_detected_tag_ids message(::autonomy_interfaces::srv::DetectAruco_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_DetectAruco_Response_detected_tag_ids(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

class Init_DetectAruco_Response_success
{
public:
  Init_DetectAruco_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectAruco_Response_message success(::autonomy_interfaces::srv::DetectAruco_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DetectAruco_Response_message(msg_);
  }

private:
  ::autonomy_interfaces::srv::DetectAruco_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::DetectAruco_Response>()
{
  return autonomy_interfaces::srv::builder::Init_DetectAruco_Response_success();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__BUILDER_HPP_
