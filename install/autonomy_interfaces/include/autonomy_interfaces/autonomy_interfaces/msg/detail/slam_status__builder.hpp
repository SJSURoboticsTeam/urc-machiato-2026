// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/SlamStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/slam_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/slam_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_SlamStatus_drift_estimate
{
public:
  explicit Init_SlamStatus_drift_estimate(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::SlamStatus drift_estimate(::autonomy_interfaces::msg::SlamStatus::_drift_estimate_type arg)
  {
    msg_.drift_estimate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_loop_closure_detected
{
public:
  explicit Init_SlamStatus_loop_closure_detected(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_drift_estimate loop_closure_detected(::autonomy_interfaces::msg::SlamStatus::_loop_closure_detected_type arg)
  {
    msg_.loop_closure_detected = std::move(arg);
    return Init_SlamStatus_drift_estimate(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_tracking_quality
{
public:
  explicit Init_SlamStatus_tracking_quality(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_loop_closure_detected tracking_quality(::autonomy_interfaces::msg::SlamStatus::_tracking_quality_type arg)
  {
    msg_.tracking_quality = std::move(arg);
    return Init_SlamStatus_loop_closure_detected(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_landmarks_tracked
{
public:
  explicit Init_SlamStatus_landmarks_tracked(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_tracking_quality landmarks_tracked(::autonomy_interfaces::msg::SlamStatus::_landmarks_tracked_type arg)
  {
    msg_.landmarks_tracked = std::move(arg);
    return Init_SlamStatus_tracking_quality(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_keyframes_tracked
{
public:
  explicit Init_SlamStatus_keyframes_tracked(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_landmarks_tracked keyframes_tracked(::autonomy_interfaces::msg::SlamStatus::_keyframes_tracked_type arg)
  {
    msg_.keyframes_tracked = std::move(arg);
    return Init_SlamStatus_landmarks_tracked(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_loop_closure_confidence
{
public:
  explicit Init_SlamStatus_loop_closure_confidence(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_keyframes_tracked loop_closure_confidence(::autonomy_interfaces::msg::SlamStatus::_loop_closure_confidence_type arg)
  {
    msg_.loop_closure_confidence = std::move(arg);
    return Init_SlamStatus_keyframes_tracked(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_map_resolution
{
public:
  explicit Init_SlamStatus_map_resolution(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_loop_closure_confidence map_resolution(::autonomy_interfaces::msg::SlamStatus::_map_resolution_type arg)
  {
    msg_.map_resolution = std::move(arg);
    return Init_SlamStatus_loop_closure_confidence(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_map_height
{
public:
  explicit Init_SlamStatus_map_height(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_map_resolution map_height(::autonomy_interfaces::msg::SlamStatus::_map_height_type arg)
  {
    msg_.map_height = std::move(arg);
    return Init_SlamStatus_map_resolution(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_map_width
{
public:
  explicit Init_SlamStatus_map_width(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_map_height map_width(::autonomy_interfaces::msg::SlamStatus::_map_width_type arg)
  {
    msg_.map_width = std::move(arg);
    return Init_SlamStatus_map_height(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_local_map
{
public:
  explicit Init_SlamStatus_local_map(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_map_width local_map(::autonomy_interfaces::msg::SlamStatus::_local_map_type arg)
  {
    msg_.local_map = std::move(arg);
    return Init_SlamStatus_map_width(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_pose
{
public:
  explicit Init_SlamStatus_pose(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_local_map pose(::autonomy_interfaces::msg::SlamStatus::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_SlamStatus_local_map(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_state
{
public:
  explicit Init_SlamStatus_state(::autonomy_interfaces::msg::SlamStatus & msg)
  : msg_(msg)
  {}
  Init_SlamStatus_pose state(::autonomy_interfaces::msg::SlamStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_SlamStatus_pose(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

class Init_SlamStatus_header
{
public:
  Init_SlamStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SlamStatus_state header(::autonomy_interfaces::msg::SlamStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SlamStatus_state(msg_);
  }

private:
  ::autonomy_interfaces::msg::SlamStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::SlamStatus>()
{
  return autonomy_interfaces::msg::builder::Init_SlamStatus_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__BUILDER_HPP_
