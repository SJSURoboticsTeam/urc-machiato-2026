// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/vision_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_VisionDetection_age
{
public:
  explicit Init_VisionDetection_age(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::VisionDetection age(::autonomy_interfaces::msg::VisionDetection::_age_type arg)
  {
    msg_.age = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_track_id
{
public:
  explicit Init_VisionDetection_track_id(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_age track_id(::autonomy_interfaces::msg::VisionDetection::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_VisionDetection_age(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_detector_type
{
public:
  explicit Init_VisionDetection_detector_type(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_track_id detector_type(::autonomy_interfaces::msg::VisionDetection::_detector_type_type arg)
  {
    msg_.detector_type = std::move(arg);
    return Init_VisionDetection_track_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_keypoints
{
public:
  explicit Init_VisionDetection_keypoints(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_detector_type keypoints(::autonomy_interfaces::msg::VisionDetection::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return Init_VisionDetection_detector_type(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_size
{
public:
  explicit Init_VisionDetection_size(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_keypoints size(::autonomy_interfaces::msg::VisionDetection::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_VisionDetection_keypoints(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_pose
{
public:
  explicit Init_VisionDetection_pose(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_size pose(::autonomy_interfaces::msg::VisionDetection::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_VisionDetection_size(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_confidence
{
public:
  explicit Init_VisionDetection_confidence(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_pose confidence(::autonomy_interfaces::msg::VisionDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_VisionDetection_pose(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_class_id
{
public:
  explicit Init_VisionDetection_class_id(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_confidence class_id(::autonomy_interfaces::msg::VisionDetection::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_VisionDetection_confidence(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_class_name
{
public:
  explicit Init_VisionDetection_class_name(::autonomy_interfaces::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_class_id class_name(::autonomy_interfaces::msg::VisionDetection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_VisionDetection_class_id(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

class Init_VisionDetection_header
{
public:
  Init_VisionDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VisionDetection_class_name header(::autonomy_interfaces::msg::VisionDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VisionDetection_class_name(msg_);
  }

private:
  ::autonomy_interfaces::msg::VisionDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::VisionDetection>()
{
  return autonomy_interfaces::msg::builder::Init_VisionDetection_header();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_
