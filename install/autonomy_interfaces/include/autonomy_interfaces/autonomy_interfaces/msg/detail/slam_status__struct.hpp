// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/SlamStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/slam_status.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp"
// Member 'local_map'
#include "nav_msgs/msg/detail/occupancy_grid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__SlamStatus __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__SlamStatus __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SlamStatus_
{
  using Type = SlamStatus_<ContainerAllocator>;

  explicit SlamStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    local_map(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->map_width = 0l;
      this->map_height = 0l;
      this->map_resolution = 0.0f;
      this->loop_closure_confidence = 0.0f;
      this->keyframes_tracked = 0l;
      this->landmarks_tracked = 0l;
      this->tracking_quality = 0.0f;
      this->loop_closure_detected = false;
      this->drift_estimate = 0.0f;
    }
  }

  explicit SlamStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc),
    pose(_alloc, _init),
    local_map(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->map_width = 0l;
      this->map_height = 0l;
      this->map_resolution = 0.0f;
      this->loop_closure_confidence = 0.0f;
      this->keyframes_tracked = 0l;
      this->landmarks_tracked = 0l;
      this->tracking_quality = 0.0f;
      this->loop_closure_detected = false;
      this->drift_estimate = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>;
  _pose_type pose;
  using _local_map_type =
    nav_msgs::msg::OccupancyGrid_<ContainerAllocator>;
  _local_map_type local_map;
  using _map_width_type =
    int32_t;
  _map_width_type map_width;
  using _map_height_type =
    int32_t;
  _map_height_type map_height;
  using _map_resolution_type =
    float;
  _map_resolution_type map_resolution;
  using _loop_closure_confidence_type =
    float;
  _loop_closure_confidence_type loop_closure_confidence;
  using _keyframes_tracked_type =
    int32_t;
  _keyframes_tracked_type keyframes_tracked;
  using _landmarks_tracked_type =
    int32_t;
  _landmarks_tracked_type landmarks_tracked;
  using _tracking_quality_type =
    float;
  _tracking_quality_type tracking_quality;
  using _loop_closure_detected_type =
    bool;
  _loop_closure_detected_type loop_closure_detected;
  using _drift_estimate_type =
    float;
  _drift_estimate_type drift_estimate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__local_map(
    const nav_msgs::msg::OccupancyGrid_<ContainerAllocator> & _arg)
  {
    this->local_map = _arg;
    return *this;
  }
  Type & set__map_width(
    const int32_t & _arg)
  {
    this->map_width = _arg;
    return *this;
  }
  Type & set__map_height(
    const int32_t & _arg)
  {
    this->map_height = _arg;
    return *this;
  }
  Type & set__map_resolution(
    const float & _arg)
  {
    this->map_resolution = _arg;
    return *this;
  }
  Type & set__loop_closure_confidence(
    const float & _arg)
  {
    this->loop_closure_confidence = _arg;
    return *this;
  }
  Type & set__keyframes_tracked(
    const int32_t & _arg)
  {
    this->keyframes_tracked = _arg;
    return *this;
  }
  Type & set__landmarks_tracked(
    const int32_t & _arg)
  {
    this->landmarks_tracked = _arg;
    return *this;
  }
  Type & set__tracking_quality(
    const float & _arg)
  {
    this->tracking_quality = _arg;
    return *this;
  }
  Type & set__loop_closure_detected(
    const bool & _arg)
  {
    this->loop_closure_detected = _arg;
    return *this;
  }
  Type & set__drift_estimate(
    const float & _arg)
  {
    this->drift_estimate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__SlamStatus
    std::shared_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__SlamStatus
    std::shared_ptr<autonomy_interfaces::msg::SlamStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SlamStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->local_map != other.local_map) {
      return false;
    }
    if (this->map_width != other.map_width) {
      return false;
    }
    if (this->map_height != other.map_height) {
      return false;
    }
    if (this->map_resolution != other.map_resolution) {
      return false;
    }
    if (this->loop_closure_confidence != other.loop_closure_confidence) {
      return false;
    }
    if (this->keyframes_tracked != other.keyframes_tracked) {
      return false;
    }
    if (this->landmarks_tracked != other.landmarks_tracked) {
      return false;
    }
    if (this->tracking_quality != other.tracking_quality) {
      return false;
    }
    if (this->loop_closure_detected != other.loop_closure_detected) {
      return false;
    }
    if (this->drift_estimate != other.drift_estimate) {
      return false;
    }
    return true;
  }
  bool operator!=(const SlamStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SlamStatus_

// alias to use template instance with default allocator
using SlamStatus =
  autonomy_interfaces::msg::SlamStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SLAM_STATUS__STRUCT_HPP_
