// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_HPP_

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
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__CameraCommand __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__CameraCommand __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraCommand_
{
  using Type = CameraCommand_<ContainerAllocator>;

  explicit CameraCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    target_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = 0;
      this->pan_angle = 0.0f;
      this->tilt_angle = 0.0f;
      this->pan_speed = 0.0f;
      this->tilt_speed = 0.0f;
      this->zoom_level = 0.0f;
      this->autofocus = false;
      this->tracking_timeout = 0.0f;
      this->scan_pattern = "";
      this->scan_speed = 0.0f;
      this->scan_range = 0.0f;
      this->max_pan_speed = 0.0f;
      this->max_tilt_speed = 0.0f;
      this->priority = 0;
      this->timeout = 0.0f;
    }
  }

  explicit CameraCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    target_position(_alloc, _init),
    scan_pattern(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = 0;
      this->pan_angle = 0.0f;
      this->tilt_angle = 0.0f;
      this->pan_speed = 0.0f;
      this->tilt_speed = 0.0f;
      this->zoom_level = 0.0f;
      this->autofocus = false;
      this->tracking_timeout = 0.0f;
      this->scan_pattern = "";
      this->scan_speed = 0.0f;
      this->scan_range = 0.0f;
      this->max_pan_speed = 0.0f;
      this->max_tilt_speed = 0.0f;
      this->priority = 0;
      this->timeout = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _command_type_type =
    uint8_t;
  _command_type_type command_type;
  using _pan_angle_type =
    float;
  _pan_angle_type pan_angle;
  using _tilt_angle_type =
    float;
  _tilt_angle_type tilt_angle;
  using _pan_speed_type =
    float;
  _pan_speed_type pan_speed;
  using _tilt_speed_type =
    float;
  _tilt_speed_type tilt_speed;
  using _zoom_level_type =
    float;
  _zoom_level_type zoom_level;
  using _autofocus_type =
    bool;
  _autofocus_type autofocus;
  using _target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_position_type target_position;
  using _tracking_timeout_type =
    float;
  _tracking_timeout_type tracking_timeout;
  using _scan_pattern_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scan_pattern_type scan_pattern;
  using _scan_speed_type =
    float;
  _scan_speed_type scan_speed;
  using _scan_range_type =
    float;
  _scan_range_type scan_range;
  using _max_pan_speed_type =
    float;
  _max_pan_speed_type max_pan_speed;
  using _max_tilt_speed_type =
    float;
  _max_tilt_speed_type max_tilt_speed;
  using _priority_type =
    uint8_t;
  _priority_type priority;
  using _timeout_type =
    float;
  _timeout_type timeout;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__command_type(
    const uint8_t & _arg)
  {
    this->command_type = _arg;
    return *this;
  }
  Type & set__pan_angle(
    const float & _arg)
  {
    this->pan_angle = _arg;
    return *this;
  }
  Type & set__tilt_angle(
    const float & _arg)
  {
    this->tilt_angle = _arg;
    return *this;
  }
  Type & set__pan_speed(
    const float & _arg)
  {
    this->pan_speed = _arg;
    return *this;
  }
  Type & set__tilt_speed(
    const float & _arg)
  {
    this->tilt_speed = _arg;
    return *this;
  }
  Type & set__zoom_level(
    const float & _arg)
  {
    this->zoom_level = _arg;
    return *this;
  }
  Type & set__autofocus(
    const bool & _arg)
  {
    this->autofocus = _arg;
    return *this;
  }
  Type & set__target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__tracking_timeout(
    const float & _arg)
  {
    this->tracking_timeout = _arg;
    return *this;
  }
  Type & set__scan_pattern(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scan_pattern = _arg;
    return *this;
  }
  Type & set__scan_speed(
    const float & _arg)
  {
    this->scan_speed = _arg;
    return *this;
  }
  Type & set__scan_range(
    const float & _arg)
  {
    this->scan_range = _arg;
    return *this;
  }
  Type & set__max_pan_speed(
    const float & _arg)
  {
    this->max_pan_speed = _arg;
    return *this;
  }
  Type & set__max_tilt_speed(
    const float & _arg)
  {
    this->max_tilt_speed = _arg;
    return *this;
  }
  Type & set__priority(
    const uint8_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__CameraCommand
    std::shared_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__CameraCommand
    std::shared_ptr<autonomy_interfaces::msg::CameraCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->command_type != other.command_type) {
      return false;
    }
    if (this->pan_angle != other.pan_angle) {
      return false;
    }
    if (this->tilt_angle != other.tilt_angle) {
      return false;
    }
    if (this->pan_speed != other.pan_speed) {
      return false;
    }
    if (this->tilt_speed != other.tilt_speed) {
      return false;
    }
    if (this->zoom_level != other.zoom_level) {
      return false;
    }
    if (this->autofocus != other.autofocus) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->tracking_timeout != other.tracking_timeout) {
      return false;
    }
    if (this->scan_pattern != other.scan_pattern) {
      return false;
    }
    if (this->scan_speed != other.scan_speed) {
      return false;
    }
    if (this->scan_range != other.scan_range) {
      return false;
    }
    if (this->max_pan_speed != other.max_pan_speed) {
      return false;
    }
    if (this->max_tilt_speed != other.max_tilt_speed) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraCommand_

// alias to use template instance with default allocator
using CameraCommand =
  autonomy_interfaces::msg::CameraCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__STRUCT_HPP_
