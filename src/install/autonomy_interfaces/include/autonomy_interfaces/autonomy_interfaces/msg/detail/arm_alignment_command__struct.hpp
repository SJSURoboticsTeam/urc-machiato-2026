// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_HPP_

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
// Member 'target_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__ArmAlignmentCommand __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__ArmAlignmentCommand __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmAlignmentCommand_
{
  using Type = ArmAlignmentCommand_<ContainerAllocator>;

  explicit ArmAlignmentCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    target_position(_init),
    target_orientation(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_type = "";
      this->alignment_id = "";
      this->approach_distance = 0.0f;
      this->final_distance = 0.0f;
      this->alignment_quality = 0.0f;
      this->max_position_error = 0.0f;
      this->max_orientation_error = 0.0f;
      this->alignment_timeout = 0.0f;
      this->max_approach_speed = 0.0f;
      this->max_rotation_speed = 0.0f;
      this->enable_collision_avoidance = false;
      this->require_position_feedback = false;
      this->require_force_feedback = false;
      this->feedback_rate = 0.0f;
      this->tag_visibility_timeout = 0.0f;
      this->allow_realignment = false;
    }
  }

  explicit ArmAlignmentCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    mission_type(_alloc),
    alignment_id(_alloc),
    target_position(_alloc, _init),
    target_orientation(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_type = "";
      this->alignment_id = "";
      this->approach_distance = 0.0f;
      this->final_distance = 0.0f;
      this->alignment_quality = 0.0f;
      this->max_position_error = 0.0f;
      this->max_orientation_error = 0.0f;
      this->alignment_timeout = 0.0f;
      this->max_approach_speed = 0.0f;
      this->max_rotation_speed = 0.0f;
      this->enable_collision_avoidance = false;
      this->require_position_feedback = false;
      this->require_force_feedback = false;
      this->feedback_rate = 0.0f;
      this->tag_visibility_timeout = 0.0f;
      this->allow_realignment = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mission_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type_type mission_type;
  using _alignment_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _alignment_id_type alignment_id;
  using _target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_position_type target_position;
  using _target_orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _target_orientation_type target_orientation;
  using _approach_distance_type =
    float;
  _approach_distance_type approach_distance;
  using _final_distance_type =
    float;
  _final_distance_type final_distance;
  using _alignment_quality_type =
    float;
  _alignment_quality_type alignment_quality;
  using _max_position_error_type =
    float;
  _max_position_error_type max_position_error;
  using _max_orientation_error_type =
    float;
  _max_orientation_error_type max_orientation_error;
  using _alignment_timeout_type =
    float;
  _alignment_timeout_type alignment_timeout;
  using _max_approach_speed_type =
    float;
  _max_approach_speed_type max_approach_speed;
  using _max_rotation_speed_type =
    float;
  _max_rotation_speed_type max_rotation_speed;
  using _enable_collision_avoidance_type =
    bool;
  _enable_collision_avoidance_type enable_collision_avoidance;
  using _safety_zones_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _safety_zones_type safety_zones;
  using _require_position_feedback_type =
    bool;
  _require_position_feedback_type require_position_feedback;
  using _require_force_feedback_type =
    bool;
  _require_force_feedback_type require_force_feedback;
  using _feedback_rate_type =
    float;
  _feedback_rate_type feedback_rate;
  using _required_aruco_tags_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _required_aruco_tags_type required_aruco_tags;
  using _tag_visibility_timeout_type =
    float;
  _tag_visibility_timeout_type tag_visibility_timeout;
  using _allow_realignment_type =
    bool;
  _allow_realignment_type allow_realignment;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mission_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_type = _arg;
    return *this;
  }
  Type & set__alignment_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->alignment_id = _arg;
    return *this;
  }
  Type & set__target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__target_orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->target_orientation = _arg;
    return *this;
  }
  Type & set__approach_distance(
    const float & _arg)
  {
    this->approach_distance = _arg;
    return *this;
  }
  Type & set__final_distance(
    const float & _arg)
  {
    this->final_distance = _arg;
    return *this;
  }
  Type & set__alignment_quality(
    const float & _arg)
  {
    this->alignment_quality = _arg;
    return *this;
  }
  Type & set__max_position_error(
    const float & _arg)
  {
    this->max_position_error = _arg;
    return *this;
  }
  Type & set__max_orientation_error(
    const float & _arg)
  {
    this->max_orientation_error = _arg;
    return *this;
  }
  Type & set__alignment_timeout(
    const float & _arg)
  {
    this->alignment_timeout = _arg;
    return *this;
  }
  Type & set__max_approach_speed(
    const float & _arg)
  {
    this->max_approach_speed = _arg;
    return *this;
  }
  Type & set__max_rotation_speed(
    const float & _arg)
  {
    this->max_rotation_speed = _arg;
    return *this;
  }
  Type & set__enable_collision_avoidance(
    const bool & _arg)
  {
    this->enable_collision_avoidance = _arg;
    return *this;
  }
  Type & set__safety_zones(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->safety_zones = _arg;
    return *this;
  }
  Type & set__require_position_feedback(
    const bool & _arg)
  {
    this->require_position_feedback = _arg;
    return *this;
  }
  Type & set__require_force_feedback(
    const bool & _arg)
  {
    this->require_force_feedback = _arg;
    return *this;
  }
  Type & set__feedback_rate(
    const float & _arg)
  {
    this->feedback_rate = _arg;
    return *this;
  }
  Type & set__required_aruco_tags(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->required_aruco_tags = _arg;
    return *this;
  }
  Type & set__tag_visibility_timeout(
    const float & _arg)
  {
    this->tag_visibility_timeout = _arg;
    return *this;
  }
  Type & set__allow_realignment(
    const bool & _arg)
  {
    this->allow_realignment = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__ArmAlignmentCommand
    std::shared_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__ArmAlignmentCommand
    std::shared_ptr<autonomy_interfaces::msg::ArmAlignmentCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmAlignmentCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mission_type != other.mission_type) {
      return false;
    }
    if (this->alignment_id != other.alignment_id) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->target_orientation != other.target_orientation) {
      return false;
    }
    if (this->approach_distance != other.approach_distance) {
      return false;
    }
    if (this->final_distance != other.final_distance) {
      return false;
    }
    if (this->alignment_quality != other.alignment_quality) {
      return false;
    }
    if (this->max_position_error != other.max_position_error) {
      return false;
    }
    if (this->max_orientation_error != other.max_orientation_error) {
      return false;
    }
    if (this->alignment_timeout != other.alignment_timeout) {
      return false;
    }
    if (this->max_approach_speed != other.max_approach_speed) {
      return false;
    }
    if (this->max_rotation_speed != other.max_rotation_speed) {
      return false;
    }
    if (this->enable_collision_avoidance != other.enable_collision_avoidance) {
      return false;
    }
    if (this->safety_zones != other.safety_zones) {
      return false;
    }
    if (this->require_position_feedback != other.require_position_feedback) {
      return false;
    }
    if (this->require_force_feedback != other.require_force_feedback) {
      return false;
    }
    if (this->feedback_rate != other.feedback_rate) {
      return false;
    }
    if (this->required_aruco_tags != other.required_aruco_tags) {
      return false;
    }
    if (this->tag_visibility_timeout != other.tag_visibility_timeout) {
      return false;
    }
    if (this->allow_realignment != other.allow_realignment) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmAlignmentCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmAlignmentCommand_

// alias to use template instance with default allocator
using ArmAlignmentCommand =
  autonomy_interfaces::msg::ArmAlignmentCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ARM_ALIGNMENT_COMMAND__STRUCT_HPP_
