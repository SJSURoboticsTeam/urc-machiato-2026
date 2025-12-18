// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FollowMeControl_Request_
{
  using Type = FollowMeControl_Request_<ContainerAllocator>;

  explicit FollowMeControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_tag_id = 0l;
      this->safety_distance = 0.0f;
      this->max_speed = 0.0f;
      this->enable_following = false;
      this->operator_id = "";
    }
  }

  explicit FollowMeControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operator_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_tag_id = 0l;
      this->safety_distance = 0.0f;
      this->max_speed = 0.0f;
      this->enable_following = false;
      this->operator_id = "";
    }
  }

  // field types and members
  using _target_tag_id_type =
    int32_t;
  _target_tag_id_type target_tag_id;
  using _safety_distance_type =
    float;
  _safety_distance_type safety_distance;
  using _max_speed_type =
    float;
  _max_speed_type max_speed;
  using _enable_following_type =
    bool;
  _enable_following_type enable_following;
  using _operator_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operator_id_type operator_id;

  // setters for named parameter idiom
  Type & set__target_tag_id(
    const int32_t & _arg)
  {
    this->target_tag_id = _arg;
    return *this;
  }
  Type & set__safety_distance(
    const float & _arg)
  {
    this->safety_distance = _arg;
    return *this;
  }
  Type & set__max_speed(
    const float & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__enable_following(
    const bool & _arg)
  {
    this->enable_following = _arg;
    return *this;
  }
  Type & set__operator_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operator_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Request
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Request
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowMeControl_Request_ & other) const
  {
    if (this->target_tag_id != other.target_tag_id) {
      return false;
    }
    if (this->safety_distance != other.safety_distance) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->enable_following != other.enable_following) {
      return false;
    }
    if (this->operator_id != other.operator_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowMeControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowMeControl_Request_

// alias to use template instance with default allocator
using FollowMeControl_Request =
  autonomy_interfaces::srv::FollowMeControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FollowMeControl_Response_
{
  using Type = FollowMeControl_Response_<ContainerAllocator>;

  explicit FollowMeControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->is_following = false;
      this->current_target_tag = 0l;
      this->current_distance = 0.0f;
    }
  }

  explicit FollowMeControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    target_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->is_following = false;
      this->current_target_tag = 0l;
      this->current_distance = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _is_following_type =
    bool;
  _is_following_type is_following;
  using _current_target_tag_type =
    int32_t;
  _current_target_tag_type current_target_tag;
  using _current_distance_type =
    float;
  _current_distance_type current_distance;
  using _target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_position_type target_position;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__is_following(
    const bool & _arg)
  {
    this->is_following = _arg;
    return *this;
  }
  Type & set__current_target_tag(
    const int32_t & _arg)
  {
    this->current_target_tag = _arg;
    return *this;
  }
  Type & set__current_distance(
    const float & _arg)
  {
    this->current_distance = _arg;
    return *this;
  }
  Type & set__target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Response
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__FollowMeControl_Response
    std::shared_ptr<autonomy_interfaces::srv::FollowMeControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowMeControl_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->is_following != other.is_following) {
      return false;
    }
    if (this->current_target_tag != other.current_target_tag) {
      return false;
    }
    if (this->current_distance != other.current_distance) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowMeControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowMeControl_Response_

// alias to use template instance with default allocator
using FollowMeControl_Response =
  autonomy_interfaces::srv::FollowMeControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct FollowMeControl
{
  using Request = autonomy_interfaces::srv::FollowMeControl_Request;
  using Response = autonomy_interfaces::srv::FollowMeControl_Response;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__FOLLOW_ME_CONTROL__STRUCT_HPP_
