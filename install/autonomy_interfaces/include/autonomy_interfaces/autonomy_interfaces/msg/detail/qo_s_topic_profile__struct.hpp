// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_topic_profile.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__QoSTopicProfile __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__QoSTopicProfile __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct QoSTopicProfile_
{
  using Type = QoSTopicProfile_<ContainerAllocator>;

  explicit QoSTopicProfile_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
      this->avg_latency_ms = 0.0;
      this->jitter_ms = 0.0;
      this->packet_loss_rate = 0.0;
      this->samples_count = 0l;
    }
  }

  explicit QoSTopicProfile_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
      this->avg_latency_ms = 0.0;
      this->jitter_ms = 0.0;
      this->packet_loss_rate = 0.0;
      this->samples_count = 0l;
    }
  }

  // field types and members
  using _topic_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_name_type topic_name;
  using _avg_latency_ms_type =
    double;
  _avg_latency_ms_type avg_latency_ms;
  using _jitter_ms_type =
    double;
  _jitter_ms_type jitter_ms;
  using _packet_loss_rate_type =
    double;
  _packet_loss_rate_type packet_loss_rate;
  using _samples_count_type =
    int32_t;
  _samples_count_type samples_count;

  // setters for named parameter idiom
  Type & set__topic_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_name = _arg;
    return *this;
  }
  Type & set__avg_latency_ms(
    const double & _arg)
  {
    this->avg_latency_ms = _arg;
    return *this;
  }
  Type & set__jitter_ms(
    const double & _arg)
  {
    this->jitter_ms = _arg;
    return *this;
  }
  Type & set__packet_loss_rate(
    const double & _arg)
  {
    this->packet_loss_rate = _arg;
    return *this;
  }
  Type & set__samples_count(
    const int32_t & _arg)
  {
    this->samples_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__QoSTopicProfile
    std::shared_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__QoSTopicProfile
    std::shared_ptr<autonomy_interfaces::msg::QoSTopicProfile_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QoSTopicProfile_ & other) const
  {
    if (this->topic_name != other.topic_name) {
      return false;
    }
    if (this->avg_latency_ms != other.avg_latency_ms) {
      return false;
    }
    if (this->jitter_ms != other.jitter_ms) {
      return false;
    }
    if (this->packet_loss_rate != other.packet_loss_rate) {
      return false;
    }
    if (this->samples_count != other.samples_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const QoSTopicProfile_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QoSTopicProfile_

// alias to use template instance with default allocator
using QoSTopicProfile =
  autonomy_interfaces::msg::QoSTopicProfile_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_HPP_
