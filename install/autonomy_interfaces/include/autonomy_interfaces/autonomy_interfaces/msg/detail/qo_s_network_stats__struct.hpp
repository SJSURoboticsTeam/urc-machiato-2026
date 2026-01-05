// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_network_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__QoSNetworkStats __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__QoSNetworkStats __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct QoSNetworkStats_
{
  using Type = QoSNetworkStats_<ContainerAllocator>;

  explicit QoSNetworkStats_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bandwidth_up_mbps = 0.0;
      this->bandwidth_down_mbps = 0.0;
      this->latency_ms = 0.0;
      this->packet_loss_rate = 0.0;
      this->current_band = "";
      this->signal_strength = 0.0;
    }
  }

  explicit QoSNetworkStats_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_band(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bandwidth_up_mbps = 0.0;
      this->bandwidth_down_mbps = 0.0;
      this->latency_ms = 0.0;
      this->packet_loss_rate = 0.0;
      this->current_band = "";
      this->signal_strength = 0.0;
    }
  }

  // field types and members
  using _bandwidth_up_mbps_type =
    double;
  _bandwidth_up_mbps_type bandwidth_up_mbps;
  using _bandwidth_down_mbps_type =
    double;
  _bandwidth_down_mbps_type bandwidth_down_mbps;
  using _latency_ms_type =
    double;
  _latency_ms_type latency_ms;
  using _packet_loss_rate_type =
    double;
  _packet_loss_rate_type packet_loss_rate;
  using _current_band_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_band_type current_band;
  using _signal_strength_type =
    double;
  _signal_strength_type signal_strength;

  // setters for named parameter idiom
  Type & set__bandwidth_up_mbps(
    const double & _arg)
  {
    this->bandwidth_up_mbps = _arg;
    return *this;
  }
  Type & set__bandwidth_down_mbps(
    const double & _arg)
  {
    this->bandwidth_down_mbps = _arg;
    return *this;
  }
  Type & set__latency_ms(
    const double & _arg)
  {
    this->latency_ms = _arg;
    return *this;
  }
  Type & set__packet_loss_rate(
    const double & _arg)
  {
    this->packet_loss_rate = _arg;
    return *this;
  }
  Type & set__current_band(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_band = _arg;
    return *this;
  }
  Type & set__signal_strength(
    const double & _arg)
  {
    this->signal_strength = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__QoSNetworkStats
    std::shared_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__QoSNetworkStats
    std::shared_ptr<autonomy_interfaces::msg::QoSNetworkStats_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QoSNetworkStats_ & other) const
  {
    if (this->bandwidth_up_mbps != other.bandwidth_up_mbps) {
      return false;
    }
    if (this->bandwidth_down_mbps != other.bandwidth_down_mbps) {
      return false;
    }
    if (this->latency_ms != other.latency_ms) {
      return false;
    }
    if (this->packet_loss_rate != other.packet_loss_rate) {
      return false;
    }
    if (this->current_band != other.current_band) {
      return false;
    }
    if (this->signal_strength != other.signal_strength) {
      return false;
    }
    return true;
  }
  bool operator!=(const QoSNetworkStats_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QoSNetworkStats_

// alias to use template instance with default allocator
using QoSNetworkStats =
  autonomy_interfaces::msg::QoSNetworkStats_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_HPP_
