// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/monitoring_stats.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__MonitoringStats __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__MonitoringStats __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MonitoringStats_
{
  using Type = MonitoringStats_<ContainerAllocator>;

  explicit MonitoringStats_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_evaluations = 0l;
      this->total_violations = 0l;
      this->evaluation_rate = 0.0;
    }
  }

  explicit MonitoringStats_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_evaluations = 0l;
      this->total_violations = 0l;
      this->evaluation_rate = 0.0;
    }
  }

  // field types and members
  using _total_evaluations_type =
    int32_t;
  _total_evaluations_type total_evaluations;
  using _total_violations_type =
    int32_t;
  _total_violations_type total_violations;
  using _evaluation_rate_type =
    double;
  _evaluation_rate_type evaluation_rate;

  // setters for named parameter idiom
  Type & set__total_evaluations(
    const int32_t & _arg)
  {
    this->total_evaluations = _arg;
    return *this;
  }
  Type & set__total_violations(
    const int32_t & _arg)
  {
    this->total_violations = _arg;
    return *this;
  }
  Type & set__evaluation_rate(
    const double & _arg)
  {
    this->evaluation_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__MonitoringStats
    std::shared_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__MonitoringStats
    std::shared_ptr<autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MonitoringStats_ & other) const
  {
    if (this->total_evaluations != other.total_evaluations) {
      return false;
    }
    if (this->total_violations != other.total_violations) {
      return false;
    }
    if (this->evaluation_rate != other.evaluation_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const MonitoringStats_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MonitoringStats_

// alias to use template instance with default allocator
using MonitoringStats =
  autonomy_interfaces::msg::MonitoringStats_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_HPP_
