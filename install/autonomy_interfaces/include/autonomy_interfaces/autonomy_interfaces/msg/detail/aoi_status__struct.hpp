// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__AOIStatus __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__AOIStatus __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AOIStatus_
{
  using Type = AOIStatus_<ContainerAllocator>;

  explicit AOIStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->sensor_type = "";
      this->current_aoi = 0.0;
      this->average_aoi = 0.0;
      this->max_aoi = 0.0;
      this->min_aoi = 0.0;
      this->is_fresh = false;
      this->quality_score = 0.0;
      this->freshness_status = "";
      this->acceptable_threshold = 0.0;
      this->optimal_threshold = 0.0;
      this->sample_count = 0ul;
      this->freshness_ratio = 0.0;
      this->transport_type = "";
      this->network_latency = 0.0;
      this->transport_latency = 0.0;
      this->congestion_detected = false;
      this->congestion_factor = 0.0;
      this->predicted_aoi = 0.0;
      this->aoi_trend = 0.0;
    }
  }

  explicit AOIStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    sensor_name(_alloc),
    sensor_type(_alloc),
    freshness_status(_alloc),
    transport_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->sensor_type = "";
      this->current_aoi = 0.0;
      this->average_aoi = 0.0;
      this->max_aoi = 0.0;
      this->min_aoi = 0.0;
      this->is_fresh = false;
      this->quality_score = 0.0;
      this->freshness_status = "";
      this->acceptable_threshold = 0.0;
      this->optimal_threshold = 0.0;
      this->sample_count = 0ul;
      this->freshness_ratio = 0.0;
      this->transport_type = "";
      this->network_latency = 0.0;
      this->transport_latency = 0.0;
      this->congestion_detected = false;
      this->congestion_factor = 0.0;
      this->predicted_aoi = 0.0;
      this->aoi_trend = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sensor_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sensor_name_type sensor_name;
  using _sensor_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sensor_type_type sensor_type;
  using _current_aoi_type =
    double;
  _current_aoi_type current_aoi;
  using _average_aoi_type =
    double;
  _average_aoi_type average_aoi;
  using _max_aoi_type =
    double;
  _max_aoi_type max_aoi;
  using _min_aoi_type =
    double;
  _min_aoi_type min_aoi;
  using _is_fresh_type =
    bool;
  _is_fresh_type is_fresh;
  using _quality_score_type =
    double;
  _quality_score_type quality_score;
  using _freshness_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _freshness_status_type freshness_status;
  using _acceptable_threshold_type =
    double;
  _acceptable_threshold_type acceptable_threshold;
  using _optimal_threshold_type =
    double;
  _optimal_threshold_type optimal_threshold;
  using _sample_count_type =
    uint32_t;
  _sample_count_type sample_count;
  using _freshness_ratio_type =
    double;
  _freshness_ratio_type freshness_ratio;
  using _transport_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _transport_type_type transport_type;
  using _network_latency_type =
    double;
  _network_latency_type network_latency;
  using _transport_latency_type =
    double;
  _transport_latency_type transport_latency;
  using _congestion_detected_type =
    bool;
  _congestion_detected_type congestion_detected;
  using _congestion_factor_type =
    double;
  _congestion_factor_type congestion_factor;
  using _predicted_aoi_type =
    double;
  _predicted_aoi_type predicted_aoi;
  using _aoi_trend_type =
    double;
  _aoi_trend_type aoi_trend;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__sensor_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sensor_name = _arg;
    return *this;
  }
  Type & set__sensor_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sensor_type = _arg;
    return *this;
  }
  Type & set__current_aoi(
    const double & _arg)
  {
    this->current_aoi = _arg;
    return *this;
  }
  Type & set__average_aoi(
    const double & _arg)
  {
    this->average_aoi = _arg;
    return *this;
  }
  Type & set__max_aoi(
    const double & _arg)
  {
    this->max_aoi = _arg;
    return *this;
  }
  Type & set__min_aoi(
    const double & _arg)
  {
    this->min_aoi = _arg;
    return *this;
  }
  Type & set__is_fresh(
    const bool & _arg)
  {
    this->is_fresh = _arg;
    return *this;
  }
  Type & set__quality_score(
    const double & _arg)
  {
    this->quality_score = _arg;
    return *this;
  }
  Type & set__freshness_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->freshness_status = _arg;
    return *this;
  }
  Type & set__acceptable_threshold(
    const double & _arg)
  {
    this->acceptable_threshold = _arg;
    return *this;
  }
  Type & set__optimal_threshold(
    const double & _arg)
  {
    this->optimal_threshold = _arg;
    return *this;
  }
  Type & set__sample_count(
    const uint32_t & _arg)
  {
    this->sample_count = _arg;
    return *this;
  }
  Type & set__freshness_ratio(
    const double & _arg)
  {
    this->freshness_ratio = _arg;
    return *this;
  }
  Type & set__transport_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->transport_type = _arg;
    return *this;
  }
  Type & set__network_latency(
    const double & _arg)
  {
    this->network_latency = _arg;
    return *this;
  }
  Type & set__transport_latency(
    const double & _arg)
  {
    this->transport_latency = _arg;
    return *this;
  }
  Type & set__congestion_detected(
    const bool & _arg)
  {
    this->congestion_detected = _arg;
    return *this;
  }
  Type & set__congestion_factor(
    const double & _arg)
  {
    this->congestion_factor = _arg;
    return *this;
  }
  Type & set__predicted_aoi(
    const double & _arg)
  {
    this->predicted_aoi = _arg;
    return *this;
  }
  Type & set__aoi_trend(
    const double & _arg)
  {
    this->aoi_trend = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__AOIStatus
    std::shared_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__AOIStatus
    std::shared_ptr<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AOIStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sensor_name != other.sensor_name) {
      return false;
    }
    if (this->sensor_type != other.sensor_type) {
      return false;
    }
    if (this->current_aoi != other.current_aoi) {
      return false;
    }
    if (this->average_aoi != other.average_aoi) {
      return false;
    }
    if (this->max_aoi != other.max_aoi) {
      return false;
    }
    if (this->min_aoi != other.min_aoi) {
      return false;
    }
    if (this->is_fresh != other.is_fresh) {
      return false;
    }
    if (this->quality_score != other.quality_score) {
      return false;
    }
    if (this->freshness_status != other.freshness_status) {
      return false;
    }
    if (this->acceptable_threshold != other.acceptable_threshold) {
      return false;
    }
    if (this->optimal_threshold != other.optimal_threshold) {
      return false;
    }
    if (this->sample_count != other.sample_count) {
      return false;
    }
    if (this->freshness_ratio != other.freshness_ratio) {
      return false;
    }
    if (this->transport_type != other.transport_type) {
      return false;
    }
    if (this->network_latency != other.network_latency) {
      return false;
    }
    if (this->transport_latency != other.transport_latency) {
      return false;
    }
    if (this->congestion_detected != other.congestion_detected) {
      return false;
    }
    if (this->congestion_factor != other.congestion_factor) {
      return false;
    }
    if (this->predicted_aoi != other.predicted_aoi) {
      return false;
    }
    if (this->aoi_trend != other.aoi_trend) {
      return false;
    }
    return true;
  }
  bool operator!=(const AOIStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AOIStatus_

// alias to use template instance with default allocator
using AOIStatus =
  autonomy_interfaces::msg::AOIStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_HPP_
