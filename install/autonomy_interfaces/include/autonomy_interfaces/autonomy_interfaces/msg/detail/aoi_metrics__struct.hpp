// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/aoi_metrics.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_HPP_

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
// Member 'last_alert_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__msg__AOIMetrics __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__msg__AOIMetrics __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AOIMetrics_
{
  using Type = AOIMetrics_<ContainerAllocator>;

  explicit AOIMetrics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    last_alert_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_average_aoi = 0.0;
      this->total_sensors = 0ul;
      this->fresh_sensors = 0ul;
      this->stale_sensors = 0ul;
      this->critical_sensors = 0ul;
      this->aoi_p50 = 0.0;
      this->aoi_p90 = 0.0;
      this->aoi_p95 = 0.0;
      this->aoi_p99 = 0.0;
      this->system_healthy = false;
      this->health_status = "";
      this->health_score = 0.0;
      this->update_rate_hz = 0.0;
      this->dropped_updates = 0ul;
      this->processing_latency = 0.0;
      this->alert_count = 0ul;
      this->serial_sensors = 0ul;
      this->can_sensors = 0ul;
      this->ethernet_sensors = 0ul;
      this->local_sensors = 0ul;
      this->avg_network_latency = 0.0;
      this->max_network_latency = 0.0;
      this->congested_links = 0ul;
      this->network_health_score = 0.0;
    }
  }

  explicit AOIMetrics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    health_status(_alloc),
    last_alert_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_average_aoi = 0.0;
      this->total_sensors = 0ul;
      this->fresh_sensors = 0ul;
      this->stale_sensors = 0ul;
      this->critical_sensors = 0ul;
      this->aoi_p50 = 0.0;
      this->aoi_p90 = 0.0;
      this->aoi_p95 = 0.0;
      this->aoi_p99 = 0.0;
      this->system_healthy = false;
      this->health_status = "";
      this->health_score = 0.0;
      this->update_rate_hz = 0.0;
      this->dropped_updates = 0ul;
      this->processing_latency = 0.0;
      this->alert_count = 0ul;
      this->serial_sensors = 0ul;
      this->can_sensors = 0ul;
      this->ethernet_sensors = 0ul;
      this->local_sensors = 0ul;
      this->avg_network_latency = 0.0;
      this->max_network_latency = 0.0;
      this->congested_links = 0ul;
      this->network_health_score = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _system_average_aoi_type =
    double;
  _system_average_aoi_type system_average_aoi;
  using _total_sensors_type =
    uint32_t;
  _total_sensors_type total_sensors;
  using _fresh_sensors_type =
    uint32_t;
  _fresh_sensors_type fresh_sensors;
  using _stale_sensors_type =
    uint32_t;
  _stale_sensors_type stale_sensors;
  using _critical_sensors_type =
    uint32_t;
  _critical_sensors_type critical_sensors;
  using _aoi_p50_type =
    double;
  _aoi_p50_type aoi_p50;
  using _aoi_p90_type =
    double;
  _aoi_p90_type aoi_p90;
  using _aoi_p95_type =
    double;
  _aoi_p95_type aoi_p95;
  using _aoi_p99_type =
    double;
  _aoi_p99_type aoi_p99;
  using _system_healthy_type =
    bool;
  _system_healthy_type system_healthy;
  using _health_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _health_status_type health_status;
  using _health_score_type =
    double;
  _health_score_type health_score;
  using _update_rate_hz_type =
    double;
  _update_rate_hz_type update_rate_hz;
  using _dropped_updates_type =
    uint32_t;
  _dropped_updates_type dropped_updates;
  using _processing_latency_type =
    double;
  _processing_latency_type processing_latency;
  using _active_alerts_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_alerts_type active_alerts;
  using _alert_count_type =
    uint32_t;
  _alert_count_type alert_count;
  using _last_alert_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_alert_time_type last_alert_time;
  using _serial_sensors_type =
    uint32_t;
  _serial_sensors_type serial_sensors;
  using _can_sensors_type =
    uint32_t;
  _can_sensors_type can_sensors;
  using _ethernet_sensors_type =
    uint32_t;
  _ethernet_sensors_type ethernet_sensors;
  using _local_sensors_type =
    uint32_t;
  _local_sensors_type local_sensors;
  using _avg_network_latency_type =
    double;
  _avg_network_latency_type avg_network_latency;
  using _max_network_latency_type =
    double;
  _max_network_latency_type max_network_latency;
  using _congested_links_type =
    uint32_t;
  _congested_links_type congested_links;
  using _network_recommendations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _network_recommendations_type network_recommendations;
  using _network_health_score_type =
    double;
  _network_health_score_type network_health_score;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__system_average_aoi(
    const double & _arg)
  {
    this->system_average_aoi = _arg;
    return *this;
  }
  Type & set__total_sensors(
    const uint32_t & _arg)
  {
    this->total_sensors = _arg;
    return *this;
  }
  Type & set__fresh_sensors(
    const uint32_t & _arg)
  {
    this->fresh_sensors = _arg;
    return *this;
  }
  Type & set__stale_sensors(
    const uint32_t & _arg)
  {
    this->stale_sensors = _arg;
    return *this;
  }
  Type & set__critical_sensors(
    const uint32_t & _arg)
  {
    this->critical_sensors = _arg;
    return *this;
  }
  Type & set__aoi_p50(
    const double & _arg)
  {
    this->aoi_p50 = _arg;
    return *this;
  }
  Type & set__aoi_p90(
    const double & _arg)
  {
    this->aoi_p90 = _arg;
    return *this;
  }
  Type & set__aoi_p95(
    const double & _arg)
  {
    this->aoi_p95 = _arg;
    return *this;
  }
  Type & set__aoi_p99(
    const double & _arg)
  {
    this->aoi_p99 = _arg;
    return *this;
  }
  Type & set__system_healthy(
    const bool & _arg)
  {
    this->system_healthy = _arg;
    return *this;
  }
  Type & set__health_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->health_status = _arg;
    return *this;
  }
  Type & set__health_score(
    const double & _arg)
  {
    this->health_score = _arg;
    return *this;
  }
  Type & set__update_rate_hz(
    const double & _arg)
  {
    this->update_rate_hz = _arg;
    return *this;
  }
  Type & set__dropped_updates(
    const uint32_t & _arg)
  {
    this->dropped_updates = _arg;
    return *this;
  }
  Type & set__processing_latency(
    const double & _arg)
  {
    this->processing_latency = _arg;
    return *this;
  }
  Type & set__active_alerts(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_alerts = _arg;
    return *this;
  }
  Type & set__alert_count(
    const uint32_t & _arg)
  {
    this->alert_count = _arg;
    return *this;
  }
  Type & set__last_alert_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_alert_time = _arg;
    return *this;
  }
  Type & set__serial_sensors(
    const uint32_t & _arg)
  {
    this->serial_sensors = _arg;
    return *this;
  }
  Type & set__can_sensors(
    const uint32_t & _arg)
  {
    this->can_sensors = _arg;
    return *this;
  }
  Type & set__ethernet_sensors(
    const uint32_t & _arg)
  {
    this->ethernet_sensors = _arg;
    return *this;
  }
  Type & set__local_sensors(
    const uint32_t & _arg)
  {
    this->local_sensors = _arg;
    return *this;
  }
  Type & set__avg_network_latency(
    const double & _arg)
  {
    this->avg_network_latency = _arg;
    return *this;
  }
  Type & set__max_network_latency(
    const double & _arg)
  {
    this->max_network_latency = _arg;
    return *this;
  }
  Type & set__congested_links(
    const uint32_t & _arg)
  {
    this->congested_links = _arg;
    return *this;
  }
  Type & set__network_recommendations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->network_recommendations = _arg;
    return *this;
  }
  Type & set__network_health_score(
    const double & _arg)
  {
    this->network_health_score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__msg__AOIMetrics
    std::shared_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__msg__AOIMetrics
    std::shared_ptr<autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AOIMetrics_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->system_average_aoi != other.system_average_aoi) {
      return false;
    }
    if (this->total_sensors != other.total_sensors) {
      return false;
    }
    if (this->fresh_sensors != other.fresh_sensors) {
      return false;
    }
    if (this->stale_sensors != other.stale_sensors) {
      return false;
    }
    if (this->critical_sensors != other.critical_sensors) {
      return false;
    }
    if (this->aoi_p50 != other.aoi_p50) {
      return false;
    }
    if (this->aoi_p90 != other.aoi_p90) {
      return false;
    }
    if (this->aoi_p95 != other.aoi_p95) {
      return false;
    }
    if (this->aoi_p99 != other.aoi_p99) {
      return false;
    }
    if (this->system_healthy != other.system_healthy) {
      return false;
    }
    if (this->health_status != other.health_status) {
      return false;
    }
    if (this->health_score != other.health_score) {
      return false;
    }
    if (this->update_rate_hz != other.update_rate_hz) {
      return false;
    }
    if (this->dropped_updates != other.dropped_updates) {
      return false;
    }
    if (this->processing_latency != other.processing_latency) {
      return false;
    }
    if (this->active_alerts != other.active_alerts) {
      return false;
    }
    if (this->alert_count != other.alert_count) {
      return false;
    }
    if (this->last_alert_time != other.last_alert_time) {
      return false;
    }
    if (this->serial_sensors != other.serial_sensors) {
      return false;
    }
    if (this->can_sensors != other.can_sensors) {
      return false;
    }
    if (this->ethernet_sensors != other.ethernet_sensors) {
      return false;
    }
    if (this->local_sensors != other.local_sensors) {
      return false;
    }
    if (this->avg_network_latency != other.avg_network_latency) {
      return false;
    }
    if (this->max_network_latency != other.max_network_latency) {
      return false;
    }
    if (this->congested_links != other.congested_links) {
      return false;
    }
    if (this->network_recommendations != other.network_recommendations) {
      return false;
    }
    if (this->network_health_score != other.network_health_score) {
      return false;
    }
    return true;
  }
  bool operator!=(const AOIMetrics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AOIMetrics_

// alias to use template instance with default allocator
using AOIMetrics =
  autonomy_interfaces::msg::AOIMetrics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_HPP_
