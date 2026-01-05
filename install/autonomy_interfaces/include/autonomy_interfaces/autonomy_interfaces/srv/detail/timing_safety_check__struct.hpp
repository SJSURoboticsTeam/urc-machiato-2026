// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/timing_safety_check.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TimingSafetyCheck_Request_
{
  using Type = TimingSafetyCheck_Request_<ContainerAllocator>;

  explicit TimingSafetyCheck_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->real_time_check = false;
      this->time_window = 0.0;
    }
  }

  explicit TimingSafetyCheck_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->real_time_check = false;
      this->time_window = 0.0;
    }
  }

  // field types and members
  using _real_time_check_type =
    bool;
  _real_time_check_type real_time_check;
  using _time_window_type =
    double;
  _time_window_type time_window;
  using _monitored_components_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _monitored_components_type monitored_components;

  // setters for named parameter idiom
  Type & set__real_time_check(
    const bool & _arg)
  {
    this->real_time_check = _arg;
    return *this;
  }
  Type & set__time_window(
    const double & _arg)
  {
    this->time_window = _arg;
    return *this;
  }
  Type & set__monitored_components(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->monitored_components = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Request
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Request
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimingSafetyCheck_Request_ & other) const
  {
    if (this->real_time_check != other.real_time_check) {
      return false;
    }
    if (this->time_window != other.time_window) {
      return false;
    }
    if (this->monitored_components != other.monitored_components) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimingSafetyCheck_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimingSafetyCheck_Request_

// alias to use template instance with default allocator
using TimingSafetyCheck_Request =
  autonomy_interfaces::srv::TimingSafetyCheck_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TimingSafetyCheck_Response_
{
  using Type = TimingSafetyCheck_Response_<ContainerAllocator>;

  explicit TimingSafetyCheck_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timing_safe = false;
      this->timing_status = "";
      this->avg_response_time = 0.0;
      this->max_response_time = 0.0;
      this->min_response_time = 0.0;
      this->jitter = 0.0;
      this->deadline_misses = 0l;
      this->deadline_miss_rate = 0.0;
      this->cpu_utilization = 0.0;
      this->memory_utilization = 0.0;
      this->thread_count = 0l;
      this->real_time_scheduling = false;
      this->timestamp = "";
    }
  }

  explicit TimingSafetyCheck_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timing_status(_alloc),
    timestamp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timing_safe = false;
      this->timing_status = "";
      this->avg_response_time = 0.0;
      this->max_response_time = 0.0;
      this->min_response_time = 0.0;
      this->jitter = 0.0;
      this->deadline_misses = 0l;
      this->deadline_miss_rate = 0.0;
      this->cpu_utilization = 0.0;
      this->memory_utilization = 0.0;
      this->thread_count = 0l;
      this->real_time_scheduling = false;
      this->timestamp = "";
    }
  }

  // field types and members
  using _timing_safe_type =
    bool;
  _timing_safe_type timing_safe;
  using _timing_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _timing_status_type timing_status;
  using _avg_response_time_type =
    double;
  _avg_response_time_type avg_response_time;
  using _max_response_time_type =
    double;
  _max_response_time_type max_response_time;
  using _min_response_time_type =
    double;
  _min_response_time_type min_response_time;
  using _jitter_type =
    double;
  _jitter_type jitter;
  using _deadline_misses_type =
    int32_t;
  _deadline_misses_type deadline_misses;
  using _deadline_miss_rate_type =
    double;
  _deadline_miss_rate_type deadline_miss_rate;
  using _components_checked_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _components_checked_type components_checked;
  using _component_avg_times_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _component_avg_times_type component_avg_times;
  using _component_deadlines_missed_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _component_deadlines_missed_type component_deadlines_missed;
  using _cpu_utilization_type =
    double;
  _cpu_utilization_type cpu_utilization;
  using _memory_utilization_type =
    double;
  _memory_utilization_type memory_utilization;
  using _thread_count_type =
    int32_t;
  _thread_count_type thread_count;
  using _real_time_scheduling_type =
    bool;
  _real_time_scheduling_type real_time_scheduling;
  using _timing_recommendations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _timing_recommendations_type timing_recommendations;
  using _timestamp_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__timing_safe(
    const bool & _arg)
  {
    this->timing_safe = _arg;
    return *this;
  }
  Type & set__timing_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->timing_status = _arg;
    return *this;
  }
  Type & set__avg_response_time(
    const double & _arg)
  {
    this->avg_response_time = _arg;
    return *this;
  }
  Type & set__max_response_time(
    const double & _arg)
  {
    this->max_response_time = _arg;
    return *this;
  }
  Type & set__min_response_time(
    const double & _arg)
  {
    this->min_response_time = _arg;
    return *this;
  }
  Type & set__jitter(
    const double & _arg)
  {
    this->jitter = _arg;
    return *this;
  }
  Type & set__deadline_misses(
    const int32_t & _arg)
  {
    this->deadline_misses = _arg;
    return *this;
  }
  Type & set__deadline_miss_rate(
    const double & _arg)
  {
    this->deadline_miss_rate = _arg;
    return *this;
  }
  Type & set__components_checked(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->components_checked = _arg;
    return *this;
  }
  Type & set__component_avg_times(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->component_avg_times = _arg;
    return *this;
  }
  Type & set__component_deadlines_missed(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->component_deadlines_missed = _arg;
    return *this;
  }
  Type & set__cpu_utilization(
    const double & _arg)
  {
    this->cpu_utilization = _arg;
    return *this;
  }
  Type & set__memory_utilization(
    const double & _arg)
  {
    this->memory_utilization = _arg;
    return *this;
  }
  Type & set__thread_count(
    const int32_t & _arg)
  {
    this->thread_count = _arg;
    return *this;
  }
  Type & set__real_time_scheduling(
    const bool & _arg)
  {
    this->real_time_scheduling = _arg;
    return *this;
  }
  Type & set__timing_recommendations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->timing_recommendations = _arg;
    return *this;
  }
  Type & set__timestamp(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Response
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Response
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimingSafetyCheck_Response_ & other) const
  {
    if (this->timing_safe != other.timing_safe) {
      return false;
    }
    if (this->timing_status != other.timing_status) {
      return false;
    }
    if (this->avg_response_time != other.avg_response_time) {
      return false;
    }
    if (this->max_response_time != other.max_response_time) {
      return false;
    }
    if (this->min_response_time != other.min_response_time) {
      return false;
    }
    if (this->jitter != other.jitter) {
      return false;
    }
    if (this->deadline_misses != other.deadline_misses) {
      return false;
    }
    if (this->deadline_miss_rate != other.deadline_miss_rate) {
      return false;
    }
    if (this->components_checked != other.components_checked) {
      return false;
    }
    if (this->component_avg_times != other.component_avg_times) {
      return false;
    }
    if (this->component_deadlines_missed != other.component_deadlines_missed) {
      return false;
    }
    if (this->cpu_utilization != other.cpu_utilization) {
      return false;
    }
    if (this->memory_utilization != other.memory_utilization) {
      return false;
    }
    if (this->thread_count != other.thread_count) {
      return false;
    }
    if (this->real_time_scheduling != other.real_time_scheduling) {
      return false;
    }
    if (this->timing_recommendations != other.timing_recommendations) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimingSafetyCheck_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimingSafetyCheck_Response_

// alias to use template instance with default allocator
using TimingSafetyCheck_Response =
  autonomy_interfaces::srv::TimingSafetyCheck_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TimingSafetyCheck_Event_
{
  using Type = TimingSafetyCheck_Event_<ContainerAllocator>;

  explicit TimingSafetyCheck_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit TimingSafetyCheck_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::TimingSafetyCheck_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::TimingSafetyCheck_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Event
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__TimingSafetyCheck_Event
    std::shared_ptr<autonomy_interfaces::srv::TimingSafetyCheck_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimingSafetyCheck_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimingSafetyCheck_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimingSafetyCheck_Event_

// alias to use template instance with default allocator
using TimingSafetyCheck_Event =
  autonomy_interfaces::srv::TimingSafetyCheck_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct TimingSafetyCheck
{
  using Request = autonomy_interfaces::srv::TimingSafetyCheck_Request;
  using Response = autonomy_interfaces::srv::TimingSafetyCheck_Response;
  using Event = autonomy_interfaces::srv::TimingSafetyCheck_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_HPP_
