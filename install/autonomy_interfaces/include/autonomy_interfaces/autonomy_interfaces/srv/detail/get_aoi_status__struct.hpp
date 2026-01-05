// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_aoi_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAOIStatus_Request_
{
  using Type = GetAOIStatus_Request_<ContainerAllocator>;

  explicit GetAOIStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->include_history = false;
      this->history_samples = 0ul;
    }
  }

  explicit GetAOIStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sensor_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->include_history = false;
      this->history_samples = 0ul;
    }
  }

  // field types and members
  using _sensor_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sensor_name_type sensor_name;
  using _include_history_type =
    bool;
  _include_history_type include_history;
  using _history_samples_type =
    uint32_t;
  _history_samples_type history_samples;

  // setters for named parameter idiom
  Type & set__sensor_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sensor_name = _arg;
    return *this;
  }
  Type & set__include_history(
    const bool & _arg)
  {
    this->include_history = _arg;
    return *this;
  }
  Type & set__history_samples(
    const uint32_t & _arg)
  {
    this->history_samples = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Request
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Request
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAOIStatus_Request_ & other) const
  {
    if (this->sensor_name != other.sensor_name) {
      return false;
    }
    if (this->include_history != other.include_history) {
      return false;
    }
    if (this->history_samples != other.history_samples) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAOIStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAOIStatus_Request_

// alias to use template instance with default allocator
using GetAOIStatus_Request =
  autonomy_interfaces::srv::GetAOIStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'sensor_status'
#include "autonomy_interfaces/msg/detail/aoi_status__struct.hpp"
// Member 'timestamp_history'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'system_metrics'
#include "autonomy_interfaces/msg/detail/aoi_metrics__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAOIStatus_Response_
{
  using Type = GetAOIStatus_Response_<ContainerAllocator>;

  explicit GetAOIStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : system_metrics(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GetAOIStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    system_metrics(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _sensor_status_type =
    std::vector<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>>;
  _sensor_status_type sensor_status;
  using _aoi_history_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _aoi_history_type aoi_history;
  using _timestamp_history_type =
    std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>>;
  _timestamp_history_type timestamp_history;
  using _system_metrics_type =
    autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator>;
  _system_metrics_type system_metrics;

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
  Type & set__sensor_status(
    const std::vector<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::msg::AOIStatus_<ContainerAllocator>>> & _arg)
  {
    this->sensor_status = _arg;
    return *this;
  }
  Type & set__aoi_history(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->aoi_history = _arg;
    return *this;
  }
  Type & set__timestamp_history(
    const std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>> & _arg)
  {
    this->timestamp_history = _arg;
    return *this;
  }
  Type & set__system_metrics(
    const autonomy_interfaces::msg::AOIMetrics_<ContainerAllocator> & _arg)
  {
    this->system_metrics = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Response
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Response
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAOIStatus_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->sensor_status != other.sensor_status) {
      return false;
    }
    if (this->aoi_history != other.aoi_history) {
      return false;
    }
    if (this->timestamp_history != other.timestamp_history) {
      return false;
    }
    if (this->system_metrics != other.system_metrics) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAOIStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAOIStatus_Response_

// alias to use template instance with default allocator
using GetAOIStatus_Response =
  autonomy_interfaces::srv::GetAOIStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAOIStatus_Event_
{
  using Type = GetAOIStatus_Event_<ContainerAllocator>;

  explicit GetAOIStatus_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetAOIStatus_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetAOIStatus_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetAOIStatus_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Event
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetAOIStatus_Event
    std::shared_ptr<autonomy_interfaces::srv::GetAOIStatus_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAOIStatus_Event_ & other) const
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
  bool operator!=(const GetAOIStatus_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAOIStatus_Event_

// alias to use template instance with default allocator
using GetAOIStatus_Event =
  autonomy_interfaces::srv::GetAOIStatus_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct GetAOIStatus
{
  using Request = autonomy_interfaces::srv::GetAOIStatus_Request;
  using Response = autonomy_interfaces::srv::GetAOIStatus_Response;
  using Event = autonomy_interfaces::srv::GetAOIStatus_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_HPP_
