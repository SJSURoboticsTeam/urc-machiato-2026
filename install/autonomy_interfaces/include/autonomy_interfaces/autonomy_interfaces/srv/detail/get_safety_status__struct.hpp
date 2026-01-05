// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_safety_status.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSafetyStatus_Request_
{
  using Type = GetSafetyStatus_Request_<ContainerAllocator>;

  explicit GetSafetyStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetSafetyStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Request
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Request
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSafetyStatus_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSafetyStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSafetyStatus_Request_

// alias to use template instance with default allocator
using GetSafetyStatus_Request =
  autonomy_interfaces::srv::GetSafetyStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'active_alerts'
#include "autonomy_interfaces/msg/detail/safety_alert__struct.hpp"
// Member 'monitoring_stats'
#include "autonomy_interfaces/msg/detail/monitoring_stats__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSafetyStatus_Response_
{
  using Type = GetSafetyStatus_Response_<ContainerAllocator>;

  explicit GetSafetyStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : monitoring_stats(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->overall_safety = "";
    }
  }

  explicit GetSafetyStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : overall_safety(_alloc),
    monitoring_stats(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->overall_safety = "";
    }
  }

  // field types and members
  using _overall_safety_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _overall_safety_type overall_safety;
  using _active_alerts_type =
    std::vector<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>>;
  _active_alerts_type active_alerts;
  using _monitoring_stats_type =
    autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator>;
  _monitoring_stats_type monitoring_stats;

  // setters for named parameter idiom
  Type & set__overall_safety(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->overall_safety = _arg;
    return *this;
  }
  Type & set__active_alerts(
    const std::vector<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::msg::SafetyAlert_<ContainerAllocator>>> & _arg)
  {
    this->active_alerts = _arg;
    return *this;
  }
  Type & set__monitoring_stats(
    const autonomy_interfaces::msg::MonitoringStats_<ContainerAllocator> & _arg)
  {
    this->monitoring_stats = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Response
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Response
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSafetyStatus_Response_ & other) const
  {
    if (this->overall_safety != other.overall_safety) {
      return false;
    }
    if (this->active_alerts != other.active_alerts) {
      return false;
    }
    if (this->monitoring_stats != other.monitoring_stats) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSafetyStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSafetyStatus_Response_

// alias to use template instance with default allocator
using GetSafetyStatus_Response =
  autonomy_interfaces::srv::GetSafetyStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSafetyStatus_Event_
{
  using Type = GetSafetyStatus_Event_<ContainerAllocator>;

  explicit GetSafetyStatus_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetSafetyStatus_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetSafetyStatus_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::GetSafetyStatus_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Event
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__GetSafetyStatus_Event
    std::shared_ptr<autonomy_interfaces::srv::GetSafetyStatus_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSafetyStatus_Event_ & other) const
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
  bool operator!=(const GetSafetyStatus_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSafetyStatus_Event_

// alias to use template instance with default allocator
using GetSafetyStatus_Event =
  autonomy_interfaces::srv::GetSafetyStatus_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct GetSafetyStatus
{
  using Request = autonomy_interfaces::srv::GetSafetyStatus_Request;
  using Response = autonomy_interfaces::srv::GetSafetyStatus_Response;
  using Event = autonomy_interfaces::srv::GetSafetyStatus_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_HPP_
