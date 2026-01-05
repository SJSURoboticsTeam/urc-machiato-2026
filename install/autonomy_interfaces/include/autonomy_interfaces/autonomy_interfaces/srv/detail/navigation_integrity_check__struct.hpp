// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/navigation_integrity_check.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NavigationIntegrityCheck_Request_
{
  using Type = NavigationIntegrityCheck_Request_<ContainerAllocator>;

  explicit NavigationIntegrityCheck_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detailed_check = false;
    }
  }

  explicit NavigationIntegrityCheck_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detailed_check = false;
    }
  }

  // field types and members
  using _detailed_check_type =
    bool;
  _detailed_check_type detailed_check;
  using _check_components_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _check_components_type check_components;

  // setters for named parameter idiom
  Type & set__detailed_check(
    const bool & _arg)
  {
    this->detailed_check = _arg;
    return *this;
  }
  Type & set__check_components(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->check_components = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Request
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Request
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationIntegrityCheck_Request_ & other) const
  {
    if (this->detailed_check != other.detailed_check) {
      return false;
    }
    if (this->check_components != other.check_components) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigationIntegrityCheck_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationIntegrityCheck_Request_

// alias to use template instance with default allocator
using NavigationIntegrityCheck_Request =
  autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NavigationIntegrityCheck_Response_
{
  using Type = NavigationIntegrityCheck_Response_<ContainerAllocator>;

  explicit NavigationIntegrityCheck_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integrity_ok = false;
      this->integrity_score = 0.0;
      this->integrity_level = "";
      this->position_accuracy = 0.0;
      this->heading_accuracy = 0.0;
      this->velocity_consistency = 0.0;
      this->satellite_count = 0l;
      this->hdop = 0.0;
      this->timestamp = "";
    }
  }

  explicit NavigationIntegrityCheck_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : integrity_level(_alloc),
    timestamp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integrity_ok = false;
      this->integrity_score = 0.0;
      this->integrity_level = "";
      this->position_accuracy = 0.0;
      this->heading_accuracy = 0.0;
      this->velocity_consistency = 0.0;
      this->satellite_count = 0l;
      this->hdop = 0.0;
      this->timestamp = "";
    }
  }

  // field types and members
  using _integrity_ok_type =
    bool;
  _integrity_ok_type integrity_ok;
  using _integrity_score_type =
    double;
  _integrity_score_type integrity_score;
  using _integrity_level_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _integrity_level_type integrity_level;
  using _checked_components_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _checked_components_type checked_components;
  using _component_status_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _component_status_type component_status;
  using _component_details_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _component_details_type component_details;
  using _position_accuracy_type =
    double;
  _position_accuracy_type position_accuracy;
  using _heading_accuracy_type =
    double;
  _heading_accuracy_type heading_accuracy;
  using _velocity_consistency_type =
    double;
  _velocity_consistency_type velocity_consistency;
  using _satellite_count_type =
    int32_t;
  _satellite_count_type satellite_count;
  using _hdop_type =
    double;
  _hdop_type hdop;
  using _recommendations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _recommendations_type recommendations;
  using _timestamp_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__integrity_ok(
    const bool & _arg)
  {
    this->integrity_ok = _arg;
    return *this;
  }
  Type & set__integrity_score(
    const double & _arg)
  {
    this->integrity_score = _arg;
    return *this;
  }
  Type & set__integrity_level(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->integrity_level = _arg;
    return *this;
  }
  Type & set__checked_components(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->checked_components = _arg;
    return *this;
  }
  Type & set__component_status(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->component_status = _arg;
    return *this;
  }
  Type & set__component_details(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->component_details = _arg;
    return *this;
  }
  Type & set__position_accuracy(
    const double & _arg)
  {
    this->position_accuracy = _arg;
    return *this;
  }
  Type & set__heading_accuracy(
    const double & _arg)
  {
    this->heading_accuracy = _arg;
    return *this;
  }
  Type & set__velocity_consistency(
    const double & _arg)
  {
    this->velocity_consistency = _arg;
    return *this;
  }
  Type & set__satellite_count(
    const int32_t & _arg)
  {
    this->satellite_count = _arg;
    return *this;
  }
  Type & set__hdop(
    const double & _arg)
  {
    this->hdop = _arg;
    return *this;
  }
  Type & set__recommendations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->recommendations = _arg;
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
    autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Response
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Response
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationIntegrityCheck_Response_ & other) const
  {
    if (this->integrity_ok != other.integrity_ok) {
      return false;
    }
    if (this->integrity_score != other.integrity_score) {
      return false;
    }
    if (this->integrity_level != other.integrity_level) {
      return false;
    }
    if (this->checked_components != other.checked_components) {
      return false;
    }
    if (this->component_status != other.component_status) {
      return false;
    }
    if (this->component_details != other.component_details) {
      return false;
    }
    if (this->position_accuracy != other.position_accuracy) {
      return false;
    }
    if (this->heading_accuracy != other.heading_accuracy) {
      return false;
    }
    if (this->velocity_consistency != other.velocity_consistency) {
      return false;
    }
    if (this->satellite_count != other.satellite_count) {
      return false;
    }
    if (this->hdop != other.hdop) {
      return false;
    }
    if (this->recommendations != other.recommendations) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigationIntegrityCheck_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationIntegrityCheck_Response_

// alias to use template instance with default allocator
using NavigationIntegrityCheck_Response =
  autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NavigationIntegrityCheck_Event_
{
  using Type = NavigationIntegrityCheck_Event_<ContainerAllocator>;

  explicit NavigationIntegrityCheck_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit NavigationIntegrityCheck_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::NavigationIntegrityCheck_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::NavigationIntegrityCheck_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Event
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__NavigationIntegrityCheck_Event
    std::shared_ptr<autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationIntegrityCheck_Event_ & other) const
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
  bool operator!=(const NavigationIntegrityCheck_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationIntegrityCheck_Event_

// alias to use template instance with default allocator
using NavigationIntegrityCheck_Event =
  autonomy_interfaces::srv::NavigationIntegrityCheck_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct NavigationIntegrityCheck
{
  using Request = autonomy_interfaces::srv::NavigationIntegrityCheck_Request;
  using Response = autonomy_interfaces::srv::NavigationIntegrityCheck_Response;
  using Event = autonomy_interfaces::srv::NavigationIntegrityCheck_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__NAVIGATION_INTEGRITY_CHECK__STRUCT_HPP_
