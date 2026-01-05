// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/software_estop.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SoftwareEstop_Request_
{
  using Type = SoftwareEstop_Request_<ContainerAllocator>;

  explicit SoftwareEstop_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operator_id = "";
      this->reason = "";
      this->acknowledge_criticality = false;
      this->force_immediate = false;
    }
  }

  explicit SoftwareEstop_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operator_id(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operator_id = "";
      this->reason = "";
      this->acknowledge_criticality = false;
      this->force_immediate = false;
    }
  }

  // field types and members
  using _operator_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operator_id_type operator_id;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _acknowledge_criticality_type =
    bool;
  _acknowledge_criticality_type acknowledge_criticality;
  using _force_immediate_type =
    bool;
  _force_immediate_type force_immediate;

  // setters for named parameter idiom
  Type & set__operator_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operator_id = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__acknowledge_criticality(
    const bool & _arg)
  {
    this->acknowledge_criticality = _arg;
    return *this;
  }
  Type & set__force_immediate(
    const bool & _arg)
  {
    this->force_immediate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Request
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Request
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SoftwareEstop_Request_ & other) const
  {
    if (this->operator_id != other.operator_id) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->acknowledge_criticality != other.acknowledge_criticality) {
      return false;
    }
    if (this->force_immediate != other.force_immediate) {
      return false;
    }
    return true;
  }
  bool operator!=(const SoftwareEstop_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SoftwareEstop_Request_

// alias to use template instance with default allocator
using SoftwareEstop_Request =
  autonomy_interfaces::srv::SoftwareEstop_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SoftwareEstop_Response_
{
  using Type = SoftwareEstop_Response_<ContainerAllocator>;

  explicit SoftwareEstop_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->estop_id = "";
      this->timestamp = 0.0;
      this->triggered_by = "";
      this->coordination_started = false;
    }
  }

  explicit SoftwareEstop_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    estop_id(_alloc),
    triggered_by(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->estop_id = "";
      this->timestamp = 0.0;
      this->triggered_by = "";
      this->coordination_started = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _estop_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _estop_id_type estop_id;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _triggered_by_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _triggered_by_type triggered_by;
  using _coordination_started_type =
    bool;
  _coordination_started_type coordination_started;

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
  Type & set__estop_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->estop_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__triggered_by(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->triggered_by = _arg;
    return *this;
  }
  Type & set__coordination_started(
    const bool & _arg)
  {
    this->coordination_started = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Response
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Response
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SoftwareEstop_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->estop_id != other.estop_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->triggered_by != other.triggered_by) {
      return false;
    }
    if (this->coordination_started != other.coordination_started) {
      return false;
    }
    return true;
  }
  bool operator!=(const SoftwareEstop_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SoftwareEstop_Response_

// alias to use template instance with default allocator
using SoftwareEstop_Response =
  autonomy_interfaces::srv::SoftwareEstop_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SoftwareEstop_Event_
{
  using Type = SoftwareEstop_Event_<ContainerAllocator>;

  explicit SoftwareEstop_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SoftwareEstop_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::SoftwareEstop_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::SoftwareEstop_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Event
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__SoftwareEstop_Event
    std::shared_ptr<autonomy_interfaces::srv::SoftwareEstop_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SoftwareEstop_Event_ & other) const
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
  bool operator!=(const SoftwareEstop_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SoftwareEstop_Event_

// alias to use template instance with default allocator
using SoftwareEstop_Event =
  autonomy_interfaces::srv::SoftwareEstop_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct SoftwareEstop
{
  using Request = autonomy_interfaces::srv::SoftwareEstop_Request;
  using Response = autonomy_interfaces::srv::SoftwareEstop_Response;
  using Event = autonomy_interfaces::srv::SoftwareEstop_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_HPP_
