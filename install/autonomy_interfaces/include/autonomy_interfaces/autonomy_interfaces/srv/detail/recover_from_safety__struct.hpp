// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/recover_from_safety.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RecoverFromSafety_Request_
{
  using Type = RecoverFromSafety_Request_<ContainerAllocator>;

  explicit RecoverFromSafety_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recovery_method = "";
      this->operator_id = "";
      this->acknowledge_risks = false;
      this->notes = "";
    }
  }

  explicit RecoverFromSafety_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : recovery_method(_alloc),
    operator_id(_alloc),
    notes(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recovery_method = "";
      this->operator_id = "";
      this->acknowledge_risks = false;
      this->notes = "";
    }
  }

  // field types and members
  using _recovery_method_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recovery_method_type recovery_method;
  using _operator_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operator_id_type operator_id;
  using _acknowledge_risks_type =
    bool;
  _acknowledge_risks_type acknowledge_risks;
  using _completed_steps_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _completed_steps_type completed_steps;
  using _notes_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _notes_type notes;

  // setters for named parameter idiom
  Type & set__recovery_method(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recovery_method = _arg;
    return *this;
  }
  Type & set__operator_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operator_id = _arg;
    return *this;
  }
  Type & set__acknowledge_risks(
    const bool & _arg)
  {
    this->acknowledge_risks = _arg;
    return *this;
  }
  Type & set__completed_steps(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->completed_steps = _arg;
    return *this;
  }
  Type & set__notes(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->notes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Request
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Request
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecoverFromSafety_Request_ & other) const
  {
    if (this->recovery_method != other.recovery_method) {
      return false;
    }
    if (this->operator_id != other.operator_id) {
      return false;
    }
    if (this->acknowledge_risks != other.acknowledge_risks) {
      return false;
    }
    if (this->completed_steps != other.completed_steps) {
      return false;
    }
    if (this->notes != other.notes) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecoverFromSafety_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecoverFromSafety_Request_

// alias to use template instance with default allocator
using RecoverFromSafety_Request =
  autonomy_interfaces::srv::RecoverFromSafety_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RecoverFromSafety_Response_
{
  using Type = RecoverFromSafety_Response_<ContainerAllocator>;

  explicit RecoverFromSafety_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->recovery_state = "";
      this->is_safe_to_proceed = false;
      this->estimated_time = 0.0;
      this->recommended_next_state = "";
    }
  }

  explicit RecoverFromSafety_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    recovery_state(_alloc),
    recommended_next_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->recovery_state = "";
      this->is_safe_to_proceed = false;
      this->estimated_time = 0.0;
      this->recommended_next_state = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _recovery_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recovery_state_type recovery_state;
  using _is_safe_to_proceed_type =
    bool;
  _is_safe_to_proceed_type is_safe_to_proceed;
  using _remaining_steps_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _remaining_steps_type remaining_steps;
  using _verified_systems_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _verified_systems_type verified_systems;
  using _failed_systems_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _failed_systems_type failed_systems;
  using _estimated_time_type =
    double;
  _estimated_time_type estimated_time;
  using _recommended_next_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recommended_next_state_type recommended_next_state;
  using _restrictions_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _restrictions_type restrictions;

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
  Type & set__recovery_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recovery_state = _arg;
    return *this;
  }
  Type & set__is_safe_to_proceed(
    const bool & _arg)
  {
    this->is_safe_to_proceed = _arg;
    return *this;
  }
  Type & set__remaining_steps(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->remaining_steps = _arg;
    return *this;
  }
  Type & set__verified_systems(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->verified_systems = _arg;
    return *this;
  }
  Type & set__failed_systems(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->failed_systems = _arg;
    return *this;
  }
  Type & set__estimated_time(
    const double & _arg)
  {
    this->estimated_time = _arg;
    return *this;
  }
  Type & set__recommended_next_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recommended_next_state = _arg;
    return *this;
  }
  Type & set__restrictions(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->restrictions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Response
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Response
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecoverFromSafety_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->recovery_state != other.recovery_state) {
      return false;
    }
    if (this->is_safe_to_proceed != other.is_safe_to_proceed) {
      return false;
    }
    if (this->remaining_steps != other.remaining_steps) {
      return false;
    }
    if (this->verified_systems != other.verified_systems) {
      return false;
    }
    if (this->failed_systems != other.failed_systems) {
      return false;
    }
    if (this->estimated_time != other.estimated_time) {
      return false;
    }
    if (this->recommended_next_state != other.recommended_next_state) {
      return false;
    }
    if (this->restrictions != other.restrictions) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecoverFromSafety_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecoverFromSafety_Response_

// alias to use template instance with default allocator
using RecoverFromSafety_Response =
  autonomy_interfaces::srv::RecoverFromSafety_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RecoverFromSafety_Event_
{
  using Type = RecoverFromSafety_Event_<ContainerAllocator>;

  explicit RecoverFromSafety_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit RecoverFromSafety_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::RecoverFromSafety_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::RecoverFromSafety_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Event
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__RecoverFromSafety_Event
    std::shared_ptr<autonomy_interfaces::srv::RecoverFromSafety_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecoverFromSafety_Event_ & other) const
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
  bool operator!=(const RecoverFromSafety_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecoverFromSafety_Event_

// alias to use template instance with default allocator
using RecoverFromSafety_Event =
  autonomy_interfaces::srv::RecoverFromSafety_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct RecoverFromSafety
{
  using Request = autonomy_interfaces::srv::RecoverFromSafety_Request;
  using Response = autonomy_interfaces::srv::RecoverFromSafety_Response;
  using Event = autonomy_interfaces::srv::RecoverFromSafety_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_HPP_
