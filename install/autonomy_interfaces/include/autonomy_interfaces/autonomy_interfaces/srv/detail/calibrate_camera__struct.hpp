// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/calibrate_camera.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCamera_Request_
{
  using Type = CalibrateCamera_Request_<ContainerAllocator>;

  explicit CalibrateCamera_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->image_directory = "";
      this->board_type = "";
      this->squares_x = 0l;
      this->squares_y = 0l;
      this->square_size = 0.0f;
      this->marker_size = 0.0f;
    }
  }

  explicit CalibrateCamera_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image_directory(_alloc),
    board_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->image_directory = "";
      this->board_type = "";
      this->squares_x = 0l;
      this->squares_y = 0l;
      this->square_size = 0.0f;
      this->marker_size = 0.0f;
    }
  }

  // field types and members
  using _image_directory_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _image_directory_type image_directory;
  using _board_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _board_type_type board_type;
  using _squares_x_type =
    int32_t;
  _squares_x_type squares_x;
  using _squares_y_type =
    int32_t;
  _squares_y_type squares_y;
  using _square_size_type =
    float;
  _square_size_type square_size;
  using _marker_size_type =
    float;
  _marker_size_type marker_size;

  // setters for named parameter idiom
  Type & set__image_directory(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->image_directory = _arg;
    return *this;
  }
  Type & set__board_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->board_type = _arg;
    return *this;
  }
  Type & set__squares_x(
    const int32_t & _arg)
  {
    this->squares_x = _arg;
    return *this;
  }
  Type & set__squares_y(
    const int32_t & _arg)
  {
    this->squares_y = _arg;
    return *this;
  }
  Type & set__square_size(
    const float & _arg)
  {
    this->square_size = _arg;
    return *this;
  }
  Type & set__marker_size(
    const float & _arg)
  {
    this->marker_size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Request
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Request
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCamera_Request_ & other) const
  {
    if (this->image_directory != other.image_directory) {
      return false;
    }
    if (this->board_type != other.board_type) {
      return false;
    }
    if (this->squares_x != other.squares_x) {
      return false;
    }
    if (this->squares_y != other.squares_y) {
      return false;
    }
    if (this->square_size != other.square_size) {
      return false;
    }
    if (this->marker_size != other.marker_size) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCamera_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCamera_Request_

// alias to use template instance with default allocator
using CalibrateCamera_Request =
  autonomy_interfaces::srv::CalibrateCamera_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCamera_Response_
{
  using Type = CalibrateCamera_Response_<ContainerAllocator>;

  explicit CalibrateCamera_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_file = "";
      this->calibration_summary = "";
      this->error_message = "";
    }
  }

  explicit CalibrateCamera_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result_file(_alloc),
    calibration_summary(_alloc),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_file = "";
      this->calibration_summary = "";
      this->error_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _result_file_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_file_type result_file;
  using _calibration_summary_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _calibration_summary_type calibration_summary;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__result_file(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result_file = _arg;
    return *this;
  }
  Type & set__calibration_summary(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->calibration_summary = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Response
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Response
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCamera_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result_file != other.result_file) {
      return false;
    }
    if (this->calibration_summary != other.calibration_summary) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCamera_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCamera_Response_

// alias to use template instance with default allocator
using CalibrateCamera_Response =
  autonomy_interfaces::srv::CalibrateCamera_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCamera_Event_
{
  using Type = CalibrateCamera_Event_<ContainerAllocator>;

  explicit CalibrateCamera_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CalibrateCamera_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::CalibrateCamera_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::CalibrateCamera_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Event
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__CalibrateCamera_Event
    std::shared_ptr<autonomy_interfaces::srv::CalibrateCamera_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCamera_Event_ & other) const
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
  bool operator!=(const CalibrateCamera_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCamera_Event_

// alias to use template instance with default allocator
using CalibrateCamera_Event =
  autonomy_interfaces::srv::CalibrateCamera_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct CalibrateCamera
{
  using Request = autonomy_interfaces::srv::CalibrateCamera_Request;
  using Response = autonomy_interfaces::srv::CalibrateCamera_Response;
  using Event = autonomy_interfaces::srv::CalibrateCamera_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_
