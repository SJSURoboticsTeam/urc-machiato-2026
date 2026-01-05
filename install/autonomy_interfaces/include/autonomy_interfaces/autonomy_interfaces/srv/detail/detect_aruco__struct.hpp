// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/detect_aruco.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectAruco_Request_
{
  using Type = DetectAruco_Request_<ContainerAllocator>;

  explicit DetectAruco_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_timeout = 0.0f;
      this->require_distance_estimate = false;
      this->max_detection_distance = 0.0f;
      this->calculate_alignment = false;
      this->target_depth = 0.0f;
      this->mission_type = "";
    }
  }

  explicit DetectAruco_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_timeout = 0.0f;
      this->require_distance_estimate = false;
      this->max_detection_distance = 0.0f;
      this->calculate_alignment = false;
      this->target_depth = 0.0f;
      this->mission_type = "";
    }
  }

  // field types and members
  using _target_tag_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _target_tag_ids_type target_tag_ids;
  using _detection_timeout_type =
    float;
  _detection_timeout_type detection_timeout;
  using _require_distance_estimate_type =
    bool;
  _require_distance_estimate_type require_distance_estimate;
  using _max_detection_distance_type =
    float;
  _max_detection_distance_type max_detection_distance;
  using _calculate_alignment_type =
    bool;
  _calculate_alignment_type calculate_alignment;
  using _target_depth_type =
    float;
  _target_depth_type target_depth;
  using _mission_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type_type mission_type;

  // setters for named parameter idiom
  Type & set__target_tag_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->target_tag_ids = _arg;
    return *this;
  }
  Type & set__detection_timeout(
    const float & _arg)
  {
    this->detection_timeout = _arg;
    return *this;
  }
  Type & set__require_distance_estimate(
    const bool & _arg)
  {
    this->require_distance_estimate = _arg;
    return *this;
  }
  Type & set__max_detection_distance(
    const float & _arg)
  {
    this->max_detection_distance = _arg;
    return *this;
  }
  Type & set__calculate_alignment(
    const bool & _arg)
  {
    this->calculate_alignment = _arg;
    return *this;
  }
  Type & set__target_depth(
    const float & _arg)
  {
    this->target_depth = _arg;
    return *this;
  }
  Type & set__mission_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Request
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Request
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectAruco_Request_ & other) const
  {
    if (this->target_tag_ids != other.target_tag_ids) {
      return false;
    }
    if (this->detection_timeout != other.detection_timeout) {
      return false;
    }
    if (this->require_distance_estimate != other.require_distance_estimate) {
      return false;
    }
    if (this->max_detection_distance != other.max_detection_distance) {
      return false;
    }
    if (this->calculate_alignment != other.calculate_alignment) {
      return false;
    }
    if (this->target_depth != other.target_depth) {
      return false;
    }
    if (this->mission_type != other.mission_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectAruco_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectAruco_Request_

// alias to use template instance with default allocator
using DetectAruco_Request =
  autonomy_interfaces::srv::DetectAruco_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'tag_positions'
// Member 'alignment_center'
// Member 'arm_target_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'detection_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'alignment_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectAruco_Response_
{
  using Type = DetectAruco_Response_<ContainerAllocator>;

  explicit DetectAruco_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : detection_time(_init),
    alignment_center(_init),
    alignment_orientation(_init),
    arm_target_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->alignment_available = false;
      this->alignment_quality = 0.0f;
    }
  }

  explicit DetectAruco_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    detection_time(_alloc, _init),
    alignment_center(_alloc, _init),
    alignment_orientation(_alloc, _init),
    arm_target_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->alignment_available = false;
      this->alignment_quality = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _detected_tag_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _detected_tag_ids_type detected_tag_ids;
  using _tag_positions_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _tag_positions_type tag_positions;
  using _tag_distances_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _tag_distances_type tag_distances;
  using _tag_angles_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _tag_angles_type tag_angles;
  using _detection_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _detection_time_type detection_time;
  using _alignment_available_type =
    bool;
  _alignment_available_type alignment_available;
  using _alignment_center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _alignment_center_type alignment_center;
  using _alignment_orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _alignment_orientation_type alignment_orientation;
  using _arm_target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _arm_target_position_type arm_target_position;
  using _alignment_quality_type =
    float;
  _alignment_quality_type alignment_quality;
  using _alignment_warnings_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _alignment_warnings_type alignment_warnings;

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
  Type & set__detected_tag_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->detected_tag_ids = _arg;
    return *this;
  }
  Type & set__tag_positions(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->tag_positions = _arg;
    return *this;
  }
  Type & set__tag_distances(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->tag_distances = _arg;
    return *this;
  }
  Type & set__tag_angles(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->tag_angles = _arg;
    return *this;
  }
  Type & set__detection_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->detection_time = _arg;
    return *this;
  }
  Type & set__alignment_available(
    const bool & _arg)
  {
    this->alignment_available = _arg;
    return *this;
  }
  Type & set__alignment_center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->alignment_center = _arg;
    return *this;
  }
  Type & set__alignment_orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->alignment_orientation = _arg;
    return *this;
  }
  Type & set__arm_target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->arm_target_position = _arg;
    return *this;
  }
  Type & set__alignment_quality(
    const float & _arg)
  {
    this->alignment_quality = _arg;
    return *this;
  }
  Type & set__alignment_warnings(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->alignment_warnings = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Response
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Response
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectAruco_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->detected_tag_ids != other.detected_tag_ids) {
      return false;
    }
    if (this->tag_positions != other.tag_positions) {
      return false;
    }
    if (this->tag_distances != other.tag_distances) {
      return false;
    }
    if (this->tag_angles != other.tag_angles) {
      return false;
    }
    if (this->detection_time != other.detection_time) {
      return false;
    }
    if (this->alignment_available != other.alignment_available) {
      return false;
    }
    if (this->alignment_center != other.alignment_center) {
      return false;
    }
    if (this->alignment_orientation != other.alignment_orientation) {
      return false;
    }
    if (this->arm_target_position != other.arm_target_position) {
      return false;
    }
    if (this->alignment_quality != other.alignment_quality) {
      return false;
    }
    if (this->alignment_warnings != other.alignment_warnings) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectAruco_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectAruco_Response_

// alias to use template instance with default allocator
using DetectAruco_Response =
  autonomy_interfaces::srv::DetectAruco_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectAruco_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectAruco_Event_
{
  using Type = DetectAruco_Event_<ContainerAllocator>;

  explicit DetectAruco_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit DetectAruco_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectAruco_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectAruco_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Event
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectAruco_Event
    std::shared_ptr<autonomy_interfaces::srv::DetectAruco_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectAruco_Event_ & other) const
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
  bool operator!=(const DetectAruco_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectAruco_Event_

// alias to use template instance with default allocator
using DetectAruco_Event =
  autonomy_interfaces::srv::DetectAruco_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct DetectAruco
{
  using Request = autonomy_interfaces::srv::DetectAruco_Request;
  using Response = autonomy_interfaces::srv::DetectAruco_Response;
  using Event = autonomy_interfaces::srv::DetectAruco_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_ARUCO__STRUCT_HPP_
