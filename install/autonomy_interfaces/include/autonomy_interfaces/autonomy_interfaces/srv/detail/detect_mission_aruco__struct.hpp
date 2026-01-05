// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/detect_mission_aruco.hpp"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectMissionAruco_Request_
{
  using Type = DetectMissionAruco_Request_<ContainerAllocator>;

  explicit DetectMissionAruco_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_type = "";
      this->detection_timeout = 0.0f;
      this->target_depth = 0.0f;
      this->max_detection_distance = 0.0f;
      this->require_all_tags = false;
      this->min_alignment_quality = 0.0f;
    }
  }

  explicit DetectMissionAruco_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_type = "";
      this->detection_timeout = 0.0f;
      this->target_depth = 0.0f;
      this->max_detection_distance = 0.0f;
      this->require_all_tags = false;
      this->min_alignment_quality = 0.0f;
    }
  }

  // field types and members
  using _mission_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type_type mission_type;
  using _required_tag_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _required_tag_ids_type required_tag_ids;
  using _optional_tag_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _optional_tag_ids_type optional_tag_ids;
  using _detection_timeout_type =
    float;
  _detection_timeout_type detection_timeout;
  using _target_depth_type =
    float;
  _target_depth_type target_depth;
  using _max_detection_distance_type =
    float;
  _max_detection_distance_type max_detection_distance;
  using _require_all_tags_type =
    bool;
  _require_all_tags_type require_all_tags;
  using _min_alignment_quality_type =
    float;
  _min_alignment_quality_type min_alignment_quality;

  // setters for named parameter idiom
  Type & set__mission_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_type = _arg;
    return *this;
  }
  Type & set__required_tag_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->required_tag_ids = _arg;
    return *this;
  }
  Type & set__optional_tag_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->optional_tag_ids = _arg;
    return *this;
  }
  Type & set__detection_timeout(
    const float & _arg)
  {
    this->detection_timeout = _arg;
    return *this;
  }
  Type & set__target_depth(
    const float & _arg)
  {
    this->target_depth = _arg;
    return *this;
  }
  Type & set__max_detection_distance(
    const float & _arg)
  {
    this->max_detection_distance = _arg;
    return *this;
  }
  Type & set__require_all_tags(
    const bool & _arg)
  {
    this->require_all_tags = _arg;
    return *this;
  }
  Type & set__min_alignment_quality(
    const float & _arg)
  {
    this->min_alignment_quality = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Request
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Request
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectMissionAruco_Request_ & other) const
  {
    if (this->mission_type != other.mission_type) {
      return false;
    }
    if (this->required_tag_ids != other.required_tag_ids) {
      return false;
    }
    if (this->optional_tag_ids != other.optional_tag_ids) {
      return false;
    }
    if (this->detection_timeout != other.detection_timeout) {
      return false;
    }
    if (this->target_depth != other.target_depth) {
      return false;
    }
    if (this->max_detection_distance != other.max_detection_distance) {
      return false;
    }
    if (this->require_all_tags != other.require_all_tags) {
      return false;
    }
    if (this->min_alignment_quality != other.min_alignment_quality) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectMissionAruco_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectMissionAruco_Request_

// alias to use template instance with default allocator
using DetectMissionAruco_Request =
  autonomy_interfaces::srv::DetectMissionAruco_Request_<std::allocator<void>>;

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
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectMissionAruco_Response_
{
  using Type = DetectMissionAruco_Response_<ContainerAllocator>;

  explicit DetectMissionAruco_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->mission_type = "";
      this->alignment_available = false;
      this->alignment_quality = 0.0f;
      this->mission_ready = false;
    }
  }

  explicit DetectMissionAruco_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    mission_type(_alloc),
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
      this->mission_type = "";
      this->alignment_available = false;
      this->alignment_quality = 0.0f;
      this->mission_ready = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _mission_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type_type mission_type;
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
  using _alignment_errors_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _alignment_errors_type alignment_errors;
  using _mission_ready_type =
    bool;
  _mission_ready_type mission_ready;
  using _missing_required_tags_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _missing_required_tags_type missing_required_tags;
  using _detected_optional_tags_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _detected_optional_tags_type detected_optional_tags;
  using _alignment_warnings_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _alignment_warnings_type alignment_warnings;
  using _mission_recommendations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _mission_recommendations_type mission_recommendations;

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
  Type & set__mission_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_type = _arg;
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
  Type & set__alignment_errors(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->alignment_errors = _arg;
    return *this;
  }
  Type & set__mission_ready(
    const bool & _arg)
  {
    this->mission_ready = _arg;
    return *this;
  }
  Type & set__missing_required_tags(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->missing_required_tags = _arg;
    return *this;
  }
  Type & set__detected_optional_tags(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->detected_optional_tags = _arg;
    return *this;
  }
  Type & set__alignment_warnings(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->alignment_warnings = _arg;
    return *this;
  }
  Type & set__mission_recommendations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->mission_recommendations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Response
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Response
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectMissionAruco_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->mission_type != other.mission_type) {
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
    if (this->alignment_errors != other.alignment_errors) {
      return false;
    }
    if (this->mission_ready != other.mission_ready) {
      return false;
    }
    if (this->missing_required_tags != other.missing_required_tags) {
      return false;
    }
    if (this->detected_optional_tags != other.detected_optional_tags) {
      return false;
    }
    if (this->alignment_warnings != other.alignment_warnings) {
      return false;
    }
    if (this->mission_recommendations != other.mission_recommendations) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectMissionAruco_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectMissionAruco_Response_

// alias to use template instance with default allocator
using DetectMissionAruco_Response =
  autonomy_interfaces::srv::DetectMissionAruco_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Event __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Event __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectMissionAruco_Event_
{
  using Type = DetectMissionAruco_Event_<ContainerAllocator>;

  explicit DetectMissionAruco_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit DetectMissionAruco_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectMissionAruco_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonomy_interfaces::srv::DetectMissionAruco_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Event
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__DetectMissionAruco_Event
    std::shared_ptr<autonomy_interfaces::srv::DetectMissionAruco_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectMissionAruco_Event_ & other) const
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
  bool operator!=(const DetectMissionAruco_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectMissionAruco_Event_

// alias to use template instance with default allocator
using DetectMissionAruco_Event =
  autonomy_interfaces::srv::DetectMissionAruco_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct DetectMissionAruco
{
  using Request = autonomy_interfaces::srv::DetectMissionAruco_Request;
  using Response = autonomy_interfaces::srv::DetectMissionAruco_Response;
  using Event = autonomy_interfaces::srv::DetectMissionAruco_Event;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__DETECT_MISSION_ARUCO__STRUCT_HPP_
