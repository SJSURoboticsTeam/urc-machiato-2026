// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'waypoints'
// Member 'typing_location'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConfigureMission_Request_
{
  using Type = ConfigureMission_Request_<ContainerAllocator>;

  explicit ConfigureMission_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : typing_location(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_name = "";
      this->time_limit = 0.0f;
      this->waypoint_timeout = 0.0f;
      this->max_linear_velocity = 0.0f;
      this->max_angular_velocity = 0.0f;
      this->waypoint_approach_tolerance = 0.0f;
      this->typing_text = "";
      this->terrain_type = "";
      this->max_incline = 0.0f;
    }
  }

  explicit ConfigureMission_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission_name(_alloc),
    typing_text(_alloc),
    typing_location(_alloc, _init),
    terrain_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_name = "";
      this->time_limit = 0.0f;
      this->waypoint_timeout = 0.0f;
      this->max_linear_velocity = 0.0f;
      this->max_angular_velocity = 0.0f;
      this->waypoint_approach_tolerance = 0.0f;
      this->typing_text = "";
      this->terrain_type = "";
      this->max_incline = 0.0f;
    }
  }

  // field types and members
  using _mission_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_name_type mission_name;
  using _objectives_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _objectives_type objectives;
  using _waypoints_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _waypoints_type waypoints;
  using _waypoint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _waypoint_names_type waypoint_names;
  using _precision_required_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _precision_required_type precision_required;
  using _waypoint_tolerances_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _waypoint_tolerances_type waypoint_tolerances;
  using _time_limit_type =
    float;
  _time_limit_type time_limit;
  using _waypoint_timeout_type =
    float;
  _waypoint_timeout_type waypoint_timeout;
  using _max_linear_velocity_type =
    float;
  _max_linear_velocity_type max_linear_velocity;
  using _max_angular_velocity_type =
    float;
  _max_angular_velocity_type max_angular_velocity;
  using _waypoint_approach_tolerance_type =
    float;
  _waypoint_approach_tolerance_type waypoint_approach_tolerance;
  using _typing_text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _typing_text_type typing_text;
  using _typing_location_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _typing_location_type typing_location;
  using _terrain_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _terrain_type_type terrain_type;
  using _max_incline_type =
    float;
  _max_incline_type max_incline;

  // setters for named parameter idiom
  Type & set__mission_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_name = _arg;
    return *this;
  }
  Type & set__objectives(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->objectives = _arg;
    return *this;
  }
  Type & set__waypoints(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->waypoints = _arg;
    return *this;
  }
  Type & set__waypoint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->waypoint_names = _arg;
    return *this;
  }
  Type & set__precision_required(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->precision_required = _arg;
    return *this;
  }
  Type & set__waypoint_tolerances(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->waypoint_tolerances = _arg;
    return *this;
  }
  Type & set__time_limit(
    const float & _arg)
  {
    this->time_limit = _arg;
    return *this;
  }
  Type & set__waypoint_timeout(
    const float & _arg)
  {
    this->waypoint_timeout = _arg;
    return *this;
  }
  Type & set__max_linear_velocity(
    const float & _arg)
  {
    this->max_linear_velocity = _arg;
    return *this;
  }
  Type & set__max_angular_velocity(
    const float & _arg)
  {
    this->max_angular_velocity = _arg;
    return *this;
  }
  Type & set__waypoint_approach_tolerance(
    const float & _arg)
  {
    this->waypoint_approach_tolerance = _arg;
    return *this;
  }
  Type & set__typing_text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->typing_text = _arg;
    return *this;
  }
  Type & set__typing_location(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->typing_location = _arg;
    return *this;
  }
  Type & set__terrain_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->terrain_type = _arg;
    return *this;
  }
  Type & set__max_incline(
    const float & _arg)
  {
    this->max_incline = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Request
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Request
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConfigureMission_Request_ & other) const
  {
    if (this->mission_name != other.mission_name) {
      return false;
    }
    if (this->objectives != other.objectives) {
      return false;
    }
    if (this->waypoints != other.waypoints) {
      return false;
    }
    if (this->waypoint_names != other.waypoint_names) {
      return false;
    }
    if (this->precision_required != other.precision_required) {
      return false;
    }
    if (this->waypoint_tolerances != other.waypoint_tolerances) {
      return false;
    }
    if (this->time_limit != other.time_limit) {
      return false;
    }
    if (this->waypoint_timeout != other.waypoint_timeout) {
      return false;
    }
    if (this->max_linear_velocity != other.max_linear_velocity) {
      return false;
    }
    if (this->max_angular_velocity != other.max_angular_velocity) {
      return false;
    }
    if (this->waypoint_approach_tolerance != other.waypoint_approach_tolerance) {
      return false;
    }
    if (this->typing_text != other.typing_text) {
      return false;
    }
    if (this->typing_location != other.typing_location) {
      return false;
    }
    if (this->terrain_type != other.terrain_type) {
      return false;
    }
    if (this->max_incline != other.max_incline) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConfigureMission_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConfigureMission_Request_

// alias to use template instance with default allocator
using ConfigureMission_Request =
  autonomy_interfaces::srv::ConfigureMission_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConfigureMission_Response_
{
  using Type = ConfigureMission_Response_<ContainerAllocator>;

  explicit ConfigureMission_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->mission_id = "";
      this->estimated_duration = 0.0f;
      this->total_waypoints = 0l;
    }
  }

  explicit ConfigureMission_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    mission_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->mission_id = "";
      this->estimated_duration = 0.0f;
      this->total_waypoints = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _mission_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_id_type mission_id;
  using _estimated_duration_type =
    float;
  _estimated_duration_type estimated_duration;
  using _total_waypoints_type =
    int32_t;
  _total_waypoints_type total_waypoints;
  using _configured_objectives_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _configured_objectives_type configured_objectives;

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
  Type & set__mission_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_id = _arg;
    return *this;
  }
  Type & set__estimated_duration(
    const float & _arg)
  {
    this->estimated_duration = _arg;
    return *this;
  }
  Type & set__total_waypoints(
    const int32_t & _arg)
  {
    this->total_waypoints = _arg;
    return *this;
  }
  Type & set__configured_objectives(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->configured_objectives = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Response
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__ConfigureMission_Response
    std::shared_ptr<autonomy_interfaces::srv::ConfigureMission_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConfigureMission_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->mission_id != other.mission_id) {
      return false;
    }
    if (this->estimated_duration != other.estimated_duration) {
      return false;
    }
    if (this->total_waypoints != other.total_waypoints) {
      return false;
    }
    if (this->configured_objectives != other.configured_objectives) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConfigureMission_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConfigureMission_Response_

// alias to use template instance with default allocator
using ConfigureMission_Response =
  autonomy_interfaces::srv::ConfigureMission_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct ConfigureMission
{
  using Request = autonomy_interfaces::srv::ConfigureMission_Request;
  using Response = autonomy_interfaces::srv::ConfigureMission_Response;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CONFIGURE_MISSION__STRUCT_HPP_
