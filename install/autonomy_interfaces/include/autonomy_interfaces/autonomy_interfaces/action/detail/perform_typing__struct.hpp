// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:action/PerformTyping.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_HPP_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'typing_goal'
#include "autonomy_interfaces/msg/detail/typing_goal__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Goal __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Goal __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_Goal_
{
  using Type = PerformTyping_Goal_<ContainerAllocator>;

  explicit PerformTyping_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : typing_goal(_init)
  {
    (void)_init;
  }

  explicit PerformTyping_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : typing_goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _typing_goal_type =
    autonomy_interfaces::msg::TypingGoal_<ContainerAllocator>;
  _typing_goal_type typing_goal;

  // setters for named parameter idiom
  Type & set__typing_goal(
    const autonomy_interfaces::msg::TypingGoal_<ContainerAllocator> & _arg)
  {
    this->typing_goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Goal
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Goal
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_Goal_ & other) const
  {
    if (this->typing_goal != other.typing_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_Goal_

// alias to use template instance with default allocator
using PerformTyping_Goal =
  autonomy_interfaces::action::PerformTyping_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'current_hand_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Result __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Result __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_Result_
{
  using Type = PerformTyping_Result_<ContainerAllocator>;

  explicit PerformTyping_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_hand_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0.0f;
      this->current_character = "";
      this->characters_completed = 0l;
    }
  }

  explicit PerformTyping_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_character(_alloc),
    current_hand_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0.0f;
      this->current_character = "";
      this->characters_completed = 0l;
    }
  }

  // field types and members
  using _progress_type =
    float;
  _progress_type progress;
  using _current_character_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_character_type current_character;
  using _characters_completed_type =
    int32_t;
  _characters_completed_type characters_completed;
  using _current_hand_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_hand_pose_type current_hand_pose;

  // setters for named parameter idiom
  Type & set__progress(
    const float & _arg)
  {
    this->progress = _arg;
    return *this;
  }
  Type & set__current_character(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_character = _arg;
    return *this;
  }
  Type & set__characters_completed(
    const int32_t & _arg)
  {
    this->characters_completed = _arg;
    return *this;
  }
  Type & set__current_hand_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_hand_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Result
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Result
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_Result_ & other) const
  {
    if (this->progress != other.progress) {
      return false;
    }
    if (this->current_character != other.current_character) {
      return false;
    }
    if (this->characters_completed != other.characters_completed) {
      return false;
    }
    if (this->current_hand_pose != other.current_hand_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_Result_

// alias to use template instance with default allocator
using PerformTyping_Result =
  autonomy_interfaces::action::PerformTyping_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_Feedback __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_Feedback_
{
  using Type = PerformTyping_Feedback_<ContainerAllocator>;

  explicit PerformTyping_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->text_typed = "";
      this->characters_attempted = 0l;
      this->characters_successful = 0l;
      this->accuracy = 0.0f;
      this->total_time = 0.0f;
    }
  }

  explicit PerformTyping_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    text_typed(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->text_typed = "";
      this->characters_attempted = 0l;
      this->characters_successful = 0l;
      this->accuracy = 0.0f;
      this->total_time = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _text_typed_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_typed_type text_typed;
  using _characters_attempted_type =
    int32_t;
  _characters_attempted_type characters_attempted;
  using _characters_successful_type =
    int32_t;
  _characters_successful_type characters_successful;
  using _accuracy_type =
    float;
  _accuracy_type accuracy;
  using _total_time_type =
    float;
  _total_time_type total_time;

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
  Type & set__text_typed(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text_typed = _arg;
    return *this;
  }
  Type & set__characters_attempted(
    const int32_t & _arg)
  {
    this->characters_attempted = _arg;
    return *this;
  }
  Type & set__characters_successful(
    const int32_t & _arg)
  {
    this->characters_successful = _arg;
    return *this;
  }
  Type & set__accuracy(
    const float & _arg)
  {
    this->accuracy = _arg;
    return *this;
  }
  Type & set__total_time(
    const float & _arg)
  {
    this->total_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Feedback
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_Feedback
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_Feedback_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->text_typed != other.text_typed) {
      return false;
    }
    if (this->characters_attempted != other.characters_attempted) {
      return false;
    }
    if (this->characters_successful != other.characters_successful) {
      return false;
    }
    if (this->accuracy != other.accuracy) {
      return false;
    }
    if (this->total_time != other.total_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_Feedback_

// alias to use template instance with default allocator
using PerformTyping_Feedback =
  autonomy_interfaces::action::PerformTyping_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "autonomy_interfaces/action/detail/perform_typing__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_SendGoal_Request_
{
  using Type = PerformTyping_SendGoal_Request_<ContainerAllocator>;

  explicit PerformTyping_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PerformTyping_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const autonomy_interfaces::action::PerformTyping_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Request
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Request
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_SendGoal_Request_

// alias to use template instance with default allocator
using PerformTyping_SendGoal_Request =
  autonomy_interfaces::action::PerformTyping_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_SendGoal_Response_
{
  using Type = PerformTyping_SendGoal_Response_<ContainerAllocator>;

  explicit PerformTyping_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PerformTyping_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Response
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_SendGoal_Response
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_SendGoal_Response_

// alias to use template instance with default allocator
using PerformTyping_SendGoal_Response =
  autonomy_interfaces::action::PerformTyping_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace action
{

struct PerformTyping_SendGoal
{
  using Request = autonomy_interfaces::action::PerformTyping_SendGoal_Request;
  using Response = autonomy_interfaces::action::PerformTyping_SendGoal_Response;
};

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_GetResult_Request_
{
  using Type = PerformTyping_GetResult_Request_<ContainerAllocator>;

  explicit PerformTyping_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PerformTyping_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Request
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Request
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_GetResult_Request_

// alias to use template instance with default allocator
using PerformTyping_GetResult_Request =
  autonomy_interfaces::action::PerformTyping_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_GetResult_Response_
{
  using Type = PerformTyping_GetResult_Response_<ContainerAllocator>;

  explicit PerformTyping_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PerformTyping_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const autonomy_interfaces::action::PerformTyping_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Response
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_GetResult_Response
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_GetResult_Response_

// alias to use template instance with default allocator
using PerformTyping_GetResult_Response =
  autonomy_interfaces::action::PerformTyping_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace action
{

struct PerformTyping_GetResult
{
  using Request = autonomy_interfaces::action::PerformTyping_GetResult_Request;
  using Response = autonomy_interfaces::action::PerformTyping_GetResult_Response;
};

}  // namespace action

}  // namespace autonomy_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__action__PerformTyping_FeedbackMessage __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PerformTyping_FeedbackMessage_
{
  using Type = PerformTyping_FeedbackMessage_<ContainerAllocator>;

  explicit PerformTyping_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PerformTyping_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const autonomy_interfaces::action::PerformTyping_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_FeedbackMessage
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__action__PerformTyping_FeedbackMessage
    std::shared_ptr<autonomy_interfaces::action::PerformTyping_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformTyping_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformTyping_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformTyping_FeedbackMessage_

// alias to use template instance with default allocator
using PerformTyping_FeedbackMessage =
  autonomy_interfaces::action::PerformTyping_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace autonomy_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace autonomy_interfaces
{

namespace action
{

struct PerformTyping
{
  /// The goal message defined in the action definition.
  using Goal = autonomy_interfaces::action::PerformTyping_Goal;
  /// The result message defined in the action definition.
  using Result = autonomy_interfaces::action::PerformTyping_Result;
  /// The feedback message defined in the action definition.
  using Feedback = autonomy_interfaces::action::PerformTyping_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = autonomy_interfaces::action::PerformTyping_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = autonomy_interfaces::action::PerformTyping_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = autonomy_interfaces::action::PerformTyping_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PerformTyping PerformTyping;

}  // namespace action

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_HPP_
