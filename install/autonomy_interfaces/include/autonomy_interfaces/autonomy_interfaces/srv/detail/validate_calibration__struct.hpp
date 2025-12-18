// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Request __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Request __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ValidateCalibration_Request_
{
  using Type = ValidateCalibration_Request_<ContainerAllocator>;

  explicit ValidateCalibration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->calibration_file = "";
    }
  }

  explicit ValidateCalibration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : calibration_file(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->calibration_file = "";
    }
  }

  // field types and members
  using _calibration_file_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _calibration_file_type calibration_file;
  using _test_images_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _test_images_type test_images;

  // setters for named parameter idiom
  Type & set__calibration_file(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->calibration_file = _arg;
    return *this;
  }
  Type & set__test_images(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->test_images = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Request
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Request
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ValidateCalibration_Request_ & other) const
  {
    if (this->calibration_file != other.calibration_file) {
      return false;
    }
    if (this->test_images != other.test_images) {
      return false;
    }
    return true;
  }
  bool operator!=(const ValidateCalibration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ValidateCalibration_Request_

// alias to use template instance with default allocator
using ValidateCalibration_Request =
  autonomy_interfaces::srv::ValidateCalibration_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces


#ifndef _WIN32
# define DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Response __attribute__((deprecated))
#else
# define DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Response __declspec(deprecated)
#endif

namespace autonomy_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ValidateCalibration_Response_
{
  using Type = ValidateCalibration_Response_<ContainerAllocator>;

  explicit ValidateCalibration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reprojection_error = 0.0f;
      this->quality_assessment = "";
      this->recommendations = "";
      this->error_message = "";
    }
  }

  explicit ValidateCalibration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : quality_assessment(_alloc),
    recommendations(_alloc),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reprojection_error = 0.0f;
      this->quality_assessment = "";
      this->recommendations = "";
      this->error_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _reprojection_error_type =
    float;
  _reprojection_error_type reprojection_error;
  using _quality_assessment_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _quality_assessment_type quality_assessment;
  using _recommendations_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recommendations_type recommendations;
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
  Type & set__reprojection_error(
    const float & _arg)
  {
    this->reprojection_error = _arg;
    return *this;
  }
  Type & set__quality_assessment(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->quality_assessment = _arg;
    return *this;
  }
  Type & set__recommendations(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recommendations = _arg;
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
    autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Response
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonomy_interfaces__srv__ValidateCalibration_Response
    std::shared_ptr<autonomy_interfaces::srv::ValidateCalibration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ValidateCalibration_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->reprojection_error != other.reprojection_error) {
      return false;
    }
    if (this->quality_assessment != other.quality_assessment) {
      return false;
    }
    if (this->recommendations != other.recommendations) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ValidateCalibration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ValidateCalibration_Response_

// alias to use template instance with default allocator
using ValidateCalibration_Response =
  autonomy_interfaces::srv::ValidateCalibration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autonomy_interfaces

namespace autonomy_interfaces
{

namespace srv
{

struct ValidateCalibration
{
  using Request = autonomy_interfaces::srv::ValidateCalibration_Request;
  using Response = autonomy_interfaces::srv::ValidateCalibration_Response;
};

}  // namespace srv

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_HPP_
