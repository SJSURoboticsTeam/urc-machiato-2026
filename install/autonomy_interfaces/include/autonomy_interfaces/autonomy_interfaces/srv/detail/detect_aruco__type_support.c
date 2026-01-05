// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/detect_aruco__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"
#include "autonomy_interfaces/srv/detail/detect_aruco__struct.h"


// Include directives for member types
// Member `target_tag_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `mission_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__DetectAruco_Request__init(message_memory);
}

void autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__DetectAruco_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__size_function__DetectAruco_Request__target_tag_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Request__target_tag_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_function__DetectAruco_Request__target_tag_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Request__target_tag_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Request__target_tag_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Request__target_tag_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_function__DetectAruco_Request__target_tag_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Request__target_tag_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_member_array[7] = {
  {
    "target_tag_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, target_tag_ids),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__size_function__DetectAruco_Request__target_tag_ids,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Request__target_tag_ids,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__get_function__DetectAruco_Request__target_tag_ids,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Request__target_tag_ids,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Request__target_tag_ids,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Request__target_tag_ids  // resize(index) function pointer
  },
  {
    "detection_timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, detection_timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "require_distance_estimate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, require_distance_estimate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_detection_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, max_detection_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "calculate_alignment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, calculate_alignment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, target_depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Request, mission_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "DetectAruco_Request",  // message name
  7,  // number of fields
  sizeof(autonomy_interfaces__srv__DetectAruco_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_member_array,  // message members
  autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectAruco_Request__get_type_hash,
  &autonomy_interfaces__srv__DetectAruco_Request__get_type_description,
  &autonomy_interfaces__srv__DetectAruco_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Request)() {
  if (!autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__struct.h"


// Include directives for member types
// Member `message`
// Member `alignment_warnings`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `detected_tag_ids`
// Member `tag_distances`
// Member `tag_angles`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tag_positions`
// Member `alignment_center`
// Member `arm_target_position`
#include "geometry_msgs/msg/point.h"
// Member `tag_positions`
// Member `alignment_center`
// Member `arm_target_position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `detection_time`
#include "builtin_interfaces/msg/time.h"
// Member `detection_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `alignment_orientation`
#include "geometry_msgs/msg/quaternion.h"
// Member `alignment_orientation`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__DetectAruco_Response__init(message_memory);
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__DetectAruco_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__detected_tag_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__detected_tag_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__detected_tag_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__detected_tag_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__detected_tag_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__detected_tag_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__detected_tag_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__detected_tag_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_positions(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_positions(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_positions(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_positions(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_positions(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_positions(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_distances(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_distances(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_distances(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_distances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_distances(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_distances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_distances(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_distances(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_angles(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_angles(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_angles(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_angles(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_angles(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_angles(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__alignment_warnings(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__alignment_warnings(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__alignment_warnings(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__alignment_warnings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__alignment_warnings(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__alignment_warnings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__alignment_warnings(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__alignment_warnings(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[13] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detected_tag_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, detected_tag_ids),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__detected_tag_ids,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__detected_tag_ids,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__detected_tag_ids,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__detected_tag_ids,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__detected_tag_ids,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__detected_tag_ids  // resize(index) function pointer
  },
  {
    "tag_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, tag_positions),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_positions,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_positions,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_positions,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_positions,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_positions,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_positions  // resize(index) function pointer
  },
  {
    "tag_distances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, tag_distances),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_distances,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_distances,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_distances,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_distances,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_distances,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_distances  // resize(index) function pointer
  },
  {
    "tag_angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, tag_angles),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__tag_angles,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__tag_angles,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__tag_angles,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__tag_angles,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__tag_angles,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__tag_angles  // resize(index) function pointer
  },
  {
    "detection_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, detection_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_available",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, alignment_available),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_center",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, alignment_center),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, alignment_orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "arm_target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, arm_target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_quality",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, alignment_quality),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_warnings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Response, alignment_warnings),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__size_function__DetectAruco_Response__alignment_warnings,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Response__alignment_warnings,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__get_function__DetectAruco_Response__alignment_warnings,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Response__alignment_warnings,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Response__alignment_warnings,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Response__alignment_warnings  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "DetectAruco_Response",  // message name
  13,  // number of fields
  sizeof(autonomy_interfaces__srv__DetectAruco_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array,  // message members
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectAruco_Response__get_type_hash,
  &autonomy_interfaces__srv__DetectAruco_Response__get_type_description,
  &autonomy_interfaces__srv__DetectAruco_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Response)() {
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/detect_aruco.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__DetectAruco_Event__init(message_memory);
}

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__DetectAruco_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__size_function__DetectAruco_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__DetectAruco_Request__Sequence * member =
    (const autonomy_interfaces__srv__DetectAruco_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__DetectAruco_Request__Sequence * member =
    (const autonomy_interfaces__srv__DetectAruco_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__DetectAruco_Request__Sequence * member =
    (autonomy_interfaces__srv__DetectAruco_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__DetectAruco_Request * item =
    ((const autonomy_interfaces__srv__DetectAruco_Request *)
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__request(untyped_member, index));
  autonomy_interfaces__srv__DetectAruco_Request * value =
    (autonomy_interfaces__srv__DetectAruco_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__DetectAruco_Request * item =
    ((autonomy_interfaces__srv__DetectAruco_Request *)
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__DetectAruco_Request * value =
    (const autonomy_interfaces__srv__DetectAruco_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__DetectAruco_Request__Sequence * member =
    (autonomy_interfaces__srv__DetectAruco_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__DetectAruco_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__DetectAruco_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__size_function__DetectAruco_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__DetectAruco_Response__Sequence * member =
    (const autonomy_interfaces__srv__DetectAruco_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__DetectAruco_Response__Sequence * member =
    (const autonomy_interfaces__srv__DetectAruco_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__DetectAruco_Response__Sequence * member =
    (autonomy_interfaces__srv__DetectAruco_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__DetectAruco_Response * item =
    ((const autonomy_interfaces__srv__DetectAruco_Response *)
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__response(untyped_member, index));
  autonomy_interfaces__srv__DetectAruco_Response * value =
    (autonomy_interfaces__srv__DetectAruco_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__DetectAruco_Response * item =
    ((autonomy_interfaces__srv__DetectAruco_Response *)
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__DetectAruco_Response * value =
    (const autonomy_interfaces__srv__DetectAruco_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__DetectAruco_Response__Sequence * member =
    (autonomy_interfaces__srv__DetectAruco_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__DetectAruco_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__DetectAruco_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__size_function__DetectAruco_Event__request,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(autonomy_interfaces__srv__DetectAruco_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__size_function__DetectAruco_Event__response,  // size() function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_const_function__DetectAruco_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__get_function__DetectAruco_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__fetch_function__DetectAruco_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__assign_function__DetectAruco_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__resize_function__DetectAruco_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "DetectAruco_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__DetectAruco_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_member_array,  // message members
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectAruco_Event__get_type_hash,
  &autonomy_interfaces__srv__DetectAruco_Event__get_type_description,
  &autonomy_interfaces__srv__DetectAruco_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Event)() {
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Request)();
  autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Response)();
  if (!autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/detect_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "DetectAruco",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__DetectAruco_Request__rosidl_typesupport_introspection_c__DetectAruco_Request_message_type_support_handle,
  &autonomy_interfaces__srv__DetectAruco_Response__rosidl_typesupport_introspection_c__DetectAruco_Response_message_type_support_handle,
  &autonomy_interfaces__srv__DetectAruco_Event__rosidl_typesupport_introspection_c__DetectAruco_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    DetectAruco
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    DetectAruco
  ),
  &autonomy_interfaces__srv__DetectAruco__get_type_hash,
  &autonomy_interfaces__srv__DetectAruco__get_type_description,
  &autonomy_interfaces__srv__DetectAruco__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco)(void) {
  if (!autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, DetectAruco_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__detect_aruco__rosidl_typesupport_introspection_c__DetectAruco_service_type_support_handle;
}
