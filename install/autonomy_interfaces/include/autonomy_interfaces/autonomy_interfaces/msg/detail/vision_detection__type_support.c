// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/msg/detail/vision_detection__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/msg/detail/vision_detection__functions.h"
#include "autonomy_interfaces/msg/detail/vision_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `class_name`
// Member `detector_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `size`
#include "geometry_msgs/msg/vector3.h"
// Member `size`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__msg__VisionDetection__init(message_memory);
}

void autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_fini_function(void * message_memory)
{
  autonomy_interfaces__msg__VisionDetection__fini(message_memory);
}

size_t autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__size_function__VisionDetection__keypoints(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_const_function__VisionDetection__keypoints(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_function__VisionDetection__keypoints(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__fetch_function__VisionDetection__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_const_function__VisionDetection__keypoints(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__assign_function__VisionDetection__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_function__VisionDetection__keypoints(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__resize_function__VisionDetection__keypoints(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "class_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, class_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "class_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, class_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, keypoints),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__size_function__VisionDetection__keypoints,  // size() function pointer
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_const_function__VisionDetection__keypoints,  // get_const(index) function pointer
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__get_function__VisionDetection__keypoints,  // get(index) function pointer
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__fetch_function__VisionDetection__keypoints,  // fetch(index, &value) function pointer
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__assign_function__VisionDetection__keypoints,  // assign(index, value) function pointer
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__resize_function__VisionDetection__keypoints  // resize(index) function pointer
  },
  {
    "detector_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, detector_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "track_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, track_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__msg__VisionDetection, age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_members = {
  "autonomy_interfaces__msg",  // message namespace
  "VisionDetection",  // message name
  10,  // number of fields
  sizeof(autonomy_interfaces__msg__VisionDetection),
  false,  // has_any_key_member_
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array,  // message members
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle = {
  0,
  &autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__msg__VisionDetection__get_type_hash,
  &autonomy_interfaces__msg__VisionDetection__get_type_description,
  &autonomy_interfaces__msg__VisionDetection__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, VisionDetection)() {
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__msg__VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
