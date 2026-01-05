// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/configure_mission__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/configure_mission__functions.h"
#include "autonomy_interfaces/srv/detail/configure_mission__struct.h"


// Include directives for member types
// Member `mission_name`
// Member `objectives`
// Member `waypoint_names`
// Member `typing_text`
// Member `terrain_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `waypoints`
// Member `typing_location`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `waypoints`
// Member `typing_location`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `precision_required`
// Member `waypoint_tolerances`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ConfigureMission_Request__init(message_memory);
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ConfigureMission_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__objectives(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__objectives(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__objectives(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__objectives(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__objectives(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__objectives(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__objectives(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__objectives(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoints(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoints(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoints(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoints(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoints(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoints(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoint_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoint_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoint_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoint_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__precision_required(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__precision_required(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__precision_required(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__precision_required(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__precision_required(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__precision_required(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__precision_required(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__precision_required(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoint_tolerances(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_tolerances(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_tolerances(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoint_tolerances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_tolerances(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoint_tolerances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_tolerances(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoint_tolerances(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_member_array[15] = {
  {
    "mission_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, mission_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objectives",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, objectives),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__objectives,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__objectives,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__objectives,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__objectives,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__objectives,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__objectives  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, waypoints),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoints,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoints,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoints,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoints,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoints,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoints  // resize(index) function pointer
  },
  {
    "waypoint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, waypoint_names),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoint_names,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_names,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_names,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoint_names,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoint_names,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoint_names  // resize(index) function pointer
  },
  {
    "precision_required",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, precision_required),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__precision_required,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__precision_required,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__precision_required,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__precision_required,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__precision_required,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__precision_required  // resize(index) function pointer
  },
  {
    "waypoint_tolerances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, waypoint_tolerances),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Request__waypoint_tolerances,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Request__waypoint_tolerances,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Request__waypoint_tolerances,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Request__waypoint_tolerances,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Request__waypoint_tolerances,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Request__waypoint_tolerances  // resize(index) function pointer
  },
  {
    "time_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, time_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoint_timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, waypoint_timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, max_linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, max_angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoint_approach_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, waypoint_approach_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "typing_text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, typing_text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "typing_location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, typing_location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "terrain_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, terrain_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_incline",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Request, max_incline),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ConfigureMission_Request",  // message name
  15,  // number of fields
  sizeof(autonomy_interfaces__srv__ConfigureMission_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_member_array,  // message members
  autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__ConfigureMission_Request__get_type_hash,
  &autonomy_interfaces__srv__ConfigureMission_Request__get_type_description,
  &autonomy_interfaces__srv__ConfigureMission_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Request)() {
  autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_member_array[12].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__struct.h"


// Include directives for member types
// Member `message`
// Member `mission_id`
// Member `configured_objectives`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ConfigureMission_Response__init(message_memory);
}

void autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ConfigureMission_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Response__configured_objectives(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Response__configured_objectives(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Response__configured_objectives(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Response__configured_objectives(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Response__configured_objectives(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Response__configured_objectives(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Response__configured_objectives(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Response__configured_objectives(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, success),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, mission_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, estimated_duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, total_waypoints),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "configured_objectives",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Response, configured_objectives),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Response__configured_objectives,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Response__configured_objectives,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Response__configured_objectives,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Response__configured_objectives,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Response__configured_objectives,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Response__configured_objectives  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ConfigureMission_Response",  // message name
  6,  // number of fields
  sizeof(autonomy_interfaces__srv__ConfigureMission_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_member_array,  // message members
  autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__ConfigureMission_Response__get_type_hash,
  &autonomy_interfaces__srv__ConfigureMission_Response__get_type_description,
  &autonomy_interfaces__srv__ConfigureMission_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Response)() {
  if (!autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/configure_mission.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ConfigureMission_Event__init(message_memory);
}

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ConfigureMission_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__ConfigureMission_Request__Sequence * member =
    (const autonomy_interfaces__srv__ConfigureMission_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__ConfigureMission_Request__Sequence * member =
    (const autonomy_interfaces__srv__ConfigureMission_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence * member =
    (autonomy_interfaces__srv__ConfigureMission_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__ConfigureMission_Request * item =
    ((const autonomy_interfaces__srv__ConfigureMission_Request *)
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__request(untyped_member, index));
  autonomy_interfaces__srv__ConfigureMission_Request * value =
    (autonomy_interfaces__srv__ConfigureMission_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__ConfigureMission_Request * item =
    ((autonomy_interfaces__srv__ConfigureMission_Request *)
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__ConfigureMission_Request * value =
    (const autonomy_interfaces__srv__ConfigureMission_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence * member =
    (autonomy_interfaces__srv__ConfigureMission_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__ConfigureMission_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__ConfigureMission_Response__Sequence * member =
    (const autonomy_interfaces__srv__ConfigureMission_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__ConfigureMission_Response__Sequence * member =
    (const autonomy_interfaces__srv__ConfigureMission_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence * member =
    (autonomy_interfaces__srv__ConfigureMission_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__ConfigureMission_Response * item =
    ((const autonomy_interfaces__srv__ConfigureMission_Response *)
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__response(untyped_member, index));
  autonomy_interfaces__srv__ConfigureMission_Response * value =
    (autonomy_interfaces__srv__ConfigureMission_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__ConfigureMission_Response * item =
    ((autonomy_interfaces__srv__ConfigureMission_Response *)
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__ConfigureMission_Response * value =
    (const autonomy_interfaces__srv__ConfigureMission_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence * member =
    (autonomy_interfaces__srv__ConfigureMission_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__ConfigureMission_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ConfigureMission_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__ConfigureMission_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Event__request,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__srv__ConfigureMission_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__size_function__ConfigureMission_Event__response,  // size() function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_const_function__ConfigureMission_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__get_function__ConfigureMission_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__fetch_function__ConfigureMission_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__assign_function__ConfigureMission_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__resize_function__ConfigureMission_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ConfigureMission_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__ConfigureMission_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_member_array,  // message members
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__ConfigureMission_Event__get_type_hash,
  &autonomy_interfaces__srv__ConfigureMission_Event__get_type_description,
  &autonomy_interfaces__srv__ConfigureMission_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Event)() {
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Request)();
  autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Response)();
  if (!autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/configure_mission__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "ConfigureMission",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__ConfigureMission_Request__rosidl_typesupport_introspection_c__ConfigureMission_Request_message_type_support_handle,
  &autonomy_interfaces__srv__ConfigureMission_Response__rosidl_typesupport_introspection_c__ConfigureMission_Response_message_type_support_handle,
  &autonomy_interfaces__srv__ConfigureMission_Event__rosidl_typesupport_introspection_c__ConfigureMission_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    ConfigureMission
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    ConfigureMission
  ),
  &autonomy_interfaces__srv__ConfigureMission__get_type_hash,
  &autonomy_interfaces__srv__ConfigureMission__get_type_description,
  &autonomy_interfaces__srv__ConfigureMission__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission)(void) {
  if (!autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ConfigureMission_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__configure_mission__rosidl_typesupport_introspection_c__ConfigureMission_service_type_support_handle;
}
