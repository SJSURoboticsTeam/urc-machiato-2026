// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/navigation_integrity_check__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"
#include "autonomy_interfaces/srv/detail/navigation_integrity_check__struct.h"


// Include directives for member types
// Member `check_components`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__init(message_memory);
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Request__check_components(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Request__check_components(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Request__check_components(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Request__check_components(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Request__check_components(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Request__check_components(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Request__check_components(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Request__check_components(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_member_array[2] = {
  {
    "detailed_check",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request, detailed_check),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "check_components",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request, check_components),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Request__check_components,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Request__check_components,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Request__check_components,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Request__check_components,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Request__check_components,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Request__check_components  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "NavigationIntegrityCheck_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_member_array,  // message members
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_hash,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Request)() {
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__struct.h"


// Include directives for member types
// Member `integrity_level`
// Member `checked_components`
// Member `component_details`
// Member `recommendations`
// Member `timestamp`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `component_status`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__init(message_memory);
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__checked_components(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__checked_components(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__checked_components(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__checked_components(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__checked_components(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__checked_components(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__checked_components(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__checked_components(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__component_status(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_status(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_status(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__component_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_status(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__component_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_status(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__component_status(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__component_details(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_details(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_details(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__component_details(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_details(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__component_details(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_details(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__component_details(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__recommendations(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__recommendations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__recommendations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__recommendations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__recommendations(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__recommendations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__recommendations(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__recommendations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_member_array[13] = {
  {
    "integrity_ok",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, integrity_ok),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "integrity_score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, integrity_score),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "integrity_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, integrity_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "checked_components",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, checked_components),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__checked_components,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__checked_components,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__checked_components,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__checked_components,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__checked_components,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__checked_components  // resize(index) function pointer
  },
  {
    "component_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, component_status),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__component_status,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_status,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_status,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__component_status,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__component_status,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__component_status  // resize(index) function pointer
  },
  {
    "component_details",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, component_details),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__component_details,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__component_details,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__component_details,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__component_details,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__component_details,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__component_details  // resize(index) function pointer
  },
  {
    "position_accuracy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, position_accuracy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "heading_accuracy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, heading_accuracy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_consistency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, velocity_consistency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "satellite_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, satellite_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hdop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, hdop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recommendations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, recommendations),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Response__recommendations,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Response__recommendations,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Response__recommendations,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Response__recommendations,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Response__recommendations,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Response__recommendations  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "NavigationIntegrityCheck_Response",  // message name
  13,  // number of fields
  sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_member_array,  // message members
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_hash,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Response)() {
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "autonomy_interfaces/srv/navigation_integrity_check.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__init(message_memory);
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(message_memory);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Event__request(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * member =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__request(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * member =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__request(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * member =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request * item =
    ((const autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__request(untyped_member, index));
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * value =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * item =
    ((autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__request(untyped_member, index));
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request * value =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Event__request(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * member =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *)(untyped_member);
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__fini(member);
  return autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Event__response(
  const void * untyped_member)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * member =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__response(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * member =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__response(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * member =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response * item =
    ((const autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__response(untyped_member, index));
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * value =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * item =
    ((autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__response(untyped_member, index));
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response * value =
    (const autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Event__response(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * member =
    (autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *)(untyped_member);
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__fini(member);
  return autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event, info),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event, request),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Event__request,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__request,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__request,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Event__request,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Event__request,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Event__request  // resize(index) function pointer
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
    offsetof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event, response),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__size_function__NavigationIntegrityCheck_Event__response,  // size() function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_const_function__NavigationIntegrityCheck_Event__response,  // get_const(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__get_function__NavigationIntegrityCheck_Event__response,  // get(index) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__fetch_function__NavigationIntegrityCheck_Event__response,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__assign_function__NavigationIntegrityCheck_Event__response,  // assign(index, value) function pointer
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__resize_function__NavigationIntegrityCheck_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "NavigationIntegrityCheck_Event",  // message name
  3,  // number of fields
  sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event),
  false,  // has_any_key_member_
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_member_array,  // message members
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_members,
  get_message_typesupport_handle_function,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_hash,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_description,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Event)() {
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Request)();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Response)();
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "NavigationIntegrityCheck",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle,
  NULL,  // response message
  // autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle
  NULL  // event_message
  // autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle
};


static rosidl_service_type_support_t autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_members,
  get_service_typesupport_handle_function,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Request__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Request_message_type_support_handle,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Response__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Response_message_type_support_handle,
  &autonomy_interfaces__srv__NavigationIntegrityCheck_Event__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    NavigationIntegrityCheck
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    autonomy_interfaces,
    srv,
    NavigationIntegrityCheck
  ),
  &autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_hash,
  &autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_description,
  &autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck)(void) {
  if (!autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, NavigationIntegrityCheck_Event)()->data;
  }

  return &autonomy_interfaces__srv__detail__navigation_integrity_check__rosidl_typesupport_introspection_c__NavigationIntegrityCheck_service_type_support_handle;
}
