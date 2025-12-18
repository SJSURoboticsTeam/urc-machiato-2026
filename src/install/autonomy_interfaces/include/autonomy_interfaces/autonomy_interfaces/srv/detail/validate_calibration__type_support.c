// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/validate_calibration__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/validate_calibration__functions.h"
#include "autonomy_interfaces/srv/detail/validate_calibration__struct.h"


// Include directives for member types
// Member `calibration_file`
// Member `test_images`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ValidateCalibration_Request__init(message_memory);
}

void autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ValidateCalibration_Request__fini(message_memory);
}

size_t autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__size_function__ValidateCalibration_Request__test_images(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateCalibration_Request__test_images(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_function__ValidateCalibration_Request__test_images(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateCalibration_Request__test_images(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateCalibration_Request__test_images(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__assign_function__ValidateCalibration_Request__test_images(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_function__ValidateCalibration_Request__test_images(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__resize_function__ValidateCalibration_Request__test_images(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_member_array[2] = {
  {
    "calibration_file",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Request, calibration_file),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "test_images",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Request, test_images),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__size_function__ValidateCalibration_Request__test_images,  // size() function pointer
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateCalibration_Request__test_images,  // get_const(index) function pointer
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__get_function__ValidateCalibration_Request__test_images,  // get(index) function pointer
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateCalibration_Request__test_images,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__assign_function__ValidateCalibration_Request__test_images,  // assign(index, value) function pointer
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__resize_function__ValidateCalibration_Request__test_images  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ValidateCalibration_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__ValidateCalibration_Request),
  autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_member_array,  // message members
  autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Request)() {
  if (!autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ValidateCalibration_Request__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/validate_calibration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/validate_calibration__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/validate_calibration__struct.h"


// Include directives for member types
// Member `quality_assessment`
// Member `recommendations`
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__ValidateCalibration_Response__init(message_memory);
}

void autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__ValidateCalibration_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reprojection_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Response, reprojection_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quality_assessment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Response, quality_assessment),  // bytes offset in struct
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
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Response, recommendations),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__ValidateCalibration_Response, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "ValidateCalibration_Response",  // message name
  5,  // number of fields
  sizeof(autonomy_interfaces__srv__ValidateCalibration_Response),
  autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_member_array,  // message members
  autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Response)() {
  if (!autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__ValidateCalibration_Response__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/validate_calibration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "ValidateCalibration",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration)() {
  if (!autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, ValidateCalibration_Response)()->data;
  }

  return &autonomy_interfaces__srv__detail__validate_calibration__rosidl_typesupport_introspection_c__ValidateCalibration_service_type_support_handle;
}
