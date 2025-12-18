// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/LoadCalibrationParameters.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/load_calibration_parameters__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/load_calibration_parameters__functions.h"
#include "autonomy_interfaces/srv/detail/load_calibration_parameters__struct.h"


// Include directives for member types
// Member `calibration_file`
// Member `parameter_namespace`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__LoadCalibrationParameters_Request__init(message_memory);
}

void autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__LoadCalibrationParameters_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_member_array[2] = {
  {
    "calibration_file",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__LoadCalibrationParameters_Request, calibration_file),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parameter_namespace",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__LoadCalibrationParameters_Request, parameter_namespace),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "LoadCalibrationParameters_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__LoadCalibrationParameters_Request),
  autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_member_array,  // message members
  autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Request)() {
  if (!autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__LoadCalibrationParameters_Request__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/load_calibration_parameters__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/load_calibration_parameters__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/load_calibration_parameters__struct.h"


// Include directives for member types
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__LoadCalibrationParameters_Response__init(message_memory);
}

void autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__LoadCalibrationParameters_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__LoadCalibrationParameters_Response, success),  // bytes offset in struct
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
    offsetof(autonomy_interfaces__srv__LoadCalibrationParameters_Response, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "LoadCalibrationParameters_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__LoadCalibrationParameters_Response),
  autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_member_array,  // message members
  autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Response)() {
  if (!autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__LoadCalibrationParameters_Response__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/load_calibration_parameters__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "LoadCalibrationParameters",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters)() {
  if (!autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, LoadCalibrationParameters_Response)()->data;
  }

  return &autonomy_interfaces__srv__detail__load_calibration_parameters__rosidl_typesupport_introspection_c__LoadCalibrationParameters_service_type_support_handle;
}
