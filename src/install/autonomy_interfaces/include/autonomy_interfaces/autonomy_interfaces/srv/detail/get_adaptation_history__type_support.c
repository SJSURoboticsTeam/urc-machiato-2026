// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:srv/GetAdaptationHistory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/srv/detail/get_adaptation_history__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/srv/detail/get_adaptation_history__functions.h"
#include "autonomy_interfaces/srv/detail/get_adaptation_history__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetAdaptationHistory_Request__init(message_memory);
}

void autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetAdaptationHistory_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_member_array[2] = {
  {
    "limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAdaptationHistory_Request, limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "include_context",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAdaptationHistory_Request, include_context),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetAdaptationHistory_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__GetAdaptationHistory_Request),
  autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_member_array,  // message members
  autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Request)() {
  if (!autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetAdaptationHistory_Request__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/srv/detail/get_adaptation_history__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_adaptation_history__functions.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_adaptation_history__struct.h"


// Include directives for member types
// Member `actions`
#include "autonomy_interfaces/msg/adaptive_action.h"
// Member `actions`
#include "autonomy_interfaces/msg/detail/adaptive_action__rosidl_typesupport_introspection_c.h"
// Member `contexts`
#include "autonomy_interfaces/msg/context_state.h"
// Member `contexts`
#include "autonomy_interfaces/msg/detail/context_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__srv__GetAdaptationHistory_Response__init(message_memory);
}

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__srv__GetAdaptationHistory_Response__fini(message_memory);
}

size_t autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__size_function__GetAdaptationHistory_Response__actions(
  const void * untyped_member)
{
  const autonomy_interfaces__msg__AdaptiveAction__Sequence * member =
    (const autonomy_interfaces__msg__AdaptiveAction__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__actions(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__msg__AdaptiveAction__Sequence * member =
    (const autonomy_interfaces__msg__AdaptiveAction__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__actions(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__msg__AdaptiveAction__Sequence * member =
    (autonomy_interfaces__msg__AdaptiveAction__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__fetch_function__GetAdaptationHistory_Response__actions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__msg__AdaptiveAction * item =
    ((const autonomy_interfaces__msg__AdaptiveAction *)
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__actions(untyped_member, index));
  autonomy_interfaces__msg__AdaptiveAction * value =
    (autonomy_interfaces__msg__AdaptiveAction *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__assign_function__GetAdaptationHistory_Response__actions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__msg__AdaptiveAction * item =
    ((autonomy_interfaces__msg__AdaptiveAction *)
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__actions(untyped_member, index));
  const autonomy_interfaces__msg__AdaptiveAction * value =
    (const autonomy_interfaces__msg__AdaptiveAction *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__resize_function__GetAdaptationHistory_Response__actions(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__msg__AdaptiveAction__Sequence * member =
    (autonomy_interfaces__msg__AdaptiveAction__Sequence *)(untyped_member);
  autonomy_interfaces__msg__AdaptiveAction__Sequence__fini(member);
  return autonomy_interfaces__msg__AdaptiveAction__Sequence__init(member, size);
}

size_t autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__size_function__GetAdaptationHistory_Response__contexts(
  const void * untyped_member)
{
  const autonomy_interfaces__msg__ContextState__Sequence * member =
    (const autonomy_interfaces__msg__ContextState__Sequence *)(untyped_member);
  return member->size;
}

const void * autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__contexts(
  const void * untyped_member, size_t index)
{
  const autonomy_interfaces__msg__ContextState__Sequence * member =
    (const autonomy_interfaces__msg__ContextState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__contexts(
  void * untyped_member, size_t index)
{
  autonomy_interfaces__msg__ContextState__Sequence * member =
    (autonomy_interfaces__msg__ContextState__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__fetch_function__GetAdaptationHistory_Response__contexts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonomy_interfaces__msg__ContextState * item =
    ((const autonomy_interfaces__msg__ContextState *)
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__contexts(untyped_member, index));
  autonomy_interfaces__msg__ContextState * value =
    (autonomy_interfaces__msg__ContextState *)(untyped_value);
  *value = *item;
}

void autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__assign_function__GetAdaptationHistory_Response__contexts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonomy_interfaces__msg__ContextState * item =
    ((autonomy_interfaces__msg__ContextState *)
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__contexts(untyped_member, index));
  const autonomy_interfaces__msg__ContextState * value =
    (const autonomy_interfaces__msg__ContextState *)(untyped_value);
  *item = *value;
}

bool autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__resize_function__GetAdaptationHistory_Response__contexts(
  void * untyped_member, size_t size)
{
  autonomy_interfaces__msg__ContextState__Sequence * member =
    (autonomy_interfaces__msg__ContextState__Sequence *)(untyped_member);
  autonomy_interfaces__msg__ContextState__Sequence__fini(member);
  return autonomy_interfaces__msg__ContextState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_member_array[2] = {
  {
    "actions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAdaptationHistory_Response, actions),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__size_function__GetAdaptationHistory_Response__actions,  // size() function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__actions,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__actions,  // get(index) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__fetch_function__GetAdaptationHistory_Response__actions,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__assign_function__GetAdaptationHistory_Response__actions,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__resize_function__GetAdaptationHistory_Response__actions  // resize(index) function pointer
  },
  {
    "contexts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__srv__GetAdaptationHistory_Response, contexts),  // bytes offset in struct
    NULL,  // default value
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__size_function__GetAdaptationHistory_Response__contexts,  // size() function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_const_function__GetAdaptationHistory_Response__contexts,  // get_const(index) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__get_function__GetAdaptationHistory_Response__contexts,  // get(index) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__fetch_function__GetAdaptationHistory_Response__contexts,  // fetch(index, &value) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__assign_function__GetAdaptationHistory_Response__contexts,  // assign(index, value) function pointer
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__resize_function__GetAdaptationHistory_Response__contexts  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_members = {
  "autonomy_interfaces__srv",  // message namespace
  "GetAdaptationHistory_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__srv__GetAdaptationHistory_Response),
  autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_member_array,  // message members
  autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Response)() {
  autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, AdaptiveAction)();
  autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, ContextState)();
  if (!autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__srv__GetAdaptationHistory_Response__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/srv/detail/get_adaptation_history__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_members = {
  "autonomy_interfaces__srv",  // service namespace
  "GetAdaptationHistory",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_type_support_handle = {
  0,
  &autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory)() {
  if (!autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, srv, GetAdaptationHistory_Response)()->data;
  }

  return &autonomy_interfaces__srv__detail__get_adaptation_history__rosidl_typesupport_introspection_c__GetAdaptationHistory_service_type_support_handle;
}
