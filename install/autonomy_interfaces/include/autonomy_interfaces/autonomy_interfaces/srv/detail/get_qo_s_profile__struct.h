// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetQoSProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_qo_s_profile.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetQoSProfile in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Request
{
  uint8_t structure_needs_at_least_one_member;
} autonomy_interfaces__srv__GetQoSProfile_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetQoSProfile_Request.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Request__Sequence
{
  autonomy_interfaces__srv__GetQoSProfile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetQoSProfile_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'topic_profiles'
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__struct.h"
// Member 'network_stats'
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__struct.h"

/// Struct defined in srv/GetQoSProfile in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Response
{
  autonomy_interfaces__msg__QoSTopicProfile__Sequence topic_profiles;
  autonomy_interfaces__msg__QoSNetworkStats network_stats;
} autonomy_interfaces__srv__GetQoSProfile_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetQoSProfile_Response.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Response__Sequence
{
  autonomy_interfaces__srv__GetQoSProfile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetQoSProfile_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__GetQoSProfile_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__GetQoSProfile_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetQoSProfile in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__GetQoSProfile_Request__Sequence request;
  autonomy_interfaces__srv__GetQoSProfile_Response__Sequence response;
} autonomy_interfaces__srv__GetQoSProfile_Event;

// Struct for a sequence of autonomy_interfaces__srv__GetQoSProfile_Event.
typedef struct autonomy_interfaces__srv__GetQoSProfile_Event__Sequence
{
  autonomy_interfaces__srv__GetQoSProfile_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetQoSProfile_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_QO_S_PROFILE__STRUCT_H_
