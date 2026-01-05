// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_topic_profile.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'topic_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/QoSTopicProfile in the package autonomy_interfaces.
typedef struct autonomy_interfaces__msg__QoSTopicProfile
{
  rosidl_runtime_c__String topic_name;
  double avg_latency_ms;
  double jitter_ms;
  double packet_loss_rate;
  int32_t samples_count;
} autonomy_interfaces__msg__QoSTopicProfile;

// Struct for a sequence of autonomy_interfaces__msg__QoSTopicProfile.
typedef struct autonomy_interfaces__msg__QoSTopicProfile__Sequence
{
  autonomy_interfaces__msg__QoSTopicProfile * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__QoSTopicProfile__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__STRUCT_H_
