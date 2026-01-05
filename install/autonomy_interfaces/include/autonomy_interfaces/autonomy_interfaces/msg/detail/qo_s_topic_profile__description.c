// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__QoSTopicProfile__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xde, 0x40, 0xb1, 0x1a, 0xf6, 0xe4, 0xcd,
      0x78, 0x47, 0x82, 0x01, 0x17, 0xd8, 0x38, 0x17,
      0x8a, 0x1d, 0x69, 0x47, 0x35, 0x17, 0x84, 0xfc,
      0x10, 0xd2, 0xdc, 0xdf, 0xda, 0x1a, 0xc5, 0xbf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char autonomy_interfaces__msg__QoSTopicProfile__TYPE_NAME[] = "autonomy_interfaces/msg/QoSTopicProfile";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__topic_name[] = "topic_name";
static char autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__avg_latency_ms[] = "avg_latency_ms";
static char autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__jitter_ms[] = "jitter_ms";
static char autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__packet_loss_rate[] = "packet_loss_rate";
static char autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__samples_count[] = "samples_count";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__QoSTopicProfile__FIELDS[] = {
  {
    {autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__topic_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__avg_latency_ms, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__jitter_ms, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__packet_loss_rate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSTopicProfile__FIELD_NAME__samples_count, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__QoSTopicProfile__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__QoSTopicProfile__TYPE_NAME, 39, 39},
      {autonomy_interfaces__msg__QoSTopicProfile__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string topic_name\n"
  "float64 avg_latency_ms\n"
  "float64 jitter_ms\n"
  "float64 packet_loss_rate\n"
  "int32 samples_count";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__QoSTopicProfile__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__QoSTopicProfile__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 104, 104},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__QoSTopicProfile__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__QoSTopicProfile__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
