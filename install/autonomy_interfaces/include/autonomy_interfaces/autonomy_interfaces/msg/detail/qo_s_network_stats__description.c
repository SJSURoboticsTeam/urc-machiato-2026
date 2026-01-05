// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/qo_s_network_stats__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__QoSNetworkStats__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xde, 0x4f, 0xe9, 0xd3, 0xa1, 0xb4, 0x18, 0x66,
      0x02, 0x8e, 0x90, 0xb5, 0x32, 0xbc, 0x01, 0xfb,
      0xbf, 0xfa, 0x00, 0xa7, 0x41, 0xa3, 0x40, 0x02,
      0x5b, 0x25, 0xf5, 0x96, 0x22, 0xf7, 0x2c, 0x66,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char autonomy_interfaces__msg__QoSNetworkStats__TYPE_NAME[] = "autonomy_interfaces/msg/QoSNetworkStats";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__bandwidth_up_mbps[] = "bandwidth_up_mbps";
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__bandwidth_down_mbps[] = "bandwidth_down_mbps";
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__latency_ms[] = "latency_ms";
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__packet_loss_rate[] = "packet_loss_rate";
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__current_band[] = "current_band";
static char autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__signal_strength[] = "signal_strength";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__QoSNetworkStats__FIELDS[] = {
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__bandwidth_up_mbps, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__bandwidth_down_mbps, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__latency_ms, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__packet_loss_rate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__current_band, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__QoSNetworkStats__FIELD_NAME__signal_strength, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__QoSNetworkStats__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__QoSNetworkStats__TYPE_NAME, 39, 39},
      {autonomy_interfaces__msg__QoSNetworkStats__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 bandwidth_up_mbps\n"
  "float64 bandwidth_down_mbps\n"
  "float64 latency_ms\n"
  "float64 packet_loss_rate\n"
  "string current_band\n"
  "float64 signal_strength";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__QoSNetworkStats__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__QoSNetworkStats__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 142, 142},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__QoSNetworkStats__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__QoSNetworkStats__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
