// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/monitoring_stats__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__MonitoringStats__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb1, 0x3b, 0xa1, 0x92, 0xda, 0xd9, 0x61, 0xd0,
      0xe0, 0x09, 0x78, 0xf0, 0x6b, 0x50, 0x94, 0xd6,
      0x86, 0xa1, 0x64, 0x26, 0xe5, 0xb8, 0x07, 0xfd,
      0x5b, 0x21, 0xdb, 0xb7, 0x5e, 0x9b, 0xe5, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char autonomy_interfaces__msg__MonitoringStats__TYPE_NAME[] = "autonomy_interfaces/msg/MonitoringStats";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__total_evaluations[] = "total_evaluations";
static char autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__total_violations[] = "total_violations";
static char autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__evaluation_rate[] = "evaluation_rate";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__MonitoringStats__FIELDS[] = {
  {
    {autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__total_evaluations, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__total_violations, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__MonitoringStats__FIELD_NAME__evaluation_rate, 15, 15},
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
autonomy_interfaces__msg__MonitoringStats__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
      {autonomy_interfaces__msg__MonitoringStats__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 total_evaluations\n"
  "int32 total_violations\n"
  "float64 evaluation_rate";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__MonitoringStats__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 71, 71},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__MonitoringStats__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__MonitoringStats__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
