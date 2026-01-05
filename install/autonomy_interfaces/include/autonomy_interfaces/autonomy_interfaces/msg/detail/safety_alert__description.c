// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/safety_alert__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__SafetyAlert__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8b, 0xc5, 0x4a, 0xc4, 0x6f, 0x13, 0x9a, 0x46,
      0xbf, 0xa7, 0x76, 0xa9, 0x25, 0x23, 0x64, 0xbf,
      0xee, 0xc6, 0xcd, 0xa6, 0xeb, 0x29, 0xcc, 0x58,
      0x40, 0x33, 0x77, 0xb4, 0x61, 0x31, 0xbd, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char autonomy_interfaces__msg__SafetyAlert__TYPE_NAME[] = "autonomy_interfaces/msg/SafetyAlert";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__property[] = "property";
static char autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__severity[] = "severity";
static char autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__details[] = "details";
static char autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__timestamp[] = "timestamp";
static char autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__acknowledged[] = "acknowledged";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__SafetyAlert__FIELDS[] = {
  {
    {autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__property, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__severity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__details, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__FIELD_NAME__acknowledged, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__SafetyAlert__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
      {autonomy_interfaces__msg__SafetyAlert__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string property\n"
  "string severity\n"
  "string details\n"
  "float64 timestamp\n"
  "bool acknowledged";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__SafetyAlert__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 83, 83},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__SafetyAlert__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__SafetyAlert__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
