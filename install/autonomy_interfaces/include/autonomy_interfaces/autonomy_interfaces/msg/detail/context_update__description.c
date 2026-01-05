// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/context_update__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__ContextUpdate__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb9, 0x84, 0x4b, 0x33, 0xbc, 0x7c, 0xbd, 0xad,
      0x9c, 0x4b, 0xd4, 0x5e, 0xf1, 0xbc, 0xba, 0x1f,
      0x50, 0x33, 0xfa, 0xa5, 0xc6, 0x98, 0x2a, 0x8e,
      0x4d, 0x65, 0xe7, 0xa3, 0xfa, 0x67, 0x3e, 0x22,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char autonomy_interfaces__msg__ContextUpdate__TYPE_NAME[] = "autonomy_interfaces/msg/ContextUpdate";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__battery_level[] = "battery_level";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__mission_status[] = "mission_status";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__mission_progress[] = "mission_progress";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__communication_active[] = "communication_active";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__safety_active[] = "safety_active";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__active_adaptations[] = "active_adaptations";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__alert_level[] = "alert_level";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__available_actions[] = "available_actions";
static char autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__ContextUpdate__FIELDS[] = {
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__battery_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__mission_status, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__mission_progress, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__communication_active, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__safety_active, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__active_adaptations, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__alert_level, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__available_actions, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextUpdate__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__ContextUpdate__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__ContextUpdate__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__ContextUpdate__TYPE_NAME, 37, 37},
      {autonomy_interfaces__msg__ContextUpdate__FIELDS, 9, 9},
    },
    {autonomy_interfaces__msg__ContextUpdate__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ContextUpdate.msg\n"
  "# Simplified context update for dashboard display\n"
  "\n"
  "# Core status indicators\n"
  "float32 battery_level\n"
  "string mission_status\n"
  "float32 mission_progress\n"
  "bool communication_active\n"
  "bool safety_active\n"
  "\n"
  "# Active adaptive actions\n"
  "string[] active_adaptations\n"
  "\n"
  "# Alert level\n"
  "string alert_level  # NONE, WARNING, CRITICAL\n"
  "\n"
  "# Quick action buttons available\n"
  "string[] available_actions\n"
  "\n"
  "# Timestamp\n"
  "builtin_interfaces/Time timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__ContextUpdate__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__ContextUpdate__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 434, 434},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__ContextUpdate__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__ContextUpdate__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
