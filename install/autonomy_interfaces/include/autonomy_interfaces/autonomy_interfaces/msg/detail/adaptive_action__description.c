// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/adaptive_action__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__AdaptiveAction__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x29, 0x16, 0x3f, 0x12, 0xe6, 0x63, 0x6c, 0x42,
      0x27, 0xe9, 0x6a, 0x39, 0x77, 0xcd, 0x35, 0xe0,
      0x95, 0xb2, 0x80, 0xaa, 0xd1, 0xe7, 0x83, 0x11,
      0x38, 0xd5, 0xa7, 0x6a, 0x49, 0xac, 0xc0, 0xf2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "autonomy_interfaces/msg/detail/context_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t autonomy_interfaces__msg__ContextState__EXPECTED_HASH = {1, {
    0x7b, 0xc8, 0x0e, 0x76, 0x1e, 0xf6, 0x8c, 0xa6,
    0xfd, 0x02, 0x4d, 0x4c, 0x91, 0x3f, 0xfd, 0xf5,
    0x02, 0x8f, 0xa2, 0x6a, 0x6a, 0xa8, 0x1d, 0x32,
    0xad, 0x3f, 0x72, 0x1c, 0xd4, 0x6b, 0x5e, 0x3a,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME[] = "autonomy_interfaces/msg/AdaptiveAction";
static char autonomy_interfaces__msg__ContextState__TYPE_NAME[] = "autonomy_interfaces/msg/ContextState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__action_type[] = "action_type";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__parameters[] = "parameters";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__trigger_context[] = "trigger_context";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__priority[] = "priority";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__expected_duration[] = "expected_duration";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__success_criteria[] = "success_criteria";
static char autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__AdaptiveAction__FIELDS[] = {
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__action_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__parameters, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__trigger_context, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__priority, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__expected_duration, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__success_criteria, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AdaptiveAction__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__AdaptiveAction__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__AdaptiveAction__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
      {autonomy_interfaces__msg__AdaptiveAction__FIELDS, 7, 7},
    },
    {autonomy_interfaces__msg__AdaptiveAction__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# AdaptiveAction.msg\n"
  "# Actions taken by the adaptive state machine\n"
  "\n"
  "# Action type\n"
  "string action_type  # EMERGENCY_RETURN, REDUCE_POWER, OBSTACLE_AVOIDANCE, etc.\n"
  "\n"
  "# Action parameters\n"
  "string[] parameters  # Key-value pairs as strings\n"
  "\n"
  "# Context that triggered the action\n"
  "ContextState trigger_context\n"
  "\n"
  "# Action priority (higher = more urgent)\n"
  "int32 priority  # 0-100\n"
  "\n"
  "# Expected duration (seconds)\n"
  "float32 expected_duration\n"
  "\n"
  "# Success criteria\n"
  "string success_criteria\n"
  "\n"
  "# Timestamp\n"
  "builtin_interfaces/Time timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__AdaptiveAction__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 512, 512},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__AdaptiveAction__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__AdaptiveAction__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
