// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/context_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__ContextState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7b, 0xc8, 0x0e, 0x76, 0x1e, 0xf6, 0x8c, 0xa6,
      0xfd, 0x02, 0x4d, 0x4c, 0x91, 0x3f, 0xfd, 0xf5,
      0x02, 0x8f, 0xa2, 0x6a, 0x6a, 0xa8, 0x1d, 0x32,
      0xad, 0x3f, 0x72, 0x1c, 0xd4, 0x6b, 0x5e, 0x3a,
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

static char autonomy_interfaces__msg__ContextState__TYPE_NAME[] = "autonomy_interfaces/msg/ContextState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_level[] = "battery_level";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_critical[] = "battery_critical";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_warning[] = "battery_warning";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_type[] = "mission_type";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_status[] = "mission_status";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_progress[] = "mission_progress";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_time_remaining[] = "mission_time_remaining";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_active[] = "communication_active";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_latency[] = "communication_latency";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_quality[] = "communication_quality";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__cpu_usage[] = "cpu_usage";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__memory_usage[] = "memory_usage";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__temperature[] = "temperature";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__obstacle_detected[] = "obstacle_detected";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__obstacle_distance[] = "obstacle_distance";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__terrain_difficulty[] = "terrain_difficulty";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__weather_adverse[] = "weather_adverse";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__safety_active[] = "safety_active";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__safety_reason[] = "safety_reason";
static char autonomy_interfaces__msg__ContextState__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__ContextState__FIELDS[] = {
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_critical, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__battery_warning, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_status, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_progress, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__mission_time_remaining, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_active, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_latency, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__communication_quality, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__cpu_usage, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__memory_usage, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__obstacle_detected, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__obstacle_distance, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__terrain_difficulty, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__weather_adverse, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__safety_active, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__safety_reason, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__ContextState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__ContextState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
      {autonomy_interfaces__msg__ContextState__FIELDS, 21, 21},
    },
    {autonomy_interfaces__msg__ContextState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ContextState.msg\n"
  "# Real-time context information for adaptive state machine\n"
  "\n"
  "# Battery information\n"
  "float32 battery_level\n"
  "float32 battery_voltage\n"
  "bool battery_critical\n"
  "bool battery_warning\n"
  "\n"
  "# Mission status\n"
  "string mission_type\n"
  "string mission_status  # IDLE, EXECUTING, COMPLETED, FAILED, PAUSED\n"
  "float32 mission_progress  # 0.0 to 1.0\n"
  "float32 mission_time_remaining  # seconds\n"
  "\n"
  "# Communication health\n"
  "bool communication_active\n"
  "float32 communication_latency  # seconds\n"
  "int32 communication_quality  # 0-100\n"
  "\n"
  "# System performance\n"
  "float32 cpu_usage\n"
  "float32 memory_usage\n"
  "float32 temperature\n"
  "\n"
  "# Environmental conditions\n"
  "bool obstacle_detected\n"
  "float32 obstacle_distance\n"
  "float32 terrain_difficulty  # 0.0 (easy) to 1.0 (difficult)\n"
  "bool weather_adverse\n"
  "\n"
  "# Safety status\n"
  "bool safety_active\n"
  "string safety_reason\n"
  "\n"
  "# Timestamp\n"
  "builtin_interfaces/Time timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__ContextState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 848, 848},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__ContextState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
