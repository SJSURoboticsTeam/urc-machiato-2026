// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/safety_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__SafetyStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0xd4, 0x83, 0xed, 0x41, 0xc0, 0x88, 0x22,
      0x12, 0x9b, 0x3b, 0xfd, 0xcd, 0x57, 0x6a, 0x50,
      0x0b, 0x1e, 0x35, 0x83, 0x34, 0x7b, 0xd5, 0xf4,
      0xb3, 0x66, 0x93, 0x61, 0xa4, 0xff, 0xd6, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char autonomy_interfaces__msg__SafetyStatus__TYPE_NAME[] = "autonomy_interfaces/msg/SafetyStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__is_safe[] = "is_safe";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__safety_level[] = "safety_level";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__active_triggers[] = "active_triggers";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_type[] = "trigger_type";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_source[] = "trigger_source";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_time[] = "trigger_time";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_description[] = "trigger_description";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__requires_manual_intervention[] = "requires_manual_intervention";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__can_auto_recover[] = "can_auto_recover";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__recovery_steps[] = "recovery_steps";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__estimated_recovery_time[] = "estimated_recovery_time";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__context_state[] = "context_state";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__mission_phase[] = "mission_phase";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__safe_to_retry[] = "safe_to_retry";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__battery_level[] = "battery_level";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__temperature[] = "temperature";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__communication_ok[] = "communication_ok";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__sensors_ok[] = "sensors_ok";
static char autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__degraded_capabilities[] = "degraded_capabilities";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__SafetyStatus__FIELDS[] = {
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__is_safe, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__safety_level, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__active_triggers, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_source, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__trigger_description, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__requires_manual_intervention, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__can_auto_recover, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__recovery_steps, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__estimated_recovery_time, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__context_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__mission_phase, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__safe_to_retry, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__battery_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__communication_ok, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__sensors_ok, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyStatus__FIELD_NAME__degraded_capabilities, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__SafetyStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__SafetyStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__SafetyStatus__TYPE_NAME, 36, 36},
      {autonomy_interfaces__msg__SafetyStatus__FIELDS, 20, 20},
    },
    {autonomy_interfaces__msg__SafetyStatus__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Safety Status Message\n"
  "# Comprehensive safety system status with trigger types and recovery requirements\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Safety state\n"
  "bool is_safe                     # Overall safety status\n"
  "string safety_level              # \"NORMAL\", \"WARNING\", \"CRITICAL\", \"EMERGENCY\"\n"
  "string[] active_triggers         # List of currently active safety triggers\n"
  "\n"
  "# Trigger information\n"
  "string trigger_type              # Type of safety trigger\n"
  "string trigger_source            # Subsystem that triggered safety response\n"
  "builtin_interfaces/Time trigger_time  # When safety was triggered\n"
  "string trigger_description       # Human-readable description\n"
  "\n"
  "# Recovery information\n"
  "bool requires_manual_intervention  # True if manual recovery required\n"
  "bool can_auto_recover            # True if automatic recovery possible\n"
  "string[] recovery_steps          # List of steps needed for recovery\n"
  "float64 estimated_recovery_time  # Estimated time to recover (seconds)\n"
  "\n"
  "# Context-specific information\n"
  "string context_state             # State when safety was triggered\n"
  "string mission_phase             # Mission context when triggered\n"
  "bool safe_to_retry               # True if safe to retry after recovery\n"
  "\n"
  "# System health\n"
  "float64 battery_level            # Current battery percentage\n"
  "float64 temperature              # System temperature (Celsius)\n"
  "bool communication_ok            # Communication link status\n"
  "bool sensors_ok                  # Sensor system status\n"
  "string[] degraded_capabilities   # List of capabilities that are degraded";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__SafetyStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__SafetyStatus__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1520, 1520},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__SafetyStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__SafetyStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
