// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/system_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__SystemState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0x30, 0x7e, 0x64, 0x38, 0x70, 0xc1, 0x3d,
      0xa5, 0xa6, 0xe7, 0x5d, 0x08, 0x19, 0x6c, 0x84,
      0x62, 0x23, 0xec, 0x4e, 0xb6, 0xcc, 0x50, 0xb9,
      0x22, 0xf6, 0xb3, 0x55, 0xd3, 0xa5, 0xc2, 0x17,
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

static char autonomy_interfaces__msg__SystemState__TYPE_NAME[] = "autonomy_interfaces/msg/SystemState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__current_state[] = "current_state";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__substate[] = "substate";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__sub_substate[] = "sub_substate";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__time_in_state[] = "time_in_state";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__state_timeout[] = "state_timeout";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__previous_state[] = "previous_state";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__transition_timestamp[] = "transition_timestamp";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__is_transitioning[] = "is_transitioning";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__preconditions_met[] = "preconditions_met";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__active_subsystems[] = "active_subsystems";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__failed_subsystems[] = "failed_subsystems";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__mission_phase[] = "mission_phase";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__operator_id[] = "operator_id";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__state_reason[] = "state_reason";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__adaptive_enabled[] = "adaptive_enabled";
static char autonomy_interfaces__msg__SystemState__FIELD_NAME__active_adaptations[] = "active_adaptations";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__SystemState__FIELDS[] = {
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__current_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__substate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__sub_substate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__time_in_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__state_timeout, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__previous_state, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__transition_timestamp, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__is_transitioning, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__preconditions_met, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__active_subsystems, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__failed_subsystems, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__mission_phase, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__state_reason, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__adaptive_enabled, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SystemState__FIELD_NAME__active_adaptations, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__SystemState__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__msg__SystemState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__SystemState__TYPE_NAME, 35, 35},
      {autonomy_interfaces__msg__SystemState__FIELDS, 17, 17},
    },
    {autonomy_interfaces__msg__SystemState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# System State Message - URC 2026 Mars Rover Autonomy\n"
  "#\n"
  "# This message provides comprehensive state information for the rover's hierarchical\n"
  "# state machine. It tracks the current operational state, subsystem status, and\n"
  "# mission context to enable coordinated autonomous operations.\n"
  "#\n"
  "# The state machine uses a three-level hierarchy:\n"
  "# - current_state: Top-level operational mode (BOOT, CALIBRATION, IDLE, etc.)\n"
  "# - substate: Mission-specific context (SCIENCE, DELIVERY, etc.)\n"
  "# - sub_substate: Detailed operational state (SAMPLE_DELIVERY, etc.)\n"
  "#\n"
  "# This design allows for flexible mission execution while maintaining clear\n"
  "# operational boundaries and safety constraints.\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Primary state information\n"
  "string current_state              # Top-level state (BOOT, CALIBRATION, IDLE, NAVIGATION, EXECUTION, RECOVERY, SHUTDOWN)\n"
  "string substate                   # Mission-specific substate (SCIENCE, DELIVERY, MAINTENANCE, etc.)\n"
  "string sub_substate              # Detailed substate (SAMPLE_COLLECTION, DELIVERY_SETUP, DIAGNOSTIC_CHECK, etc.)\n"
  "\n"
  "# State metadata\n"
  "float64 time_in_state            # Seconds spent in current state (resets on transition)\n"
  "float64 state_timeout            # Maximum allowed time in state (0.0 = no timeout, seconds)\n"
  "string previous_state            # Last state before current transition\n"
  "builtin_interfaces/Time transition_timestamp  # UTC timestamp when current state was entered\n"
  "\n"
  "# System status flags\n"
  "bool is_transitioning            # True during state transitions (prevents conflicting operations)\n"
  "bool preconditions_met           # True when all autonomous operation prerequisites are satisfied\n"
  "string[] active_subsystems       # List of currently active subsystems (navigation, vision, slam, etc.)\n"
  "string[] failed_subsystems       # List of subsystems currently in error state\n"
  "\n"
  "# Mission and operational context\n"
  "string mission_phase             # Competition mission context (setup, terrain, equipment_service, science, etc.)\n"
  "string operator_id               # Identifier of who/what initiated current state (auto, manual, safety_system)\n"
  "string state_reason              # Human-readable explanation for current state and any restrictions\n"
  "\n"
  "# Adaptive status\n"
  "bool adaptive_enabled               # True if adaptive transitions are active\n"
  "string[] active_adaptations         # List of currently active adaptations\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__SystemState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__SystemState__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2366, 2366},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__SystemState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__SystemState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
