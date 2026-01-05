// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/state_transition__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__StateTransition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0x37, 0xbe, 0xbf, 0xfc, 0xcd, 0xc2, 0xbc,
      0x25, 0xdb, 0x08, 0xfb, 0x58, 0xfc, 0xbb, 0x29,
      0x18, 0xd4, 0x12, 0x22, 0x9c, 0xb0, 0xa7, 0x3c,
      0x60, 0x22, 0x21, 0xb6, 0x91, 0xf9, 0x39, 0x4c,
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

static char autonomy_interfaces__msg__StateTransition__TYPE_NAME[] = "autonomy_interfaces/msg/StateTransition";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__from_state[] = "from_state";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__to_state[] = "to_state";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__start_time[] = "start_time";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__end_time[] = "end_time";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__transition_duration[] = "transition_duration";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__reason[] = "reason";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__initiated_by[] = "initiated_by";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__failure_reason[] = "failure_reason";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__preconditions_checked[] = "preconditions_checked";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__entry_actions_executed[] = "entry_actions_executed";
static char autonomy_interfaces__msg__StateTransition__FIELD_NAME__exit_actions_executed[] = "exit_actions_executed";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__StateTransition__FIELDS[] = {
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__from_state, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__to_state, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__start_time, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__end_time, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__transition_duration, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__initiated_by, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__failure_reason, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__preconditions_checked, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__entry_actions_executed, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__StateTransition__FIELD_NAME__exit_actions_executed, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__StateTransition__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__msg__StateTransition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__StateTransition__TYPE_NAME, 39, 39},
      {autonomy_interfaces__msg__StateTransition__FIELDS, 13, 13},
    },
    {autonomy_interfaces__msg__StateTransition__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# State Transition Event Message\n"
  "# Records state transitions for logging and debugging\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Transition details\n"
  "string from_state                # State transitioning from\n"
  "string to_state                  # State transitioning to\n"
  "builtin_interfaces/Time start_time    # When transition started\n"
  "builtin_interfaces/Time end_time      # When transition completed\n"
  "float64 transition_duration      # Duration in seconds\n"
  "\n"
  "# Transition metadata\n"
  "bool success                     # True if transition completed successfully\n"
  "string reason                    # Reason for transition\n"
  "string initiated_by              # Source of transition request (frontend, safety, auto)\n"
  "string failure_reason            # If failed, why it failed\n"
  "\n"
  "# Context information\n"
  "string[] preconditions_checked   # List of preconditions that were validated\n"
  "string[] entry_actions_executed  # Actions executed on entering new state\n"
  "string[] exit_actions_executed   # Actions executed on leaving old state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__StateTransition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__StateTransition__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 989, 989},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__StateTransition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__StateTransition__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
