// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/change_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ChangeState__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7e, 0x30, 0xcc, 0xf0, 0x74, 0x35, 0x90, 0xc5,
      0xda, 0xe8, 0xb1, 0x20, 0x1a, 0x98, 0x10, 0x5a,
      0xf0, 0xd9, 0x5b, 0xc4, 0x9e, 0xed, 0xcc, 0x96,
      0xb0, 0x91, 0x36, 0x9a, 0xa5, 0x9f, 0xd3, 0x19,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ChangeState_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1d, 0x37, 0xc9, 0x02, 0x93, 0x86, 0x3f, 0x28,
      0xe7, 0xbb, 0x76, 0x4e, 0xa4, 0xa3, 0x27, 0xf5,
      0xc3, 0x42, 0xcd, 0x28, 0xed, 0xae, 0x44, 0x32,
      0x32, 0xc8, 0x99, 0x13, 0x83, 0x53, 0xef, 0x62,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ChangeState_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcd, 0x15, 0x77, 0xa3, 0xe3, 0xba, 0x50, 0x28,
      0xbe, 0xd6, 0x50, 0x30, 0x36, 0x06, 0xb7, 0x44,
      0x61, 0x98, 0x79, 0xdf, 0xe5, 0xa6, 0xf7, 0x12,
      0x56, 0xd2, 0xf0, 0xf2, 0x8e, 0x57, 0xbc, 0x4b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ChangeState_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x08, 0x90, 0x39, 0xef, 0xbd, 0x01, 0x9d, 0x76,
      0xad, 0x75, 0xdc, 0x1e, 0x1a, 0xa1, 0xe2, 0x63,
      0xf7, 0xc1, 0x26, 0xfe, 0x70, 0xd2, 0xbc, 0x91,
      0x0b, 0x55, 0x56, 0xbf, 0x26, 0xed, 0x52, 0xc5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char autonomy_interfaces__srv__ChangeState__TYPE_NAME[] = "autonomy_interfaces/srv/ChangeState";
static char autonomy_interfaces__srv__ChangeState_Event__TYPE_NAME[] = "autonomy_interfaces/srv/ChangeState_Event";
static char autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME[] = "autonomy_interfaces/srv/ChangeState_Request";
static char autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME[] = "autonomy_interfaces/srv/ChangeState_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__ChangeState__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__ChangeState__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__ChangeState__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ChangeState__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ChangeState__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ChangeState_Event__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__ChangeState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__ChangeState_Event__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__ChangeState__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ChangeState__TYPE_NAME, 35, 35},
      {autonomy_interfaces__srv__ChangeState__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__ChangeState__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__ChangeState_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__ChangeState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__ChangeState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_state[] = "desired_state";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_substate[] = "desired_substate";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_calibration_substate[] = "desired_calibration_substate";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__reason[] = "reason";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__operator_id[] = "operator_id";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__force[] = "force";
static char autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__metadata[] = "metadata";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ChangeState_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_substate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__desired_calibration_substate, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__force, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Request__FIELD_NAME__metadata, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__ChangeState_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
      {autonomy_interfaces__srv__ChangeState_Request__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_state[] = "actual_state";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_substate[] = "actual_substate";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_calibration_substate[] = "actual_calibration_substate";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__transition_time[] = "transition_time";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__preconditions_met[] = "preconditions_met";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__failed_preconditions[] = "failed_preconditions";
static char autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__warnings[] = "warnings";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ChangeState_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_state, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_substate, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__actual_calibration_substate, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__transition_time, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__preconditions_met, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__failed_preconditions, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__FIELD_NAME__warnings, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__ChangeState_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__ChangeState_Response__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ChangeState_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__ChangeState_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__ChangeState_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ChangeState_Event__TYPE_NAME, 41, 41},
      {autonomy_interfaces__srv__ChangeState_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__ChangeState_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__ChangeState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__ChangeState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request to change system state with validation\n"
  "\n"
  "# Request\n"
  "string desired_state             # Target state to transition to\n"
  "string desired_substate          # Optional: target substate (empty string if not applicable)\n"
  "string desired_calibration_substate  # Optional: target calibration substate (empty string if not applicable)\n"
  "string reason                    # Human-readable reason for state change\n"
  "string operator_id               # ID of operator/system requesting change\n"
  "bool force                       # If true, skip some validation checks (use with caution)\n"
  "string[] metadata                # Additional key=value metadata pairs\n"
  "\n"
  "---\n"
  "# Response\n"
  "\n"
  "bool success                     # True if transition successful\n"
  "string actual_state              # Actual state after transition attempt\n"
  "string actual_substate           # Actual substate after transition\n"
  "string actual_calibration_substate  # Actual calibration substate after transition\n"
  "float64 transition_time          # Time taken to complete transition (seconds)\n"
  "string message                   # Human-readable status message\n"
  "\n"
  "# Validation details\n"
  "bool preconditions_met           # Were all preconditions satisfied\n"
  "string[] failed_preconditions    # List of preconditions that failed\n"
  "string[] warnings                # Non-critical warnings about the transition";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ChangeState__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ChangeState__TYPE_NAME, 35, 35},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1327, 1327},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ChangeState_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ChangeState_Request__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ChangeState_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ChangeState_Response__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ChangeState_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ChangeState_Event__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ChangeState__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ChangeState__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__ChangeState_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__ChangeState_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__ChangeState_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ChangeState_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ChangeState_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ChangeState_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ChangeState_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ChangeState_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ChangeState_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__ChangeState_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__ChangeState_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
