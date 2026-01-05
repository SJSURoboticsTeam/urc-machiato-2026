// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/recover_from_safety__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__RecoverFromSafety__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0xdf, 0x37, 0xc5, 0x60, 0x6e, 0x7f, 0xcb,
      0x40, 0x0a, 0x90, 0x81, 0xfb, 0x4a, 0xc2, 0xd3,
      0x03, 0x6a, 0xa6, 0x41, 0xe7, 0xcc, 0x2c, 0x85,
      0x60, 0xb2, 0xc6, 0xea, 0x6b, 0xcb, 0x69, 0x07,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__RecoverFromSafety_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2f, 0x2f, 0x78, 0x11, 0x63, 0x58, 0x68, 0xf9,
      0xb9, 0x81, 0x72, 0xa0, 0x84, 0x6f, 0x47, 0xd2,
      0x28, 0x09, 0x97, 0x38, 0x80, 0x78, 0x53, 0xb5,
      0xfb, 0x9c, 0x90, 0xbb, 0xf2, 0x1d, 0xe0, 0x7f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__RecoverFromSafety_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0xef, 0x72, 0xc5, 0x2c, 0xfe, 0x18, 0xbf,
      0xcd, 0x9b, 0x78, 0x02, 0x18, 0x82, 0xdc, 0x43,
      0x1c, 0x11, 0xf7, 0x47, 0x98, 0x29, 0x14, 0x7e,
      0xe0, 0xf0, 0xb9, 0x1d, 0x4c, 0x97, 0x61, 0x5b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__RecoverFromSafety_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x78, 0xc3, 0x7f, 0x4a, 0xa6, 0x1d, 0x51, 0xc2,
      0xef, 0x14, 0x6d, 0x06, 0x6f, 0x4e, 0xa1, 0xf1,
      0x55, 0x3a, 0xa7, 0xab, 0x71, 0x76, 0x4c, 0xcc,
      0xb4, 0xf2, 0xca, 0x5b, 0xfb, 0x34, 0x0f, 0x42,
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

static char autonomy_interfaces__srv__RecoverFromSafety__TYPE_NAME[] = "autonomy_interfaces/srv/RecoverFromSafety";
static char autonomy_interfaces__srv__RecoverFromSafety_Event__TYPE_NAME[] = "autonomy_interfaces/srv/RecoverFromSafety_Event";
static char autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME[] = "autonomy_interfaces/srv/RecoverFromSafety_Request";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME[] = "autonomy_interfaces/srv/RecoverFromSafety_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__RecoverFromSafety__FIELDS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__RecoverFromSafety_Event__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__RecoverFromSafety__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Event__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
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
autonomy_interfaces__srv__RecoverFromSafety__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__RecoverFromSafety__TYPE_NAME, 41, 41},
      {autonomy_interfaces__srv__RecoverFromSafety__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__RecoverFromSafety__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__RecoverFromSafety_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__RecoverFromSafety_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__RecoverFromSafety_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__recovery_method[] = "recovery_method";
static char autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__operator_id[] = "operator_id";
static char autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__acknowledge_risks[] = "acknowledge_risks";
static char autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__completed_steps[] = "completed_steps";
static char autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__notes[] = "notes";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__RecoverFromSafety_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__recovery_method, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__acknowledge_risks, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__completed_steps, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELD_NAME__notes, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__RecoverFromSafety_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
      {autonomy_interfaces__srv__RecoverFromSafety_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__recovery_state[] = "recovery_state";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__is_safe_to_proceed[] = "is_safe_to_proceed";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__remaining_steps[] = "remaining_steps";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__verified_systems[] = "verified_systems";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__failed_systems[] = "failed_systems";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__estimated_time[] = "estimated_time";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__recommended_next_state[] = "recommended_next_state";
static char autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__restrictions[] = "restrictions";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__RecoverFromSafety_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__recovery_state, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__is_safe_to_proceed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__remaining_steps, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__verified_systems, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__failed_systems, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__estimated_time, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__recommended_next_state, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELD_NAME__restrictions, 12, 12},
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
autonomy_interfaces__srv__RecoverFromSafety_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__RecoverFromSafety_Response__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__RecoverFromSafety_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__RecoverFromSafety_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
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
autonomy_interfaces__srv__RecoverFromSafety_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__RecoverFromSafety_Event__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__RecoverFromSafety_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__RecoverFromSafety_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__RecoverFromSafety_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__RecoverFromSafety_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Initiate recovery from safety state\n"
  "\n"
  "# Request\n"
  "string recovery_method           # \"AUTO\", \"MANUAL_GUIDED\", \"FULL_RESET\"\n"
  "string operator_id               # ID of operator initiating recovery\n"
  "bool acknowledge_risks           # Operator acknowledges recovery risks\n"
  "string[] completed_steps         # Manual steps already completed (for verification)\n"
  "string notes                     # Additional notes about recovery conditions\n"
  "\n"
  "---\n"
  "# Response\n"
  "\n"
  "bool success                     # True if recovery initiated/completed\n"
  "string message                   # Human-readable status message\n"
  "string recovery_state            # \"IN_PROGRESS\", \"COMPLETED\", \"FAILED\", \"REQUIRES_ACTION\"\n"
  "\n"
  "# Recovery details\n"
  "bool is_safe_to_proceed          # True if safe to continue operations\n"
  "string[] remaining_steps         # Steps still needed for full recovery\n"
  "string[] verified_systems        # Systems that passed verification\n"
  "string[] failed_systems          # Systems that failed verification\n"
  "float64 estimated_time           # Estimated time to complete recovery (seconds)\n"
  "\n"
  "# Next state information\n"
  "string recommended_next_state    # Recommended state after recovery\n"
  "string[] restrictions            # Operational restrictions after recovery";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__RecoverFromSafety__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__RecoverFromSafety__TYPE_NAME, 41, 41},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1221, 1221},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__RecoverFromSafety_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__RecoverFromSafety_Request__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__RecoverFromSafety_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__RecoverFromSafety_Response__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__RecoverFromSafety_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__RecoverFromSafety_Event__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__RecoverFromSafety__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__RecoverFromSafety__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__RecoverFromSafety_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__RecoverFromSafety_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__RecoverFromSafety_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__RecoverFromSafety_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__RecoverFromSafety_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__RecoverFromSafety_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__RecoverFromSafety_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__RecoverFromSafety_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
