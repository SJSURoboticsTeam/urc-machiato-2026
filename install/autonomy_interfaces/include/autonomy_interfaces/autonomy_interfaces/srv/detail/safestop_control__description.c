// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/SafestopControl.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/safestop_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SafestopControl__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x10, 0x55, 0xb6, 0xfe, 0x4b, 0x90, 0xc1, 0x96,
      0xa8, 0xd8, 0xa6, 0x4e, 0xf0, 0xdf, 0x3c, 0xe4,
      0xbd, 0x25, 0xa2, 0x5a, 0x12, 0x35, 0xe2, 0x6e,
      0x49, 0x61, 0x25, 0xc8, 0x56, 0x20, 0x55, 0x55,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SafestopControl_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x10, 0x5a, 0x87, 0x0e, 0xd6, 0xd6, 0x05, 0x5a,
      0x77, 0xd3, 0xde, 0x51, 0x0a, 0x2e, 0xe2, 0x27,
      0x29, 0xac, 0xd1, 0xb0, 0x61, 0xe0, 0xd9, 0xc1,
      0x2d, 0xde, 0x26, 0x57, 0x29, 0x0b, 0xfb, 0x0d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SafestopControl_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xca, 0xd1, 0x06, 0xeb, 0xc0, 0x15, 0x4f, 0x8c,
      0x6c, 0xfb, 0x17, 0x31, 0x48, 0xea, 0x5b, 0x96,
      0x28, 0xf0, 0xe2, 0x0f, 0x2a, 0x2c, 0xd7, 0xa8,
      0x77, 0x3a, 0x6d, 0xc1, 0xde, 0x1d, 0x19, 0xa2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SafestopControl_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x9d, 0x02, 0xcd, 0x9d, 0xe7, 0xc2, 0xaf,
      0x13, 0xb9, 0x29, 0x81, 0xf8, 0xf0, 0x38, 0x60,
      0x14, 0x33, 0xd6, 0x29, 0x95, 0xbf, 0xd3, 0x22,
      0x7e, 0x38, 0x77, 0x25, 0x58, 0x40, 0xcb, 0x81,
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

static char autonomy_interfaces__srv__SafestopControl__TYPE_NAME[] = "autonomy_interfaces/srv/SafestopControl";
static char autonomy_interfaces__srv__SafestopControl_Event__TYPE_NAME[] = "autonomy_interfaces/srv/SafestopControl_Event";
static char autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME[] = "autonomy_interfaces/srv/SafestopControl_Request";
static char autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME[] = "autonomy_interfaces/srv/SafestopControl_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__SafestopControl__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__SafestopControl__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__SafestopControl__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SafestopControl__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SafestopControl_Event__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__SafestopControl__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl_Event__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__SafestopControl__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SafestopControl__TYPE_NAME, 39, 39},
      {autonomy_interfaces__srv__SafestopControl__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__SafestopControl__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__SafestopControl_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__SafestopControl_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__SafestopControl_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__command[] = "command";
static char autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__operator_id[] = "operator_id";
static char autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__acknowledge_risks[] = "acknowledge_risks";
static char autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__reason[] = "reason";
static char autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__affected_subsystems[] = "affected_subsystems";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SafestopControl_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__acknowledge_risks, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Request__FIELD_NAME__affected_subsystems, 19, 19},
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
autonomy_interfaces__srv__SafestopControl_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__SafestopControl_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__current_state[] = "current_state";
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__affected_subsystems[] = "affected_subsystems";
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__timestamp[] = "timestamp";
static char autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__operator_id[] = "operator_id";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SafestopControl_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__current_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__affected_subsystems, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__FIELD_NAME__operator_id, 11, 11},
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
autonomy_interfaces__srv__SafestopControl_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__SafestopControl_Response__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SafestopControl_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__SafestopControl_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__SafestopControl_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SafestopControl_Event__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__SafestopControl_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__SafestopControl_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__SafestopControl_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__SafestopControl_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Safestop Control Service\n"
  "# Allows operator to engage/disengage/toggle safestop\n"
  "\n"
  "# Request\n"
  "string command              # \"ENGAGE\", \"DISENGAGE\", \"TOGGLE\"\n"
  "string operator_id          # ID of operator initiating the command\n"
  "bool acknowledge_risks      # Operator acknowledges any risks\n"
  "string reason               # Optional reason for the command\n"
  "string[] affected_subsystems # Specific subsystems to control (empty = all)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success                # Command executed successfully\n"
  "string message              # Human-readable status message\n"
  "string current_state        # \"ENGAGED\", \"DISENGAGED\", \"IN_PROGRESS\", \"FAILED\"\n"
  "string[] affected_subsystems # Subsystems that were affected\n"
  "float64 timestamp           # When command was processed\n"
  "string operator_id          # Operator who executed the command";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SafestopControl__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SafestopControl__TYPE_NAME, 39, 39},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 819, 819},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SafestopControl_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SafestopControl_Request__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SafestopControl_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SafestopControl_Response__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SafestopControl_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SafestopControl_Event__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SafestopControl__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SafestopControl__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__SafestopControl_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__SafestopControl_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__SafestopControl_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SafestopControl_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SafestopControl_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SafestopControl_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SafestopControl_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SafestopControl_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SafestopControl_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__SafestopControl_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__SafestopControl_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
