// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/software_estop__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SoftwareEstop__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0x89, 0x64, 0x1a, 0x16, 0x19, 0x72, 0x34,
      0x0e, 0xd4, 0x8d, 0x4f, 0xee, 0x9e, 0x2b, 0x42,
      0x05, 0x6c, 0xc2, 0x6b, 0xe6, 0x6c, 0xec, 0x94,
      0x53, 0xc8, 0x2b, 0xa5, 0xc6, 0xad, 0xf7, 0xbe,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SoftwareEstop_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x99, 0xd0, 0xb1, 0x5e, 0x7f, 0x64, 0xbc, 0xb9,
      0x21, 0xc6, 0x62, 0x46, 0xc7, 0xd2, 0x17, 0x5b,
      0x18, 0x59, 0x37, 0x9a, 0xd7, 0xc1, 0x4d, 0xfd,
      0x77, 0xf1, 0x79, 0x87, 0x02, 0xbf, 0xa0, 0xb6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SoftwareEstop_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa0, 0x08, 0x0c, 0x1f, 0xd8, 0x99, 0xb6, 0xc4,
      0xde, 0x24, 0x5a, 0xf5, 0x33, 0x51, 0xf2, 0xfb,
      0x21, 0xab, 0xd3, 0x75, 0xf2, 0x5f, 0x8a, 0x6a,
      0x7f, 0x54, 0x11, 0x83, 0xd2, 0x24, 0xbf, 0x40,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__SoftwareEstop_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe7, 0x7e, 0xe3, 0x19, 0x10, 0x3f, 0x24, 0xed,
      0xfe, 0x94, 0xb5, 0xc0, 0x01, 0x9d, 0xd0, 0x3a,
      0xdc, 0xb1, 0xea, 0xd2, 0xd0, 0xd3, 0x63, 0x60,
      0xca, 0xc1, 0xa8, 0x5d, 0x87, 0x97, 0x0c, 0xf3,
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

static char autonomy_interfaces__srv__SoftwareEstop__TYPE_NAME[] = "autonomy_interfaces/srv/SoftwareEstop";
static char autonomy_interfaces__srv__SoftwareEstop_Event__TYPE_NAME[] = "autonomy_interfaces/srv/SoftwareEstop_Event";
static char autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME[] = "autonomy_interfaces/srv/SoftwareEstop_Request";
static char autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME[] = "autonomy_interfaces/srv/SoftwareEstop_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SoftwareEstop__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__SoftwareEstop_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__SoftwareEstop__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
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
autonomy_interfaces__srv__SoftwareEstop__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SoftwareEstop__TYPE_NAME, 37, 37},
      {autonomy_interfaces__srv__SoftwareEstop__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__SoftwareEstop__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__SoftwareEstop_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__operator_id[] = "operator_id";
static char autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__reason[] = "reason";
static char autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__acknowledge_criticality[] = "acknowledge_criticality";
static char autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__force_immediate[] = "force_immediate";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SoftwareEstop_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__acknowledge_criticality, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__FIELD_NAME__force_immediate, 15, 15},
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
autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__SoftwareEstop_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__estop_id[] = "estop_id";
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__timestamp[] = "timestamp";
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__triggered_by[] = "triggered_by";
static char autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__coordination_started[] = "coordination_started";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SoftwareEstop_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__estop_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__triggered_by, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__FIELD_NAME__coordination_started, 20, 20},
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
autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
      {autonomy_interfaces__srv__SoftwareEstop_Response__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__SoftwareEstop_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__SoftwareEstop_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
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
autonomy_interfaces__srv__SoftwareEstop_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__SoftwareEstop_Event__TYPE_NAME, 43, 43},
      {autonomy_interfaces__srv__SoftwareEstop_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__SoftwareEstop_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Software Emergency Stop Service\n"
  "# Triggers full emergency shutdown via software (equivalent to hardware ESTOP)\n"
  "#\n"
  "# WARNING: This service triggers a critical emergency stop that requires manual\n"
  "# intervention to recover from. Use only in actual emergency situations.\n"
  "# Behavior is identical to pressing the physical emergency stop button.\n"
  "\n"
  "# Request\n"
  "string operator_id              # ID of operator triggering software ESTOP\n"
  "string reason                   # Reason for triggering software ESTOP\n"
  "bool acknowledge_criticality    # Operator acknowledges this is critical action\n"
  "bool force_immediate            # Force immediate shutdown (no coordination)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success                    # True if software ESTOP was triggered\n"
  "string message                  # Status message\n"
  "string estop_id                 # Unique ID for this ESTOP event\n"
  "float64 timestamp               # When ESTOP was triggered\n"
  "string triggered_by             # Operator who triggered it\n"
  "bool coordination_started       # Whether subsystem coordination began";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SoftwareEstop__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SoftwareEstop__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1045, 1045},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SoftwareEstop_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SoftwareEstop_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SoftwareEstop_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SoftwareEstop_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__SoftwareEstop_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__SoftwareEstop_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SoftwareEstop__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SoftwareEstop__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__SoftwareEstop_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__SoftwareEstop_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__SoftwareEstop_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SoftwareEstop_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SoftwareEstop_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SoftwareEstop_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SoftwareEstop_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__SoftwareEstop_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__SoftwareEstop_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__SoftwareEstop_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__SoftwareEstop_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
