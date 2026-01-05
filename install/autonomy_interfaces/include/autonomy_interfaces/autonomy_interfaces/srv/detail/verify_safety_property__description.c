// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/VerifySafetyProperty.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/verify_safety_property__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__VerifySafetyProperty__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x07, 0x4f, 0x10, 0x06, 0x96, 0xfe, 0xb4, 0x0f,
      0xd0, 0x46, 0xe0, 0x37, 0x88, 0x53, 0x96, 0xae,
      0x48, 0xea, 0xac, 0x48, 0x79, 0x83, 0x53, 0xe0,
      0x46, 0x78, 0x3f, 0x2f, 0x9a, 0xb8, 0x93, 0x48,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__VerifySafetyProperty_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0xa5, 0x06, 0x29, 0xbe, 0x03, 0xce, 0xd4,
      0x14, 0x0f, 0x95, 0x58, 0xb0, 0x2b, 0xc0, 0x35,
      0x88, 0x41, 0x40, 0x12, 0x79, 0x96, 0xeb, 0x61,
      0x9d, 0x21, 0xc2, 0x24, 0xef, 0x46, 0x66, 0x37,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__VerifySafetyProperty_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0xf7, 0x26, 0xb3, 0x9c, 0x36, 0x23, 0xd1,
      0xda, 0xff, 0x6a, 0xda, 0xb5, 0xae, 0x89, 0xab,
      0x04, 0xb8, 0x85, 0x40, 0xd1, 0x05, 0x4f, 0x4d,
      0xba, 0x25, 0x10, 0x79, 0xeb, 0x65, 0x1b, 0xe3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__VerifySafetyProperty_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0xe9, 0x98, 0x79, 0xb5, 0x44, 0x69, 0x61,
      0xf0, 0x6d, 0xf0, 0x34, 0xc1, 0x6d, 0xa7, 0xcf,
      0x43, 0xec, 0x9c, 0xfd, 0xa4, 0x75, 0x5f, 0x97,
      0x6a, 0x46, 0xe6, 0x18, 0x3a, 0x4b, 0xb0, 0xc4,
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

static char autonomy_interfaces__srv__VerifySafetyProperty__TYPE_NAME[] = "autonomy_interfaces/srv/VerifySafetyProperty";
static char autonomy_interfaces__srv__VerifySafetyProperty_Event__TYPE_NAME[] = "autonomy_interfaces/srv/VerifySafetyProperty_Event";
static char autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME[] = "autonomy_interfaces/srv/VerifySafetyProperty_Request";
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME[] = "autonomy_interfaces/srv/VerifySafetyProperty_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__VerifySafetyProperty__FIELDS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__VerifySafetyProperty_Event__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__VerifySafetyProperty__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
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
autonomy_interfaces__srv__VerifySafetyProperty__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__VerifySafetyProperty__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__VerifySafetyProperty__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__VerifySafetyProperty__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__VerifySafetyProperty_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__VerifySafetyProperty_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__VerifySafetyProperty_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__VerifySafetyProperty_Request__FIELD_NAME__property_name[] = "property_name";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__VerifySafetyProperty_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Request__FIELD_NAME__property_name, 13, 13},
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
autonomy_interfaces__srv__VerifySafetyProperty_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
      {autonomy_interfaces__srv__VerifySafetyProperty_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__property_name[] = "property_name";
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__satisfied[] = "satisfied";
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__details[] = "details";
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__violation_count[] = "violation_count";
static char autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__last_violation[] = "last_violation";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__property_name, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__satisfied, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__details, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__violation_count, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELD_NAME__last_violation, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__VerifySafetyProperty_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
      {autonomy_interfaces__srv__VerifySafetyProperty_Response__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__VerifySafetyProperty_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
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
autonomy_interfaces__srv__VerifySafetyProperty_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__VerifySafetyProperty_Event__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__VerifySafetyProperty_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__VerifySafetyProperty_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__VerifySafetyProperty_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string property_name\n"
  "---\n"
  "string property_name\n"
  "bool satisfied\n"
  "string details\n"
  "int32 violation_count\n"
  "float64 last_violation";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__VerifySafetyProperty__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__VerifySafetyProperty__TYPE_NAME, 44, 44},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 121, 121},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__VerifySafetyProperty_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__VerifySafetyProperty_Request__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__VerifySafetyProperty_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__VerifySafetyProperty_Response__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__VerifySafetyProperty_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__VerifySafetyProperty_Event__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__VerifySafetyProperty__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__VerifySafetyProperty__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__VerifySafetyProperty_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__VerifySafetyProperty_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__VerifySafetyProperty_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__VerifySafetyProperty_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__VerifySafetyProperty_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__VerifySafetyProperty_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__VerifySafetyProperty_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__VerifySafetyProperty_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__VerifySafetyProperty_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__VerifySafetyProperty_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__VerifySafetyProperty_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
