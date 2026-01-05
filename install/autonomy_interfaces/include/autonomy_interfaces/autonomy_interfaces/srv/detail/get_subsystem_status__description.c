// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetSubsystemStatus.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_subsystem_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSubsystemStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x18, 0x50, 0xb0, 0x3d, 0x29, 0x41, 0x4d, 0xeb,
      0xf9, 0x93, 0x21, 0xee, 0x2f, 0x09, 0xfe, 0xe2,
      0xc7, 0x01, 0x56, 0x89, 0x85, 0xb8, 0xac, 0xb8,
      0xc8, 0xeb, 0xd5, 0xf0, 0xb5, 0x23, 0xb1, 0x54,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSubsystemStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2a, 0x14, 0x49, 0xc5, 0xb8, 0x0a, 0x93, 0xec,
      0xd0, 0xe4, 0xbf, 0xc5, 0xd8, 0xe8, 0x26, 0x53,
      0xc1, 0xc4, 0x47, 0xf8, 0xd8, 0xb7, 0x3c, 0xe3,
      0xba, 0xb8, 0xb4, 0x15, 0x2e, 0x0c, 0x7d, 0x54,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSubsystemStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x82, 0xf0, 0xb3, 0x1b, 0xd7, 0x66, 0xdd,
      0x6d, 0xfc, 0xbd, 0x67, 0x39, 0x86, 0x09, 0xd8,
      0x73, 0x21, 0x19, 0xdc, 0xad, 0xa9, 0x99, 0x59,
      0xdb, 0xea, 0x70, 0x81, 0x27, 0x4f, 0x72, 0x1f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSubsystemStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaa, 0xf4, 0x87, 0x9a, 0xc3, 0x86, 0xac, 0xa0,
      0x67, 0x1d, 0x24, 0x04, 0xff, 0xb4, 0x86, 0x5a,
      0x56, 0xc6, 0x96, 0xa3, 0x34, 0xca, 0xc9, 0xe6,
      0xb2, 0x4c, 0x91, 0xec, 0xc2, 0x5e, 0xb1, 0x34,
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

static char autonomy_interfaces__srv__GetSubsystemStatus__TYPE_NAME[] = "autonomy_interfaces/srv/GetSubsystemStatus";
static char autonomy_interfaces__srv__GetSubsystemStatus_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetSubsystemStatus_Event";
static char autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetSubsystemStatus_Request";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetSubsystemStatus_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSubsystemStatus__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSubsystemStatus_Event__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSubsystemStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
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
autonomy_interfaces__srv__GetSubsystemStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSubsystemStatus__TYPE_NAME, 42, 42},
      {autonomy_interfaces__srv__GetSubsystemStatus__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSubsystemStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__GetSubsystemStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetSubsystemStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetSubsystemStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSubsystemStatus_Request__FIELD_NAME__subsystem_name[] = "subsystem_name";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSubsystemStatus_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Request__FIELD_NAME__subsystem_name, 14, 14},
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
autonomy_interfaces__srv__GetSubsystemStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__GetSubsystemStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__error_message[] = "error_message";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_names[] = "subsystem_names";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_states[] = "subsystem_states";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_health[] = "subsystem_health";
static char autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__status_messages[] = "status_messages";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__error_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_names, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_states, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__subsystem_health, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELD_NAME__status_messages, 15, 15},
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
autonomy_interfaces__srv__GetSubsystemStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
      {autonomy_interfaces__srv__GetSubsystemStatus_Response__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSubsystemStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
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
autonomy_interfaces__srv__GetSubsystemStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSubsystemStatus_Event__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__GetSubsystemStatus_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__GetSubsystemStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetSubsystemStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request subsystem status information\n"
  "\n"
  "string subsystem_name  # \"navigation\", \"slam\", \"computer_vision\", \"autonomous_typing\", \"led_status\", \"all\"\n"
  "\n"
  "---\n"
  "# Response with subsystem status\n"
  "\n"
  "bool success\n"
  "string error_message\n"
  "string[] subsystem_names\n"
  "string[] subsystem_states\n"
  "float32[] subsystem_health  # 0.0 to 1.0 for each subsystem\n"
  "string[] status_messages";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSubsystemStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSubsystemStatus__TYPE_NAME, 42, 42},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 356, 356},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSubsystemStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSubsystemStatus_Request__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSubsystemStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSubsystemStatus_Response__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSubsystemStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSubsystemStatus_Event__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSubsystemStatus__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__GetSubsystemStatus_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetSubsystemStatus_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetSubsystemStatus_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSubsystemStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSubsystemStatus_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSubsystemStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__GetSubsystemStatus_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetSubsystemStatus_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
