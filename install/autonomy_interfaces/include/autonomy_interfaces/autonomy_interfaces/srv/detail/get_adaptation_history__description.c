// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetAdaptationHistory.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_adaptation_history__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAdaptationHistory__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x00, 0xf3, 0xa8, 0x10, 0xd2, 0x32, 0xca, 0x83,
      0x40, 0x64, 0x05, 0x42, 0xbb, 0x1d, 0x93, 0xd0,
      0xc9, 0x8b, 0xe8, 0xd6, 0x40, 0x73, 0xbf, 0x22,
      0xc9, 0x72, 0x6f, 0x96, 0x31, 0x98, 0xc7, 0x6c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAdaptationHistory_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0x28, 0x85, 0x7c, 0xf0, 0xfa, 0xea, 0x51,
      0xea, 0xa6, 0x31, 0x08, 0xbd, 0xed, 0xbf, 0xff,
      0xf8, 0x84, 0xf2, 0xf2, 0x7e, 0x28, 0x67, 0x58,
      0x64, 0xe9, 0x42, 0xb8, 0xb2, 0xf7, 0xa4, 0x67,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAdaptationHistory_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb3, 0x11, 0x87, 0x77, 0x44, 0x12, 0xa7, 0x02,
      0xab, 0x17, 0x67, 0x6e, 0x9a, 0x2c, 0x25, 0x4f,
      0xb6, 0xcc, 0xb3, 0x36, 0xe9, 0xb2, 0xb7, 0xe8,
      0xa8, 0x72, 0x0b, 0x31, 0xd6, 0x73, 0x15, 0x21,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAdaptationHistory_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0xb9, 0x54, 0x13, 0xdc, 0xf0, 0x2d, 0x85,
      0xaf, 0x57, 0x07, 0x80, 0x0f, 0x3c, 0x3f, 0x15,
      0x8f, 0x55, 0xa8, 0xc6, 0xd9, 0x47, 0x3a, 0xca,
      0x88, 0x22, 0xf4, 0x5f, 0x65, 0x8b, 0xa0, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "autonomy_interfaces/msg/detail/context_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "autonomy_interfaces/msg/detail/adaptive_action__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t autonomy_interfaces__msg__AdaptiveAction__EXPECTED_HASH = {1, {
    0x29, 0x16, 0x3f, 0x12, 0xe6, 0x63, 0x6c, 0x42,
    0x27, 0xe9, 0x6a, 0x39, 0x77, 0xcd, 0x35, 0xe0,
    0x95, 0xb2, 0x80, 0xaa, 0xd1, 0xe7, 0x83, 0x11,
    0x38, 0xd5, 0xa7, 0x6a, 0x49, 0xac, 0xc0, 0xf2,
  }};
static const rosidl_type_hash_t autonomy_interfaces__msg__ContextState__EXPECTED_HASH = {1, {
    0x7b, 0xc8, 0x0e, 0x76, 0x1e, 0xf6, 0x8c, 0xa6,
    0xfd, 0x02, 0x4d, 0x4c, 0x91, 0x3f, 0xfd, 0xf5,
    0x02, 0x8f, 0xa2, 0x6a, 0x6a, 0xa8, 0x1d, 0x32,
    0xad, 0x3f, 0x72, 0x1c, 0xd4, 0x6b, 0x5e, 0x3a,
  }};
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

static char autonomy_interfaces__srv__GetAdaptationHistory__TYPE_NAME[] = "autonomy_interfaces/srv/GetAdaptationHistory";
static char autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME[] = "autonomy_interfaces/msg/AdaptiveAction";
static char autonomy_interfaces__msg__ContextState__TYPE_NAME[] = "autonomy_interfaces/msg/ContextState";
static char autonomy_interfaces__srv__GetAdaptationHistory_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetAdaptationHistory_Event";
static char autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetAdaptationHistory_Request";
static char autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetAdaptationHistory_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAdaptationHistory__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAdaptationHistory_Event__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAdaptationHistory__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
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
autonomy_interfaces__srv__GetAdaptationHistory__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAdaptationHistory__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__GetAdaptationHistory__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetAdaptationHistory__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AdaptiveAction__EXPECTED_HASH, autonomy_interfaces__msg__AdaptiveAction__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AdaptiveAction__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetAdaptationHistory_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetAdaptationHistory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = autonomy_interfaces__srv__GetAdaptationHistory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELD_NAME__limit[] = "limit";
static char autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELD_NAME__include_context[] = "include_context";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELD_NAME__limit, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELD_NAME__include_context, 15, 15},
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
autonomy_interfaces__srv__GetAdaptationHistory_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
      {autonomy_interfaces__srv__GetAdaptationHistory_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELD_NAME__actions[] = "actions";
static char autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELD_NAME__contexts[] = "contexts";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELD_NAME__actions, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELD_NAME__contexts, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAdaptationHistory_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetAdaptationHistory_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
      {autonomy_interfaces__srv__GetAdaptationHistory_Response__FIELDS, 2, 2},
    },
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AdaptiveAction__EXPECTED_HASH, autonomy_interfaces__msg__AdaptiveAction__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AdaptiveAction__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAdaptationHistory_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AdaptiveAction__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
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
autonomy_interfaces__srv__GetAdaptationHistory_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAdaptationHistory_Event__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__GetAdaptationHistory_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AdaptiveAction__EXPECTED_HASH, autonomy_interfaces__msg__AdaptiveAction__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AdaptiveAction__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetAdaptationHistory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetAdaptationHistory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetAdaptationHistory.srv\n"
  "# Request adaptation decision history\n"
  "\n"
  "int32 limit  # Maximum number of entries to return (0 = all)\n"
  "bool include_context  # Include full context with each action\n"
  "\n"
  "---\n"
  "AdaptiveAction[] actions\n"
  "ContextState[] contexts";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAdaptationHistory__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAdaptationHistory__TYPE_NAME, 44, 44},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 243, 243},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAdaptationHistory_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAdaptationHistory_Request__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAdaptationHistory_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAdaptationHistory_Response__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAdaptationHistory_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAdaptationHistory_Event__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAdaptationHistory__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAdaptationHistory__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AdaptiveAction__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetAdaptationHistory_Event__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetAdaptationHistory_Request__get_individual_type_description_source(NULL);
    sources[5] = *autonomy_interfaces__srv__GetAdaptationHistory_Response__get_individual_type_description_source(NULL);
    sources[6] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAdaptationHistory_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAdaptationHistory_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAdaptationHistory_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAdaptationHistory_Response__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AdaptiveAction__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAdaptationHistory_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAdaptationHistory_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AdaptiveAction__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetAdaptationHistory_Request__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetAdaptationHistory_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
