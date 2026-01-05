// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetContext.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_context__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetContext__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x37, 0xa5, 0x3e, 0xd1, 0x86, 0xed, 0xfa, 0xc0,
      0x5c, 0xde, 0x51, 0x94, 0x06, 0x71, 0x52, 0xb0,
      0x35, 0x9a, 0xca, 0x32, 0x44, 0x6d, 0x5d, 0xac,
      0x4f, 0x40, 0xb6, 0x49, 0x1a, 0x2c, 0xd8, 0xcb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetContext_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x58, 0x24, 0xbd, 0x9e, 0xf6, 0x0e, 0x43, 0xbf,
      0x7b, 0x75, 0x4d, 0x3f, 0xab, 0x1d, 0x37, 0xa0,
      0xd6, 0x3d, 0xb2, 0xc5, 0xd8, 0x4b, 0x8a, 0xe4,
      0xd2, 0x96, 0xd1, 0xa3, 0xb9, 0x9c, 0x4e, 0xb6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetContext_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcb, 0x54, 0xa0, 0xcb, 0x79, 0xd8, 0xb3, 0x37,
      0x95, 0xaf, 0x4f, 0xb0, 0x45, 0x2f, 0x0c, 0x78,
      0x94, 0x54, 0xc5, 0xc3, 0xc2, 0xec, 0xf9, 0x40,
      0x8b, 0x6b, 0x37, 0x71, 0x14, 0xb1, 0xab, 0x58,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetContext_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xca, 0x0f, 0x9e, 0x8b, 0x24, 0x5a, 0xc8, 0x8a,
      0x87, 0x7c, 0x7c, 0x7e, 0xf9, 0xfd, 0xcb, 0x8e,
      0xdc, 0x04, 0x00, 0xc0, 0x66, 0x72, 0xc4, 0x21,
      0xc9, 0x91, 0x5e, 0x60, 0x8b, 0x1f, 0x5a, 0x43,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "autonomy_interfaces/msg/detail/context_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
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

static char autonomy_interfaces__srv__GetContext__TYPE_NAME[] = "autonomy_interfaces/srv/GetContext";
static char autonomy_interfaces__msg__ContextState__TYPE_NAME[] = "autonomy_interfaces/msg/ContextState";
static char autonomy_interfaces__srv__GetContext_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetContext_Event";
static char autonomy_interfaces__srv__GetContext_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetContext_Request";
static char autonomy_interfaces__srv__GetContext_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetContext_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetContext__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetContext__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetContext__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetContext__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetContext__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetContext_Event__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetContext__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Event__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
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
autonomy_interfaces__srv__GetContext__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetContext__TYPE_NAME, 34, 34},
      {autonomy_interfaces__srv__GetContext__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetContext__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetContext_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetContext_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetContext_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetContext_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetContext_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetContext_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetContext_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
      {autonomy_interfaces__srv__GetContext_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetContext_Response__FIELD_NAME__context[] = "context";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetContext_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetContext_Response__FIELD_NAME__context, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetContext_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__srv__GetContext_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
      {autonomy_interfaces__srv__GetContext_Response__FIELDS, 1, 1},
    },
    {autonomy_interfaces__srv__GetContext_Response__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetContext_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetContext_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__ContextState__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
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
autonomy_interfaces__srv__GetContext_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetContext_Event__TYPE_NAME, 40, 40},
      {autonomy_interfaces__srv__GetContext_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetContext_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__ContextState__EXPECTED_HASH, autonomy_interfaces__msg__ContextState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__ContextState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetContext_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetContext_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetContext.srv\n"
  "# Request current system context\n"
  "\n"
  "---\n"
  "ContextState context";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetContext__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetContext__TYPE_NAME, 34, 34},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 76, 76},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetContext_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetContext_Request__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetContext_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetContext_Response__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetContext_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetContext_Event__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetContext__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetContext__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetContext_Event__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetContext_Request__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetContext_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetContext_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetContext_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetContext_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetContext_Response__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetContext_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetContext_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__ContextState__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetContext_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetContext_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
