// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_safety_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSafetyStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x0b, 0xcf, 0x46, 0xdb, 0x0a, 0xfb, 0x3d,
      0x9a, 0x8e, 0x5d, 0x60, 0xee, 0x64, 0x08, 0x32,
      0xa5, 0xbe, 0xe9, 0x86, 0x25, 0xcb, 0x09, 0x76,
      0xe0, 0x25, 0x8e, 0xf1, 0xa3, 0xf7, 0x09, 0xc5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa6, 0x08, 0xe4, 0x0e, 0x80, 0xe0, 0xab, 0xe6,
      0x40, 0x6e, 0x98, 0x68, 0x8f, 0xcd, 0xfb, 0x21,
      0xa6, 0x90, 0xfa, 0x99, 0x0b, 0x1b, 0x33, 0x0c,
      0x02, 0xb3, 0xbb, 0x33, 0xb9, 0xfa, 0x0f, 0x23,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x65, 0x5c, 0xd2, 0x63, 0x40, 0x85, 0x70, 0xaf,
      0x3d, 0x7b, 0x6d, 0xf4, 0x43, 0x34, 0x6d, 0x37,
      0xc4, 0x67, 0xb9, 0x5a, 0x62, 0xc2, 0x82, 0xd3,
      0x32, 0x4f, 0x4b, 0x38, 0x7f, 0x79, 0xb7, 0xa2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x58, 0x5b, 0x2b, 0x1c, 0x41, 0x86, 0x87, 0x01,
      0x5c, 0x3b, 0x6d, 0x7f, 0x3f, 0xc5, 0x68, 0x8e,
      0xb2, 0x5b, 0x29, 0x67, 0x2c, 0x5f, 0xa8, 0xb2,
      0x75, 0xa4, 0xe9, 0x9a, 0xfa, 0x33, 0xa8, 0xe0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "autonomy_interfaces/msg/detail/safety_alert__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "autonomy_interfaces/msg/detail/monitoring_stats__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t autonomy_interfaces__msg__MonitoringStats__EXPECTED_HASH = {1, {
    0xb1, 0x3b, 0xa1, 0x92, 0xda, 0xd9, 0x61, 0xd0,
    0xe0, 0x09, 0x78, 0xf0, 0x6b, 0x50, 0x94, 0xd6,
    0x86, 0xa1, 0x64, 0x26, 0xe5, 0xb8, 0x07, 0xfd,
    0x5b, 0x21, 0xdb, 0xb7, 0x5e, 0x9b, 0xe5, 0x4e,
  }};
static const rosidl_type_hash_t autonomy_interfaces__msg__SafetyAlert__EXPECTED_HASH = {1, {
    0x8b, 0xc5, 0x4a, 0xc4, 0x6f, 0x13, 0x9a, 0x46,
    0xbf, 0xa7, 0x76, 0xa9, 0x25, 0x23, 0x64, 0xbf,
    0xee, 0xc6, 0xcd, 0xa6, 0xeb, 0x29, 0xcc, 0x58,
    0x40, 0x33, 0x77, 0xb4, 0x61, 0x31, 0xbd, 0x7d,
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

static char autonomy_interfaces__srv__GetSafetyStatus__TYPE_NAME[] = "autonomy_interfaces/srv/GetSafetyStatus";
static char autonomy_interfaces__msg__MonitoringStats__TYPE_NAME[] = "autonomy_interfaces/msg/MonitoringStats";
static char autonomy_interfaces__msg__SafetyAlert__TYPE_NAME[] = "autonomy_interfaces/msg/SafetyAlert";
static char autonomy_interfaces__srv__GetSafetyStatus_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetSafetyStatus_Event";
static char autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetSafetyStatus_Request";
static char autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetSafetyStatus_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSafetyStatus__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSafetyStatus_Event__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSafetyStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Event__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__GetSafetyStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSafetyStatus__TYPE_NAME, 39, 39},
      {autonomy_interfaces__srv__GetSafetyStatus__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSafetyStatus__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__MonitoringStats__EXPECTED_HASH, autonomy_interfaces__msg__MonitoringStats__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__MonitoringStats__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__SafetyAlert__EXPECTED_HASH, autonomy_interfaces__msg__SafetyAlert__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__SafetyAlert__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSafetyStatus_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSafetyStatus_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__GetSafetyStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__overall_safety[] = "overall_safety";
static char autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__active_alerts[] = "active_alerts";
static char autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__monitoring_stats[] = "monitoring_stats";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSafetyStatus_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__overall_safety, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__active_alerts, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__FIELD_NAME__monitoring_stats, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSafetyStatus_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__GetSafetyStatus_Response__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSafetyStatus_Response__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__MonitoringStats__EXPECTED_HASH, autonomy_interfaces__msg__MonitoringStats__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__MonitoringStats__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__SafetyAlert__EXPECTED_HASH, autonomy_interfaces__msg__SafetyAlert__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__SafetyAlert__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSafetyStatus_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSafetyStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__MonitoringStats__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__SafetyAlert__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSafetyStatus_Event__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__GetSafetyStatus_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSafetyStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__MonitoringStats__EXPECTED_HASH, autonomy_interfaces__msg__MonitoringStats__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__MonitoringStats__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__SafetyAlert__EXPECTED_HASH, autonomy_interfaces__msg__SafetyAlert__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__SafetyAlert__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "string overall_safety\n"
  "SafetyAlert[] active_alerts\n"
  "MonitoringStats monitoring_stats";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSafetyStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSafetyStatus__TYPE_NAME, 39, 39},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSafetyStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSafetyStatus_Request__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSafetyStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSafetyStatus_Response__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSafetyStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSafetyStatus_Event__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSafetyStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSafetyStatus__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__MonitoringStats__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__SafetyAlert__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetSafetyStatus_Event__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetSafetyStatus_Request__get_individual_type_description_source(NULL);
    sources[5] = *autonomy_interfaces__srv__GetSafetyStatus_Response__get_individual_type_description_source(NULL);
    sources[6] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSafetyStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSafetyStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSafetyStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSafetyStatus_Response__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__MonitoringStats__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__SafetyAlert__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSafetyStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSafetyStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__MonitoringStats__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__SafetyAlert__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetSafetyStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetSafetyStatus_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
