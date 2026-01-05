// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_aoi_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAOIStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0x97, 0xcc, 0x7f, 0xdc, 0xb3, 0xd9, 0xb1,
      0xf3, 0x81, 0x6a, 0xb4, 0xb6, 0x21, 0x09, 0xe4,
      0x78, 0xe0, 0xcf, 0x7c, 0x32, 0x54, 0x19, 0x75,
      0xa4, 0x5c, 0x20, 0x43, 0xad, 0x9f, 0x0f, 0x3e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAOIStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0xea, 0xd5, 0x30, 0x49, 0x64, 0xb5, 0x10,
      0x96, 0x45, 0x47, 0xeb, 0x2a, 0x50, 0xfc, 0x30,
      0xa0, 0xb0, 0x09, 0x93, 0xb6, 0xa5, 0x29, 0xdd,
      0xa2, 0x70, 0x1f, 0x6d, 0x4b, 0xd3, 0x9c, 0x22,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAOIStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x21, 0x88, 0x4d, 0x25, 0x0a, 0xa6, 0x7c, 0xab,
      0x98, 0x3e, 0x77, 0x55, 0xd4, 0xb7, 0x27, 0x73,
      0x80, 0xdb, 0x50, 0x44, 0x19, 0x27, 0x10, 0x9f,
      0x0a, 0xb5, 0x62, 0x14, 0xf0, 0x7d, 0xf1, 0x95,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetAOIStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe9, 0x6f, 0xf6, 0xaf, 0x1b, 0xaf, 0x7b, 0xaf,
      0x14, 0xca, 0x8f, 0xe8, 0x20, 0x8d, 0xd9, 0xfe,
      0x67, 0x0a, 0xcd, 0xea, 0xbe, 0xc2, 0xc7, 0x2c,
      0xfd, 0xa6, 0xf8, 0x9a, 0x7e, 0xca, 0x83, 0x53,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "autonomy_interfaces/msg/detail/aoi_metrics__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "autonomy_interfaces/msg/detail/aoi_status__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t autonomy_interfaces__msg__AOIMetrics__EXPECTED_HASH = {1, {
    0x21, 0x31, 0x1b, 0xdc, 0x48, 0x8d, 0x28, 0xfc,
    0x29, 0x4f, 0xfe, 0x01, 0x9e, 0x4a, 0x4a, 0x03,
    0xce, 0x06, 0xa8, 0x62, 0xd2, 0x63, 0xc3, 0x5e,
    0x28, 0x5a, 0xbf, 0x8d, 0xf0, 0xa2, 0x09, 0x70,
  }};
static const rosidl_type_hash_t autonomy_interfaces__msg__AOIStatus__EXPECTED_HASH = {1, {
    0xa5, 0x25, 0x7d, 0x3c, 0x62, 0x75, 0x95, 0x79,
    0xc4, 0xf0, 0x70, 0x5f, 0x6f, 0x3f, 0x02, 0x44,
    0x5e, 0x0b, 0xa3, 0x58, 0xd4, 0x7b, 0xe7, 0x62,
    0x4b, 0xa8, 0x43, 0x6e, 0x9b, 0xb2, 0xc6, 0x0e,
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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char autonomy_interfaces__srv__GetAOIStatus__TYPE_NAME[] = "autonomy_interfaces/srv/GetAOIStatus";
static char autonomy_interfaces__msg__AOIMetrics__TYPE_NAME[] = "autonomy_interfaces/msg/AOIMetrics";
static char autonomy_interfaces__msg__AOIStatus__TYPE_NAME[] = "autonomy_interfaces/msg/AOIStatus";
static char autonomy_interfaces__srv__GetAOIStatus_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetAOIStatus_Event";
static char autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetAOIStatus_Request";
static char autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetAOIStatus_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAOIStatus__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetAOIStatus_Event__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAOIStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Event__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
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
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetAOIStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAOIStatus__TYPE_NAME, 36, 36},
      {autonomy_interfaces__srv__GetAOIStatus__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetAOIStatus__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIMetrics__EXPECTED_HASH, autonomy_interfaces__msg__AOIMetrics__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AOIMetrics__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIStatus__EXPECTED_HASH, autonomy_interfaces__msg__AOIStatus__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__AOIStatus__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetAOIStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__sensor_name[] = "sensor_name";
static char autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__include_history[] = "include_history";
static char autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__history_samples[] = "history_samples";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAOIStatus_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__sensor_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__include_history, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Request__FIELD_NAME__history_samples, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__GetAOIStatus_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__sensor_status[] = "sensor_status";
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__aoi_history[] = "aoi_history";
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__timestamp_history[] = "timestamp_history";
static char autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__system_metrics[] = "system_metrics";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAOIStatus_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__sensor_status, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__aoi_history, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__timestamp_history, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__FIELD_NAME__system_metrics, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAOIStatus_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
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
autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__GetAOIStatus_Response__FIELDS, 6, 6},
    },
    {autonomy_interfaces__srv__GetAOIStatus_Response__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIMetrics__EXPECTED_HASH, autonomy_interfaces__msg__AOIMetrics__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AOIMetrics__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIStatus__EXPECTED_HASH, autonomy_interfaces__msg__AOIStatus__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__AOIStatus__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetAOIStatus_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetAOIStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
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
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetAOIStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetAOIStatus_Event__TYPE_NAME, 42, 42},
      {autonomy_interfaces__srv__GetAOIStatus_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetAOIStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIMetrics__EXPECTED_HASH, autonomy_interfaces__msg__AOIMetrics__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__msg__AOIMetrics__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&autonomy_interfaces__msg__AOIStatus__EXPECTED_HASH, autonomy_interfaces__msg__AOIStatus__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__msg__AOIStatus__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetAOIStatus.srv\n"
  "# Service to query AoI status for sensors/systems\n"
  "\n"
  "# Request\n"
  "string sensor_name          # Specific sensor to query (empty for all)\n"
  "bool include_history        # Whether to include historical data\n"
  "uint32 history_samples      # Number of historical samples to return (0 = current only)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success               # Whether query was successful\n"
  "string message             # Status message\n"
  "\n"
  "# Current status\n"
  "autonomy_interfaces/AOIStatus[] sensor_status  # Status for each sensor\n"
  "\n"
  "# Historical data (if requested)\n"
  "float64[] aoi_history       # Historical AoI values\n"
  "builtin_interfaces/Time[] timestamp_history  # Corresponding timestamps\n"
  "\n"
  "# System summary\n"
  "autonomy_interfaces/AOIMetrics system_metrics  # Overall system AoI metrics";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAOIStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAOIStatus__TYPE_NAME, 36, 36},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 764, 764},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAOIStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAOIStatus_Request__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAOIStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAOIStatus_Response__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetAOIStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetAOIStatus_Event__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAOIStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAOIStatus__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AOIMetrics__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__AOIStatus__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetAOIStatus_Event__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetAOIStatus_Request__get_individual_type_description_source(NULL);
    sources[5] = *autonomy_interfaces__srv__GetAOIStatus_Response__get_individual_type_description_source(NULL);
    sources[6] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAOIStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAOIStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAOIStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAOIStatus_Response__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AOIMetrics__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__AOIStatus__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetAOIStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetAOIStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__msg__AOIMetrics__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__msg__AOIStatus__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetAOIStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *autonomy_interfaces__srv__GetAOIStatus_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
