// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x94, 0x75, 0x4f, 0x76, 0x0f, 0x4e, 0xbf, 0x01,
      0x80, 0xd1, 0x80, 0xa5, 0x22, 0x85, 0xdb, 0xda,
      0x88, 0xf2, 0x51, 0x4d, 0x01, 0xea, 0x39, 0xfc,
      0x26, 0xed, 0x79, 0x30, 0x33, 0x0a, 0x76, 0x14,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf0, 0xb4, 0x52, 0x76, 0x58, 0x6a, 0x5e, 0x1b,
      0x2b, 0x4e, 0x7e, 0xf0, 0x78, 0x0d, 0xe9, 0x9e,
      0xb1, 0xb8, 0x8a, 0xb9, 0x02, 0x00, 0x12, 0x3b,
      0x1d, 0x62, 0x49, 0x3f, 0x00, 0x68, 0xdb, 0xb8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1a, 0xbf, 0xad, 0xc0, 0xa3, 0x74, 0xc7, 0xe5,
      0x33, 0x7d, 0xae, 0x28, 0x81, 0x79, 0x3a, 0xcd,
      0x7c, 0xe7, 0x06, 0x3a, 0x71, 0x75, 0xae, 0xdb,
      0xb8, 0x20, 0xea, 0x2d, 0xae, 0xe7, 0x26, 0x7d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x64, 0xe6, 0xf3, 0xcd, 0xda, 0x4a, 0x5b, 0x4a,
      0x8a, 0x96, 0x1e, 0xa8, 0x0a, 0xf1, 0xd6, 0x50,
      0x27, 0xeb, 0x73, 0x75, 0xe0, 0x1b, 0x23, 0x64,
      0x8d, 0x94, 0xb3, 0x94, 0xf8, 0x28, 0xe4, 0x81,
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

static char autonomy_interfaces__srv__NavigationIntegrityCheck__TYPE_NAME[] = "autonomy_interfaces/srv/NavigationIntegrityCheck";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Event__TYPE_NAME[] = "autonomy_interfaces/srv/NavigationIntegrityCheck_Event";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME[] = "autonomy_interfaces/srv/NavigationIntegrityCheck_Request";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME[] = "autonomy_interfaces/srv/NavigationIntegrityCheck_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__NavigationIntegrityCheck__FIELDS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__NavigationIntegrityCheck__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
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
autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__NavigationIntegrityCheck__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__NavigationIntegrityCheck__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__NavigationIntegrityCheck__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELD_NAME__detailed_check[] = "detailed_check";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELD_NAME__check_components[] = "check_components";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELD_NAME__detailed_check, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELD_NAME__check_components, 16, 16},
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_ok[] = "integrity_ok";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_score[] = "integrity_score";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_level[] = "integrity_level";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__checked_components[] = "checked_components";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__component_status[] = "component_status";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__component_details[] = "component_details";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__position_accuracy[] = "position_accuracy";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__heading_accuracy[] = "heading_accuracy";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__velocity_consistency[] = "velocity_consistency";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__satellite_count[] = "satellite_count";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__hdop[] = "hdop";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__recommendations[] = "recommendations";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_ok, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_score, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__integrity_level, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__checked_components, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__component_status, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__component_details, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__position_accuracy, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__heading_accuracy, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__velocity_consistency, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__satellite_count, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__hdop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__recommendations, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELD_NAME__timestamp, 9, 9},
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__NavigationIntegrityCheck_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__TYPE_NAME, 54, 54},
      {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Navigation Integrity Check Service\n"
  "# Checks and reports navigation system integrity\n"
  "\n"
  "# Request\n"
  "bool detailed_check         # Request detailed integrity analysis\n"
  "string[] check_components   # Specific components to check (empty = all)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool integrity_ok           # Overall navigation integrity status\n"
  "float64 integrity_score     # Integrity score (0.0-1.0)\n"
  "string integrity_level      # \"NOMINAL\", \"DEGRADED\", \"CRITICAL\", \"FAILED\"\n"
  "\n"
  "# Component status\n"
  "string[] checked_components # Components that were checked\n"
  "bool[] component_status     # Status of each component\n"
  "string[] component_details  # Detailed status for each component\n"
  "\n"
  "# Metrics\n"
  "float64 position_accuracy   # Current position accuracy (meters)\n"
  "float64 heading_accuracy    # Current heading accuracy (degrees)\n"
  "float64 velocity_consistency # Velocity consistency score (0.0-1.0)\n"
  "int32 satellite_count       # Number of satellites used\n"
  "float64 hdop                # Horizontal dilution of precision\n"
  "\n"
  "# Recommendations\n"
  "string[] recommendations    # Recommended actions if integrity is compromised\n"
  "string timestamp            # When check was performed";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__NavigationIntegrityCheck__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__NavigationIntegrityCheck__TYPE_NAME, 48, 48},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1127, 1127},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Request__TYPE_NAME, 56, 56},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Response__TYPE_NAME, 57, 57},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__NavigationIntegrityCheck_Event__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__NavigationIntegrityCheck__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__NavigationIntegrityCheck_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
