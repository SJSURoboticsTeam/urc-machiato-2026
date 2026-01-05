// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__TimingSafetyCheck__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0x20, 0x82, 0x2d, 0xdf, 0x44, 0x9d, 0x52,
      0x57, 0x4a, 0x7e, 0x9c, 0x17, 0x76, 0xe7, 0x4b,
      0xbc, 0x56, 0x3c, 0x97, 0x53, 0x0d, 0x05, 0x49,
      0xf8, 0xb5, 0x54, 0x37, 0x12, 0x2a, 0x40, 0x4a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd8, 0xc2, 0xdc, 0xb2, 0x82, 0xac, 0x4d, 0x57,
      0xd0, 0x4e, 0xbc, 0xd6, 0x1b, 0xaa, 0x7b, 0x75,
      0xfd, 0x8c, 0x6a, 0xed, 0x54, 0x42, 0x5a, 0x45,
      0xd8, 0x84, 0x4a, 0x85, 0x4f, 0x2f, 0xa8, 0x27,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa5, 0x82, 0x9c, 0x12, 0x19, 0x08, 0x35, 0xde,
      0x4b, 0x1e, 0x57, 0x6d, 0x73, 0x39, 0xe8, 0xe5,
      0xcf, 0x22, 0xf7, 0x01, 0x0b, 0x68, 0xf1, 0xc7,
      0x85, 0x19, 0x6b, 0x5b, 0x12, 0xcd, 0xb2, 0x0f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x76, 0xfb, 0x8f, 0xa8, 0xe5, 0x98, 0x36,
      0x54, 0x7a, 0x39, 0x6a, 0x14, 0x95, 0x28, 0x7d,
      0xad, 0x53, 0x54, 0xbd, 0xdb, 0x09, 0xfd, 0xef,
      0x45, 0xed, 0x30, 0xae, 0x6b, 0x6d, 0xbe, 0x2d,
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

static char autonomy_interfaces__srv__TimingSafetyCheck__TYPE_NAME[] = "autonomy_interfaces/srv/TimingSafetyCheck";
static char autonomy_interfaces__srv__TimingSafetyCheck_Event__TYPE_NAME[] = "autonomy_interfaces/srv/TimingSafetyCheck_Event";
static char autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME[] = "autonomy_interfaces/srv/TimingSafetyCheck_Request";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME[] = "autonomy_interfaces/srv/TimingSafetyCheck_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__TimingSafetyCheck__FIELDS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__TimingSafetyCheck_Event__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__TimingSafetyCheck__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
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
autonomy_interfaces__srv__TimingSafetyCheck__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__TimingSafetyCheck__TYPE_NAME, 41, 41},
      {autonomy_interfaces__srv__TimingSafetyCheck__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__TimingSafetyCheck__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__real_time_check[] = "real_time_check";
static char autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__time_window[] = "time_window";
static char autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__monitored_components[] = "monitored_components";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__real_time_check, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__time_window, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELD_NAME__monitored_components, 20, 20},
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
autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
      {autonomy_interfaces__srv__TimingSafetyCheck_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_safe[] = "timing_safe";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_status[] = "timing_status";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__avg_response_time[] = "avg_response_time";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__max_response_time[] = "max_response_time";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__min_response_time[] = "min_response_time";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__jitter[] = "jitter";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__deadline_misses[] = "deadline_misses";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__deadline_miss_rate[] = "deadline_miss_rate";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__components_checked[] = "components_checked";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__component_avg_times[] = "component_avg_times";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__component_deadlines_missed[] = "component_deadlines_missed";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__cpu_utilization[] = "cpu_utilization";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__memory_utilization[] = "memory_utilization";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__thread_count[] = "thread_count";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__real_time_scheduling[] = "real_time_scheduling";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_recommendations[] = "timing_recommendations";
static char autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_safe, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_status, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__avg_response_time, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__max_response_time, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__min_response_time, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__jitter, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__deadline_misses, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__deadline_miss_rate, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__components_checked, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__component_avg_times, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__component_deadlines_missed, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__cpu_utilization, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__memory_utilization, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__thread_count, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__real_time_scheduling, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timing_recommendations, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELD_NAME__timestamp, 9, 9},
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
autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__TimingSafetyCheck_Response__FIELDS, 17, 17},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__TimingSafetyCheck_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
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
autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__TimingSafetyCheck_Event__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__TimingSafetyCheck_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Timing Safety Check Service\n"
  "# Monitors timing constraints and deadlines for safety-critical operations\n"
  "\n"
  "# Request\n"
  "bool real_time_check        # Check real-time performance\n"
  "float64 time_window         # Time window to analyze (seconds)\n"
  "string[] monitored_components # Specific components to check timing for\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool timing_safe            # Overall timing safety status\n"
  "string timing_status        # \"NOMINAL\", \"WARNING\", \"CRITICAL\", \"FAILED\"\n"
  "\n"
  "# Timing metrics\n"
  "float64 avg_response_time   # Average response time (ms)\n"
  "float64 max_response_time   # Maximum response time (ms)\n"
  "float64 min_response_time   # Minimum response time (ms)\n"
  "float64 jitter              # Response time jitter (ms)\n"
  "int32 deadline_misses       # Number of deadline misses in time window\n"
  "float64 deadline_miss_rate  # Rate of deadline misses (per second)\n"
  "\n"
  "# Component timing\n"
  "string[] components_checked # Components that were checked\n"
  "float64[] component_avg_times # Average response times per component\n"
  "int32[] component_deadlines_missed # Deadline misses per component\n"
  "\n"
  "# System health\n"
  "float64 cpu_utilization     # Current CPU utilization (0.0-1.0)\n"
  "float64 memory_utilization  # Current memory utilization (0.0-1.0)\n"
  "int32 thread_count          # Current number of active threads\n"
  "bool real_time_scheduling   # Whether real-time scheduling is active\n"
  "\n"
  "# Recommendations\n"
  "string[] timing_recommendations # Actions to improve timing if needed\n"
  "string timestamp            # When check was performed";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__TimingSafetyCheck__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__TimingSafetyCheck__TYPE_NAME, 41, 41},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1479, 1479},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__TimingSafetyCheck_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__TimingSafetyCheck_Request__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__TimingSafetyCheck_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__TimingSafetyCheck_Response__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__TimingSafetyCheck_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__TimingSafetyCheck_Event__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__TimingSafetyCheck__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__TimingSafetyCheck_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__TimingSafetyCheck_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__TimingSafetyCheck_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__TimingSafetyCheck_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__TimingSafetyCheck_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__TimingSafetyCheck_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__TimingSafetyCheck_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__TimingSafetyCheck_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
