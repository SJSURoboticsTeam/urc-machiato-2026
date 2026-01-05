// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/get_system_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSystemState__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0x7b, 0xc8, 0xca, 0x04, 0x08, 0x80, 0x8e,
      0xea, 0xf1, 0x83, 0xde, 0x99, 0x2b, 0x0c, 0xa8,
      0xa7, 0x0d, 0x02, 0xcb, 0xf7, 0x99, 0x17, 0xb7,
      0xfd, 0x4a, 0xec, 0x9f, 0xd1, 0x27, 0x4e, 0x80,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSystemState_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb0, 0x38, 0xce, 0xd3, 0xeb, 0x2d, 0x93, 0x57,
      0xb9, 0x37, 0x98, 0x9c, 0xf7, 0x28, 0x49, 0xa6,
      0xda, 0x46, 0xc0, 0x89, 0x80, 0xfd, 0xde, 0xb3,
      0x54, 0x82, 0x23, 0xda, 0x10, 0x4c, 0xbc, 0x21,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSystemState_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x66, 0xcd, 0x60, 0xda, 0x63, 0x36, 0x59, 0xab,
      0x84, 0xaf, 0x8c, 0x5e, 0x16, 0x7b, 0x9e, 0x52,
      0x7e, 0xa0, 0xec, 0xed, 0x33, 0xc4, 0x00, 0xb1,
      0x18, 0x2c, 0xb5, 0x5c, 0x83, 0xf2, 0x24, 0xe8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__GetSystemState_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0x1a, 0xb2, 0x5d, 0x85, 0x21, 0xde, 0x8d,
      0xd3, 0x5a, 0xf0, 0x91, 0x59, 0x0f, 0x97, 0x8d,
      0x28, 0x69, 0x70, 0xd2, 0x85, 0x6c, 0x0e, 0x53,
      0xb0, 0x9f, 0x0f, 0x3c, 0x62, 0x87, 0x18, 0xa6,
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

static char autonomy_interfaces__srv__GetSystemState__TYPE_NAME[] = "autonomy_interfaces/srv/GetSystemState";
static char autonomy_interfaces__srv__GetSystemState_Event__TYPE_NAME[] = "autonomy_interfaces/srv/GetSystemState_Event";
static char autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME[] = "autonomy_interfaces/srv/GetSystemState_Request";
static char autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME[] = "autonomy_interfaces/srv/GetSystemState_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSystemState__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__GetSystemState__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__GetSystemState__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSystemState__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__GetSystemState_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSystemState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
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
autonomy_interfaces__srv__GetSystemState__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSystemState__TYPE_NAME, 38, 38},
      {autonomy_interfaces__srv__GetSystemState__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSystemState__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__GetSystemState_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetSystemState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__GetSystemState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__include_history[] = "include_history";
static char autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__include_subsystems[] = "include_subsystems";
static char autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__history_limit[] = "history_limit";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSystemState_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__include_history, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__include_subsystems, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Request__FIELD_NAME__history_limit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetSystemState_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
      {autonomy_interfaces__srv__GetSystemState_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__current_state[] = "current_state";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__substate[] = "substate";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__sub_substate[] = "sub_substate";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__time_in_state[] = "time_in_state";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__state_entered[] = "state_entered";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__recent_states[] = "recent_states";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__state_timestamps[] = "state_timestamps";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__transition_reasons[] = "transition_reasons";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__active_subsystems[] = "active_subsystems";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__inactive_subsystems[] = "inactive_subsystems";
static char autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__failed_subsystems[] = "failed_subsystems";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSystemState_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__current_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__substate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__sub_substate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__time_in_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__state_entered, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__recent_states, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__state_timestamps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__transition_reasons, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__active_subsystems, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__inactive_subsystems, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__FIELD_NAME__failed_subsystems, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSystemState_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__GetSystemState_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__GetSystemState_Response__FIELDS, 13, 13},
    },
    {autonomy_interfaces__srv__GetSystemState_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__GetSystemState_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__GetSystemState_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
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
autonomy_interfaces__srv__GetSystemState_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__GetSystemState_Event__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__GetSystemState_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__GetSystemState_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__GetSystemState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__GetSystemState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Query current system state\n"
  "\n"
  "# Request\n"
  "bool include_history             # If true, include recent state history\n"
  "bool include_subsystems          # If true, include subsystem status details\n"
  "int32 history_limit              # Number of recent transitions to include (default: 10)\n"
  "\n"
  "---\n"
  "# Response\n"
  "\n"
  "bool success                     # True if query successful\n"
  "string message                   # Status message\n"
  "\n"
  "# Current state information\n"
  "string current_state             # Current top-level state\n"
  "string substate                  # Current substate\n"
  "string sub_substate             # Current sub-substate\n"
  "float64 time_in_state           # Seconds in current state\n"
  "builtin_interfaces/Time state_entered  # When current state was entered\n"
  "\n"
  "# History (if requested)\n"
  "string[] recent_states           # List of recent states\n"
  "builtin_interfaces/Time[] state_timestamps  # Timestamps for each state\n"
  "string[] transition_reasons      # Reasons for each transition\n"
  "\n"
  "# Subsystem status (if requested)\n"
  "string[] active_subsystems       # Currently active subsystems\n"
  "string[] inactive_subsystems     # Inactive subsystems\n"
  "string[] failed_subsystems       # Failed/error subsystems";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSystemState__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSystemState__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1162, 1162},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSystemState_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSystemState_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSystemState_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSystemState_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__GetSystemState_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__GetSystemState_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSystemState__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSystemState__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__GetSystemState_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetSystemState_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__GetSystemState_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSystemState_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSystemState_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSystemState_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSystemState_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__GetSystemState_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__GetSystemState_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__GetSystemState_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__GetSystemState_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
