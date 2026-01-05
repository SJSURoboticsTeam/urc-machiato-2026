// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__FollowMeControl__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x48, 0x7c, 0xc0, 0xb7, 0x02, 0x02, 0xdb,
      0xc1, 0x0a, 0x5f, 0x9f, 0x6d, 0x92, 0x8a, 0xd7,
      0xf8, 0x72, 0xd7, 0xa8, 0xfd, 0xad, 0x38, 0x69,
      0x36, 0x6f, 0x6e, 0x1a, 0x08, 0x2b, 0x56, 0x01,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__FollowMeControl_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0xdd, 0xc8, 0xc6, 0xb8, 0xac, 0x96, 0x63,
      0xe8, 0x61, 0x7d, 0x87, 0xe1, 0xe1, 0x13, 0x36,
      0x42, 0x21, 0x31, 0x09, 0x09, 0x85, 0xd0, 0xca,
      0x0d, 0xd5, 0x09, 0x33, 0x9d, 0x40, 0x3b, 0x34,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__FollowMeControl_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0xfd, 0xf6, 0xfc, 0xd5, 0x72, 0x17, 0xdf,
      0x65, 0xaa, 0x3f, 0xcd, 0xf9, 0x93, 0x0b, 0xe2,
      0xc4, 0x91, 0xb0, 0xdc, 0x49, 0x0e, 0x86, 0xe6,
      0x4d, 0x49, 0xef, 0x99, 0xb1, 0xe8, 0xf4, 0xcb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__FollowMeControl_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0xed, 0x89, 0x24, 0xe9, 0x23, 0x17, 0x98,
      0xc5, 0x52, 0x21, 0x54, 0xdc, 0x63, 0xf9, 0x75,
      0xbe, 0x5d, 0xcf, 0xf8, 0x63, 0xb9, 0x1d, 0x41,
      0x72, 0xfa, 0xae, 0x86, 0x0e, 0x9f, 0x27, 0x1a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char autonomy_interfaces__srv__FollowMeControl__TYPE_NAME[] = "autonomy_interfaces/srv/FollowMeControl";
static char autonomy_interfaces__srv__FollowMeControl_Event__TYPE_NAME[] = "autonomy_interfaces/srv/FollowMeControl_Event";
static char autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME[] = "autonomy_interfaces/srv/FollowMeControl_Request";
static char autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME[] = "autonomy_interfaces/srv/FollowMeControl_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__FollowMeControl__FIELDS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__FollowMeControl_Event__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__FollowMeControl__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl_Event__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__FollowMeControl__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__FollowMeControl__TYPE_NAME, 39, 39},
      {autonomy_interfaces__srv__FollowMeControl__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__FollowMeControl__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__FollowMeControl_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__FollowMeControl_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__FollowMeControl_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__target_tag_id[] = "target_tag_id";
static char autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__safety_distance[] = "safety_distance";
static char autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__max_speed[] = "max_speed";
static char autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__enable_following[] = "enable_following";
static char autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__operator_id[] = "operator_id";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__FollowMeControl_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__target_tag_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__safety_distance, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__max_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__enable_following, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__FIELD_NAME__operator_id, 11, 11},
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
autonomy_interfaces__srv__FollowMeControl_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__FollowMeControl_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__is_following[] = "is_following";
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__current_target_tag[] = "current_target_tag";
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__current_distance[] = "current_distance";
static char autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__target_position[] = "target_position";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__FollowMeControl_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__is_following, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__current_target_tag, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__current_distance, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__FIELD_NAME__target_position, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__FollowMeControl_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__FollowMeControl_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__FollowMeControl_Response__FIELDS, 6, 6},
    },
    {autonomy_interfaces__srv__FollowMeControl_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__FollowMeControl_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__FollowMeControl_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__FollowMeControl_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__FollowMeControl_Event__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__FollowMeControl_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__FollowMeControl_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__FollowMeControl_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__FollowMeControl_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# FollowMeControl.srv\n"
  "# Service for controlling follow me behavior\n"
  "\n"
  "# Request\n"
  "int32 target_tag_id  # ArUco tag ID to follow\n"
  "float32 safety_distance  # Safe following distance in meters\n"
  "float32 max_speed  # Maximum following speed in m/s\n"
  "bool enable_following  # Start/stop following\n"
  "string operator_id  # ID of operator requesting control\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success  # Whether command was successful\n"
  "string message  # Status message\n"
  "bool is_following  # Current following status\n"
  "int32 current_target_tag  # Currently tracked tag ID\n"
  "float32 current_distance  # Current distance to target\n"
  "geometry_msgs/Point target_position  # Current target position";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__FollowMeControl__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__FollowMeControl__TYPE_NAME, 39, 39},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 652, 652},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__FollowMeControl_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__FollowMeControl_Request__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__FollowMeControl_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__FollowMeControl_Response__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__FollowMeControl_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__FollowMeControl_Event__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__FollowMeControl__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__FollowMeControl__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__FollowMeControl_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__FollowMeControl_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__FollowMeControl_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__FollowMeControl_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__FollowMeControl_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__FollowMeControl_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__FollowMeControl_Response__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__FollowMeControl_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__FollowMeControl_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__FollowMeControl_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__FollowMeControl_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
