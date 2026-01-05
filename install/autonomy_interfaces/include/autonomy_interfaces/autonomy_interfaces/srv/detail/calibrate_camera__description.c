// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/calibrate_camera__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__CalibrateCamera__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0x86, 0xbf, 0x79, 0x49, 0xd8, 0x95, 0xb5,
      0x2c, 0x3e, 0x3f, 0x8b, 0xd4, 0x11, 0xc0, 0x9c,
      0x2b, 0xd5, 0x3a, 0x64, 0x2a, 0x5c, 0x63, 0x64,
      0x53, 0xc3, 0x17, 0x17, 0x81, 0xd3, 0xe3, 0x2a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__CalibrateCamera_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xda, 0xa5, 0xe5, 0xd6, 0xe8, 0x6f, 0x3b,
      0x73, 0x6d, 0xe6, 0xfd, 0x46, 0xb1, 0x81, 0xd2,
      0x50, 0xfe, 0xfb, 0xdd, 0xc7, 0x69, 0x43, 0x3e,
      0xa1, 0x96, 0xa4, 0x24, 0xaa, 0x3c, 0x41, 0x9f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__CalibrateCamera_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6b, 0xa5, 0xf4, 0x32, 0x15, 0x0e, 0x7f, 0x88,
      0x3c, 0x7b, 0x63, 0x2c, 0x71, 0xe9, 0xad, 0x6b,
      0x65, 0xdf, 0xec, 0x5b, 0xf2, 0xfb, 0x68, 0x28,
      0xef, 0x74, 0x28, 0x0e, 0x8c, 0x5e, 0xa4, 0x3b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__CalibrateCamera_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa6, 0x5d, 0xab, 0xa3, 0xed, 0x56, 0x36, 0xa8,
      0x8f, 0xa9, 0x75, 0xde, 0x0e, 0xce, 0x46, 0x83,
      0x3a, 0xce, 0xdb, 0xe2, 0x68, 0x2a, 0xb3, 0x96,
      0x84, 0x34, 0xc2, 0xd4, 0x2e, 0x52, 0xbe, 0xc5,
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

static char autonomy_interfaces__srv__CalibrateCamera__TYPE_NAME[] = "autonomy_interfaces/srv/CalibrateCamera";
static char autonomy_interfaces__srv__CalibrateCamera_Event__TYPE_NAME[] = "autonomy_interfaces/srv/CalibrateCamera_Event";
static char autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME[] = "autonomy_interfaces/srv/CalibrateCamera_Request";
static char autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME[] = "autonomy_interfaces/srv/CalibrateCamera_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__CalibrateCamera__FIELDS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__CalibrateCamera_Event__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__CalibrateCamera__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera_Event__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__CalibrateCamera__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__CalibrateCamera__TYPE_NAME, 39, 39},
      {autonomy_interfaces__srv__CalibrateCamera__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__CalibrateCamera__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__CalibrateCamera_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__CalibrateCamera_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__CalibrateCamera_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__image_directory[] = "image_directory";
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__board_type[] = "board_type";
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__squares_x[] = "squares_x";
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__squares_y[] = "squares_y";
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__square_size[] = "square_size";
static char autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__marker_size[] = "marker_size";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__CalibrateCamera_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__image_directory, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__board_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__squares_x, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__squares_y, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__square_size, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__FIELD_NAME__marker_size, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__CalibrateCamera_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
      {autonomy_interfaces__srv__CalibrateCamera_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__result_file[] = "result_file";
static char autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__calibration_summary[] = "calibration_summary";
static char autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__error_message[] = "error_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__CalibrateCamera_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__result_file, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__calibration_summary, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__FIELD_NAME__error_message, 13, 13},
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
autonomy_interfaces__srv__CalibrateCamera_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__CalibrateCamera_Response__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__CalibrateCamera_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__CalibrateCamera_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
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
autonomy_interfaces__srv__CalibrateCamera_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__CalibrateCamera_Event__TYPE_NAME, 45, 45},
      {autonomy_interfaces__srv__CalibrateCamera_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__CalibrateCamera_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__CalibrateCamera_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__CalibrateCamera_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request camera calibration with image directory\n"
  "string image_directory\n"
  "string board_type  # \"charuco\" or \"chessboard\"\n"
  "int32 squares_x\n"
  "int32 squares_y\n"
  "float32 square_size\n"
  "float32 marker_size\n"
  "---\n"
  "bool success\n"
  "string result_file\n"
  "string calibration_summary\n"
  "string error_message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__CalibrateCamera__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__CalibrateCamera__TYPE_NAME, 39, 39},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 276, 276},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__CalibrateCamera_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__CalibrateCamera_Request__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__CalibrateCamera_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__CalibrateCamera_Response__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__CalibrateCamera_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__CalibrateCamera_Event__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__CalibrateCamera__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__CalibrateCamera__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__CalibrateCamera_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__CalibrateCamera_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__CalibrateCamera_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__CalibrateCamera_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__CalibrateCamera_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__CalibrateCamera_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__CalibrateCamera_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__CalibrateCamera_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__CalibrateCamera_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__CalibrateCamera_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__CalibrateCamera_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
