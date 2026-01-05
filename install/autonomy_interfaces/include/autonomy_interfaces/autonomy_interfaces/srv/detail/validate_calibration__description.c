// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/validate_calibration__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ValidateCalibration__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x38, 0x28, 0x81, 0x74, 0x4b, 0xae, 0x31, 0x26,
      0x82, 0xed, 0xf8, 0x4c, 0x77, 0xe6, 0x20, 0xcd,
      0x6d, 0xfa, 0xab, 0xe8, 0x15, 0x24, 0x4b, 0x66,
      0xfe, 0xf2, 0x3c, 0x19, 0x97, 0xf0, 0xf6, 0x82,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ValidateCalibration_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x19, 0x31, 0x2b, 0x83, 0x3f, 0xd8, 0x04,
      0xd0, 0x9f, 0x18, 0xbb, 0x93, 0x1f, 0x06, 0x03,
      0x98, 0x16, 0xd8, 0xbc, 0x4d, 0xae, 0x01, 0xf1,
      0xbc, 0x75, 0x54, 0x18, 0xd4, 0x88, 0x1b, 0x2c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ValidateCalibration_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xde, 0xe7, 0xcf, 0x38, 0x52, 0x53, 0x54, 0x77,
      0x31, 0x87, 0x26, 0x25, 0x2b, 0x14, 0xc4, 0x31,
      0xc1, 0xb9, 0xf1, 0xd4, 0xf5, 0x2d, 0x00, 0x7b,
      0x0c, 0xc1, 0xaa, 0xf5, 0x2b, 0x31, 0x5a, 0x54,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__ValidateCalibration_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0xe1, 0x1b, 0x5b, 0xb7, 0x60, 0x3d, 0x58,
      0x78, 0x59, 0x39, 0xe6, 0x78, 0xda, 0xad, 0x02,
      0xd9, 0x16, 0x4f, 0xf6, 0x72, 0xc1, 0xda, 0x9e,
      0x6e, 0x43, 0x57, 0xb6, 0xda, 0x64, 0x08, 0x69,
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

static char autonomy_interfaces__srv__ValidateCalibration__TYPE_NAME[] = "autonomy_interfaces/srv/ValidateCalibration";
static char autonomy_interfaces__srv__ValidateCalibration_Event__TYPE_NAME[] = "autonomy_interfaces/srv/ValidateCalibration_Event";
static char autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME[] = "autonomy_interfaces/srv/ValidateCalibration_Request";
static char autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME[] = "autonomy_interfaces/srv/ValidateCalibration_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ValidateCalibration__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__ValidateCalibration_Event__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__ValidateCalibration__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration_Event__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
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
autonomy_interfaces__srv__ValidateCalibration__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ValidateCalibration__TYPE_NAME, 43, 43},
      {autonomy_interfaces__srv__ValidateCalibration__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__ValidateCalibration__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__ValidateCalibration_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__ValidateCalibration_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__ValidateCalibration_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ValidateCalibration_Request__FIELD_NAME__calibration_file[] = "calibration_file";
static char autonomy_interfaces__srv__ValidateCalibration_Request__FIELD_NAME__test_images[] = "test_images";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ValidateCalibration_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration_Request__FIELD_NAME__calibration_file, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Request__FIELD_NAME__test_images, 11, 11},
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
autonomy_interfaces__srv__ValidateCalibration_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
      {autonomy_interfaces__srv__ValidateCalibration_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__reprojection_error[] = "reprojection_error";
static char autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__quality_assessment[] = "quality_assessment";
static char autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__recommendations[] = "recommendations";
static char autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__error_message[] = "error_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ValidateCalibration_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__reprojection_error, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__quality_assessment, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__recommendations, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__FIELD_NAME__error_message, 13, 13},
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
autonomy_interfaces__srv__ValidateCalibration_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
      {autonomy_interfaces__srv__ValidateCalibration_Response__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__ValidateCalibration_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__ValidateCalibration_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
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
autonomy_interfaces__srv__ValidateCalibration_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__ValidateCalibration_Event__TYPE_NAME, 49, 49},
      {autonomy_interfaces__srv__ValidateCalibration_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__ValidateCalibration_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__ValidateCalibration_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__ValidateCalibration_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Validate existing calibration quality\n"
  "string calibration_file\n"
  "string[] test_images\n"
  "---\n"
  "bool success\n"
  "float32 reprojection_error\n"
  "string quality_assessment\n"
  "string recommendations\n"
  "string error_message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ValidateCalibration__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ValidateCalibration__TYPE_NAME, 43, 43},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 199, 199},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ValidateCalibration_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ValidateCalibration_Request__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ValidateCalibration_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ValidateCalibration_Response__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__ValidateCalibration_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__ValidateCalibration_Event__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ValidateCalibration__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ValidateCalibration__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__ValidateCalibration_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__ValidateCalibration_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__ValidateCalibration_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ValidateCalibration_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ValidateCalibration_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ValidateCalibration_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ValidateCalibration_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__ValidateCalibration_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__ValidateCalibration_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__ValidateCalibration_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__ValidateCalibration_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
