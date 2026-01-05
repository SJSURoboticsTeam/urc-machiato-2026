// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectAruco__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x73, 0x3c, 0x1e, 0x7e, 0xb6, 0xff, 0xb8, 0x11,
      0xbb, 0x4a, 0x3e, 0x85, 0xe0, 0xe0, 0xc5, 0xfb,
      0xfc, 0x78, 0x6b, 0x2f, 0xe3, 0xa2, 0x70, 0xee,
      0x1a, 0x1e, 0x75, 0x90, 0xb2, 0x2c, 0x23, 0x2e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectAruco_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x61, 0xa8, 0xe2, 0xc2, 0x2f, 0x8a, 0x14, 0xdc,
      0xcf, 0x7b, 0x14, 0x45, 0x4a, 0xd0, 0xc3, 0x76,
      0xea, 0xf7, 0x56, 0x78, 0x37, 0x87, 0x66, 0xbf,
      0x6d, 0xa9, 0x82, 0x9f, 0xce, 0x1c, 0xa5, 0xc6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectAruco_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0xed, 0x7c, 0xf1, 0xaa, 0x7c, 0xcc, 0x12,
      0x49, 0xa0, 0x88, 0x81, 0xa8, 0x39, 0x49, 0xea,
      0xdc, 0x9f, 0xdc, 0x49, 0xda, 0x0a, 0x32, 0xb1,
      0x3b, 0xd4, 0x37, 0x0d, 0x9d, 0xb1, 0x02, 0x10,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectAruco_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb3, 0x9d, 0x63, 0x19, 0x0c, 0xbf, 0xf8, 0xc4,
      0xc9, 0xcf, 0x1f, 0xbd, 0x14, 0xbf, 0xcb, 0xb4,
      0x37, 0x1e, 0x20, 0x7a, 0xea, 0x0f, 0xa4, 0xed,
      0x91, 0xec, 0xc0, 0x32, 0xc1, 0x2f, 0x01, 0x3c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/quaternion__functions.h"
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
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char autonomy_interfaces__srv__DetectAruco__TYPE_NAME[] = "autonomy_interfaces/srv/DetectAruco";
static char autonomy_interfaces__srv__DetectAruco_Event__TYPE_NAME[] = "autonomy_interfaces/srv/DetectAruco_Event";
static char autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME[] = "autonomy_interfaces/srv/DetectAruco_Request";
static char autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME[] = "autonomy_interfaces/srv/DetectAruco_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectAruco__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__DetectAruco__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__DetectAruco__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectAruco__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectAruco_Event__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectAruco__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco_Event__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
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
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__DetectAruco__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectAruco__TYPE_NAME, 35, 35},
      {autonomy_interfaces__srv__DetectAruco__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__DetectAruco__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__DetectAruco_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__DetectAruco_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__DetectAruco_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__target_tag_ids[] = "target_tag_ids";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__detection_timeout[] = "detection_timeout";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__require_distance_estimate[] = "require_distance_estimate";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__max_detection_distance[] = "max_detection_distance";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__calculate_alignment[] = "calculate_alignment";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__target_depth[] = "target_depth";
static char autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__mission_type[] = "mission_type";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectAruco_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__target_tag_ids, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__detection_timeout, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__require_distance_estimate, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__max_detection_distance, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__calculate_alignment, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__target_depth, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Request__FIELD_NAME__mission_type, 12, 12},
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
autonomy_interfaces__srv__DetectAruco_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
      {autonomy_interfaces__srv__DetectAruco_Request__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__detected_tag_ids[] = "detected_tag_ids";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_positions[] = "tag_positions";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_distances[] = "tag_distances";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_angles[] = "tag_angles";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__detection_time[] = "detection_time";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_available[] = "alignment_available";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_center[] = "alignment_center";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_orientation[] = "alignment_orientation";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__arm_target_position[] = "arm_target_position";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_quality[] = "alignment_quality";
static char autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_warnings[] = "alignment_warnings";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectAruco_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__detected_tag_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_positions, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_distances, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__tag_angles, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__detection_time, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_available, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_center, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_orientation, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__arm_target_position, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_quality, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__FIELD_NAME__alignment_warnings, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectAruco_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__DetectAruco_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
      {autonomy_interfaces__srv__DetectAruco_Response__FIELDS, 13, 13},
    },
    {autonomy_interfaces__srv__DetectAruco_Response__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectAruco_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectAruco_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
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
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__srv__DetectAruco_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectAruco_Event__TYPE_NAME, 41, 41},
      {autonomy_interfaces__srv__DetectAruco_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__DetectAruco_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__DetectAruco_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__DetectAruco_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# DetectAruco.srv\n"
  "# Service for ArUco tag detection requests\n"
  "\n"
  "# Request\n"
  "int32[] target_tag_ids  # List of ArUco tag IDs to detect (empty = detect any)\n"
  "float32 detection_timeout  # Timeout in seconds (0.0 = no timeout)\n"
  "bool require_distance_estimate  # Whether to require distance estimation\n"
  "float32 max_detection_distance  # Maximum detection distance in meters (0.0 = no limit)\n"
  "bool calculate_alignment  # Whether to calculate alignment for multi-tag scenarios\n"
  "float32 target_depth  # Target depth for alignment (meters from tag plane)\n"
  "string mission_type  # Mission type: \"TYPING\", \"USB\", \"FOLLOW_ME\", \"GENERAL\"\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success  # Whether detection was successful\n"
  "string message  # Status message\n"
  "int32[] detected_tag_ids  # IDs of detected tags\n"
  "geometry_msgs/Point[] tag_positions  # 3D positions of detected tags\n"
  "float32[] tag_distances  # Estimated distances to tags\n"
  "float32[] tag_angles  # Angles to tags (radians)\n"
  "builtin_interfaces/Time detection_time  # When detection occurred\n"
  "\n"
  "# Alignment information (only if calculate_alignment=true)\n"
  "bool alignment_available  # Whether alignment calculation is available\n"
  "geometry_msgs/Point alignment_center  # Calculated center point of detected tags\n"
  "geometry_msgs/Quaternion alignment_orientation  # Calculated orientation for coplanar alignment\n"
  "geometry_msgs/Point arm_target_position  # Recommended arm position for alignment\n"
  "float32 alignment_quality  # Quality score of alignment (0.0-1.0)\n"
  "string[] alignment_warnings  # Warnings about alignment quality or missing tags";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectAruco__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectAruco__TYPE_NAME, 35, 35},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1536, 1536},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectAruco_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectAruco_Request__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectAruco_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectAruco_Response__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectAruco_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectAruco_Event__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectAruco__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectAruco__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__DetectAruco_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__DetectAruco_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__DetectAruco_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectAruco_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectAruco_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectAruco_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectAruco_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectAruco_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectAruco_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__DetectAruco_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__DetectAruco_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
