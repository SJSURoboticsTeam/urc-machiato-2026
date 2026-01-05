// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectMissionAruco__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1c, 0xef, 0xec, 0x60, 0x35, 0x50, 0x26, 0xaf,
      0x25, 0xcb, 0xc9, 0x8c, 0x3e, 0x45, 0x7c, 0xe2,
      0x80, 0xbd, 0xc6, 0xb4, 0xef, 0xdb, 0x2e, 0xf1,
      0xfd, 0x36, 0x49, 0xd0, 0x2b, 0xa5, 0xc1, 0xe9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0x8b, 0xe6, 0x22, 0x2a, 0xc3, 0xaa, 0x3d,
      0xe6, 0x9e, 0x9c, 0xeb, 0x42, 0xaa, 0x13, 0x05,
      0x0d, 0x47, 0x82, 0x4a, 0x3e, 0x8f, 0x88, 0xb2,
      0xa0, 0xcc, 0xf8, 0xc2, 0xba, 0x0c, 0x9a, 0x89,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbb, 0x18, 0x31, 0x7c, 0x45, 0xf8, 0xad, 0x40,
      0x5c, 0xc3, 0x04, 0xa4, 0x58, 0x48, 0x68, 0xa9,
      0x58, 0xc9, 0x91, 0x4f, 0x52, 0xb8, 0x74, 0x69,
      0x3c, 0x8e, 0x38, 0x4b, 0xba, 0x31, 0xa1, 0x3c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xad, 0xa3, 0x86, 0x75, 0x17, 0x4c, 0xa0, 0x58,
      0x6f, 0xfc, 0xe1, 0xdc, 0xc0, 0x94, 0xe8, 0x84,
      0x52, 0x31, 0x03, 0xe7, 0xa4, 0xc6, 0x4e, 0x57,
      0x6e, 0xef, 0x76, 0x16, 0x5b, 0x9f, 0x89, 0x18,
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

static char autonomy_interfaces__srv__DetectMissionAruco__TYPE_NAME[] = "autonomy_interfaces/srv/DetectMissionAruco";
static char autonomy_interfaces__srv__DetectMissionAruco_Event__TYPE_NAME[] = "autonomy_interfaces/srv/DetectMissionAruco_Event";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME[] = "autonomy_interfaces/srv/DetectMissionAruco_Request";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME[] = "autonomy_interfaces/srv/DetectMissionAruco_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__request_message[] = "request_message";
static char autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__response_message[] = "response_message";
static char autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectMissionAruco__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {autonomy_interfaces__srv__DetectMissionAruco_Event__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectMissionAruco__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Event__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
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
autonomy_interfaces__srv__DetectMissionAruco__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectMissionAruco__TYPE_NAME, 42, 42},
      {autonomy_interfaces__srv__DetectMissionAruco__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__DetectMissionAruco__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description(NULL)->type_description.fields;
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
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__mission_type[] = "mission_type";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__required_tag_ids[] = "required_tag_ids";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__optional_tag_ids[] = "optional_tag_ids";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__detection_timeout[] = "detection_timeout";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__target_depth[] = "target_depth";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__max_detection_distance[] = "max_detection_distance";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__require_all_tags[] = "require_all_tags";
static char autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__min_alignment_quality[] = "min_alignment_quality";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectMissionAruco_Request__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__mission_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__required_tag_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__optional_tag_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__detection_timeout, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__target_depth, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__max_detection_distance, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__require_all_tags, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELD_NAME__min_alignment_quality, 21, 21},
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
autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
      {autonomy_interfaces__srv__DetectMissionAruco_Request__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__success[] = "success";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__message[] = "message";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_type[] = "mission_type";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detected_tag_ids[] = "detected_tag_ids";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_positions[] = "tag_positions";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_distances[] = "tag_distances";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_angles[] = "tag_angles";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detection_time[] = "detection_time";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_available[] = "alignment_available";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_center[] = "alignment_center";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_orientation[] = "alignment_orientation";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__arm_target_position[] = "arm_target_position";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_quality[] = "alignment_quality";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_errors[] = "alignment_errors";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_ready[] = "mission_ready";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__missing_required_tags[] = "missing_required_tags";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detected_optional_tags[] = "detected_optional_tags";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_warnings[] = "alignment_warnings";
static char autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_recommendations[] = "mission_recommendations";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectMissionAruco_Response__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detected_tag_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_positions, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_distances, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__tag_angles, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detection_time, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_available, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_center, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_orientation, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__arm_target_position, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_quality, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_errors, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_ready, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__missing_required_tags, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__detected_optional_tags, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__alignment_warnings, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELD_NAME__mission_recommendations, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectMissionAruco_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
      {autonomy_interfaces__srv__DetectMissionAruco_Response__FIELDS, 19, 19},
    },
    {autonomy_interfaces__srv__DetectMissionAruco_Response__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
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
static char autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__info[] = "info";
static char autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__request[] = "request";
static char autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__srv__DetectMissionAruco_Event__FIELDS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__srv__DetectMissionAruco_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
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
autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__srv__DetectMissionAruco_Event__TYPE_NAME, 48, 48},
      {autonomy_interfaces__srv__DetectMissionAruco_Event__FIELDS, 3, 3},
    },
    {autonomy_interfaces__srv__DetectMissionAruco_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description(NULL)->type_description.fields;
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
  "# DetectMissionAruco.srv\n"
  "# Service for mission-specific ArUco tag detection with auto-alignment\n"
  "\n"
  "# Request\n"
  "string mission_type  # Mission type: \"AUTONOMOUS_TYPING\", \"USB_CONNECTION\", \"PANEL_OPERATIONS\"\n"
  "int32[] required_tag_ids  # Required ArUco tag IDs for this mission\n"
  "int32[] optional_tag_ids  # Optional ArUco tag IDs (improves alignment quality)\n"
  "float32 detection_timeout  # Timeout in seconds (0.0 = no timeout)\n"
  "float32 target_depth  # Target depth for arm positioning (meters from tag plane)\n"
  "float32 max_detection_distance  # Maximum detection distance in meters\n"
  "bool require_all_tags  # Whether all required tags must be detected\n"
  "float32 min_alignment_quality  # Minimum alignment quality threshold (0.0-1.0)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success  # Whether detection and alignment calculation was successful\n"
  "string message  # Status message\n"
  "string mission_type  # Mission type that was processed\n"
  "\n"
  "# Detection results\n"
  "int32[] detected_tag_ids  # IDs of successfully detected tags\n"
  "geometry_msgs/Point[] tag_positions  # 3D positions of detected tags\n"
  "float32[] tag_distances  # Estimated distances to tags\n"
  "float32[] tag_angles  # Angles to tags (radians)\n"
  "builtin_interfaces/Time detection_time  # When detection occurred\n"
  "\n"
  "# Alignment calculation results\n"
  "bool alignment_available  # Whether alignment calculation is available\n"
  "geometry_msgs/Point alignment_center  # Calculated center point of detected tags\n"
  "geometry_msgs/Quaternion alignment_orientation  # Calculated orientation for coplanar alignment\n"
  "geometry_msgs/Point arm_target_position  # Recommended arm position for alignment\n"
  "float32 alignment_quality  # Quality score of alignment (0.0-1.0)\n"
  "float32[] alignment_errors  # Individual alignment errors for each tag\n"
  "\n"
  "# Mission-specific information\n"
  "bool mission_ready  # Whether mission can proceed based on detection results\n"
  "string[] missing_required_tags  # Required tags that were not detected\n"
  "string[] detected_optional_tags  # Optional tags that were detected\n"
  "string[] alignment_warnings  # Warnings about alignment quality or missing tags\n"
  "string[] mission_recommendations  # Recommendations for improving mission success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectMissionAruco__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectMissionAruco__TYPE_NAME, 42, 42},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 2127, 2127},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectMissionAruco_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectMissionAruco_Request__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectMissionAruco_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectMissionAruco_Response__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__srv__DetectMissionAruco_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__srv__DetectMissionAruco_Event__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectMissionAruco__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectMissionAruco__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__DetectMissionAruco_Event__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__DetectMissionAruco_Request__get_individual_type_description_source(NULL);
    sources[3] = *autonomy_interfaces__srv__DetectMissionAruco_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectMissionAruco_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectMissionAruco_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__srv__DetectMissionAruco_Event__get_individual_type_description_source(NULL),
    sources[1] = *autonomy_interfaces__srv__DetectMissionAruco_Request__get_individual_type_description_source(NULL);
    sources[2] = *autonomy_interfaces__srv__DetectMissionAruco_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
