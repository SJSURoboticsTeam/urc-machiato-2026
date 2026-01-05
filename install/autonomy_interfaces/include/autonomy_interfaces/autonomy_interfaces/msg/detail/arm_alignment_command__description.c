// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/arm_alignment_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__ArmAlignmentCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe9, 0xeb, 0xf5, 0xf8, 0x24, 0x03, 0x25, 0x01,
      0x5e, 0x9a, 0x74, 0xe4, 0x02, 0xf8, 0xe0, 0x4b,
      0x01, 0xd4, 0xb7, 0x84, 0xe2, 0x0b, 0xaa, 0xaf,
      0x29, 0xfd, 0x77, 0xd3, 0xc6, 0x52, 0xd2, 0x88,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char autonomy_interfaces__msg__ArmAlignmentCommand__TYPE_NAME[] = "autonomy_interfaces/msg/ArmAlignmentCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__mission_type[] = "mission_type";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_id[] = "alignment_id";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__target_position[] = "target_position";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__target_orientation[] = "target_orientation";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__approach_distance[] = "approach_distance";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__final_distance[] = "final_distance";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_quality[] = "alignment_quality";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_position_error[] = "max_position_error";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_orientation_error[] = "max_orientation_error";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_timeout[] = "alignment_timeout";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_approach_speed[] = "max_approach_speed";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_rotation_speed[] = "max_rotation_speed";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__enable_collision_avoidance[] = "enable_collision_avoidance";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__safety_zones[] = "safety_zones";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__require_position_feedback[] = "require_position_feedback";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__require_force_feedback[] = "require_force_feedback";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__feedback_rate[] = "feedback_rate";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__required_aruco_tags[] = "required_aruco_tags";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__tag_visibility_timeout[] = "tag_visibility_timeout";
static char autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__allow_realignment[] = "allow_realignment";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__ArmAlignmentCommand__FIELDS[] = {
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__mission_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_id, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__target_position, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__target_orientation, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__approach_distance, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__final_distance, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_quality, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_position_error, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_orientation_error, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__alignment_timeout, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_approach_speed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__max_rotation_speed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__enable_collision_avoidance, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__safety_zones, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__require_position_feedback, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__require_force_feedback, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__feedback_rate, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__required_aruco_tags, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__tag_visibility_timeout, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__ArmAlignmentCommand__FIELD_NAME__allow_realignment, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__ArmAlignmentCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__ArmAlignmentCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__ArmAlignmentCommand__TYPE_NAME, 43, 43},
      {autonomy_interfaces__msg__ArmAlignmentCommand__FIELDS, 21, 21},
    },
    {autonomy_interfaces__msg__ArmAlignmentCommand__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ArmAlignmentCommand.msg\n"
  "# Command for arm alignment based on ArUco tag detection\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Mission information\n"
  "string mission_type  # Mission type: \"AUTONOMOUS_TYPING\", \"USB_CONNECTION\", \"PANEL_OPERATIONS\"\n"
  "string alignment_id  # Unique identifier for this alignment operation\n"
  "\n"
  "# Target information\n"
  "geometry_msgs/Point target_position  # Target position for arm end-effector\n"
  "geometry_msgs/Quaternion target_orientation  # Target orientation for arm end-effector\n"
  "float32 approach_distance  # Distance to maintain from target during approach\n"
  "float32 final_distance  # Final distance from target for operation\n"
  "\n"
  "# Alignment parameters\n"
  "float32 alignment_quality  # Quality score of alignment (0.0-1.0)\n"
  "float32 max_position_error  # Maximum allowed position error (meters)\n"
  "float32 max_orientation_error  # Maximum allowed orientation error (radians)\n"
  "float32 alignment_timeout  # Timeout for alignment operation (seconds)\n"
  "\n"
  "# Safety parameters\n"
  "float32 max_approach_speed  # Maximum approach speed (m/s)\n"
  "float32 max_rotation_speed  # Maximum rotation speed (rad/s)\n"
  "bool enable_collision_avoidance  # Whether to enable collision avoidance\n"
  "string[] safety_zones  # Safety zones to avoid during movement\n"
  "\n"
  "# Feedback requirements\n"
  "bool require_position_feedback  # Whether to require position feedback during alignment\n"
  "bool require_force_feedback  # Whether to require force feedback during alignment\n"
  "float32 feedback_rate  # Rate for feedback updates (Hz)\n"
  "\n"
  "# Mission-specific parameters\n"
  "string[] required_aruco_tags  # ArUco tags that must remain visible during operation\n"
  "float32 tag_visibility_timeout  # Timeout for tag visibility loss (seconds)\n"
  "bool allow_realignment  # Whether to allow realignment if tags are lost";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__ArmAlignmentCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__ArmAlignmentCommand__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1725, 1725},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__ArmAlignmentCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__ArmAlignmentCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
