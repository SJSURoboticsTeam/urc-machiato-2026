// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/aoi_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__AOIStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa5, 0x25, 0x7d, 0x3c, 0x62, 0x75, 0x95, 0x79,
      0xc4, 0xf0, 0x70, 0x5f, 0x6f, 0x3f, 0x02, 0x44,
      0x5e, 0x0b, 0xa3, 0x58, 0xd4, 0x7b, 0xe7, 0x62,
      0x4b, 0xa8, 0x43, 0x6e, 0x9b, 0xb2, 0xc6, 0x0e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char autonomy_interfaces__msg__AOIStatus__TYPE_NAME[] = "autonomy_interfaces/msg/AOIStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sensor_name[] = "sensor_name";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sensor_type[] = "sensor_type";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__current_aoi[] = "current_aoi";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__average_aoi[] = "average_aoi";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__max_aoi[] = "max_aoi";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__min_aoi[] = "min_aoi";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__is_fresh[] = "is_fresh";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__quality_score[] = "quality_score";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__freshness_status[] = "freshness_status";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__acceptable_threshold[] = "acceptable_threshold";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__optimal_threshold[] = "optimal_threshold";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sample_count[] = "sample_count";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__freshness_ratio[] = "freshness_ratio";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__transport_type[] = "transport_type";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__network_latency[] = "network_latency";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__transport_latency[] = "transport_latency";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__congestion_detected[] = "congestion_detected";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__congestion_factor[] = "congestion_factor";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__predicted_aoi[] = "predicted_aoi";
static char autonomy_interfaces__msg__AOIStatus__FIELD_NAME__aoi_trend[] = "aoi_trend";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__AOIStatus__FIELDS[] = {
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sensor_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sensor_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__current_aoi, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__average_aoi, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__max_aoi, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__min_aoi, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__is_fresh, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__quality_score, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__freshness_status, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__acceptable_threshold, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__optimal_threshold, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__sample_count, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__freshness_ratio, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__transport_type, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__network_latency, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__transport_latency, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__congestion_detected, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__congestion_factor, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__predicted_aoi, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIStatus__FIELD_NAME__aoi_trend, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__AOIStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__msg__AOIStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
      {autonomy_interfaces__msg__AOIStatus__FIELDS, 21, 21},
    },
    {autonomy_interfaces__msg__AOIStatus__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# AOIStatus.msg\n"
  "# Age of Information status for sensor/system monitoring\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Sensor identification\n"
  "string sensor_name         # Name of sensor/system (e.g., \"imu\", \"gps\", \"slam_pose\")\n"
  "string sensor_type         # Type category (e.g., \"sensor\", \"system\", \"fusion\")\n"
  "\n"
  "# AoI metrics (seconds)\n"
  "float64 current_aoi        # Current age of information\n"
  "float64 average_aoi        # Rolling average AoI over last N samples\n"
  "float64 max_aoi           # Maximum AoI observed in window\n"
  "float64 min_aoi           # Minimum AoI observed in window\n"
  "\n"
  "# Quality assessment\n"
  "bool is_fresh            # Whether current data is within acceptable age\n"
  "float64 quality_score     # 0.0-1.0 quality score based on AoI\n"
  "string freshness_status   # \"FRESH\", \"ACCEPTABLE\", \"STALE\", \"CRITICAL\"\n"
  "\n"
  "# Configuration\n"
  "float64 acceptable_threshold # Maximum acceptable AoI (seconds)\n"
  "float64 optimal_threshold    # Optimal AoI threshold (seconds)\n"
  "\n"
  "# Statistics\n"
  "uint32 sample_count       # Number of samples in current window\n"
  "float64 freshness_ratio   # Ratio of fresh samples (0.0-1.0)\n"
  "\n"
  "# Network-aware metrics\n"
  "string transport_type      # Transport type: \"SERIAL\", \"CAN\", \"ETHERNET\", \"LOCAL\"\n"
  "float64 network_latency    # Network transmission latency (seconds)\n"
  "float64 transport_latency  # Transport-specific latency (serial/CAN delays)\n"
  "bool congestion_detected   # Whether network congestion was detected\n"
  "float64 congestion_factor  # Current congestion multiplier (1.0 = no congestion)\n"
  "\n"
  "# Predictive metrics\n"
  "float64 predicted_aoi      # Predicted AOI for next sample\n"
  "float64 aoi_trend          # AOI trend (-1 decreasing, 0 stable, 1 increasing)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__AOIStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__AOIStatus__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1628, 1628},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__AOIStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__AOIStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
