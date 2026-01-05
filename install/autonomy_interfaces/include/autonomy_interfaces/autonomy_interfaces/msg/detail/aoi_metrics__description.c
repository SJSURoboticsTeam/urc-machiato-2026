// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice

#include "autonomy_interfaces/msg/detail/aoi_metrics__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__AOIMetrics__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x21, 0x31, 0x1b, 0xdc, 0x48, 0x8d, 0x28, 0xfc,
      0x29, 0x4f, 0xfe, 0x01, 0x9e, 0x4a, 0x4a, 0x03,
      0xce, 0x06, 0xa8, 0x62, 0xd2, 0x63, 0xc3, 0x5e,
      0x28, 0x5a, 0xbf, 0x8d, 0xf0, 0xa2, 0x09, 0x70,
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

static char autonomy_interfaces__msg__AOIMetrics__TYPE_NAME[] = "autonomy_interfaces/msg/AOIMetrics";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__header[] = "header";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__system_average_aoi[] = "system_average_aoi";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__total_sensors[] = "total_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__fresh_sensors[] = "fresh_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__stale_sensors[] = "stale_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__critical_sensors[] = "critical_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p50[] = "aoi_p50";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p90[] = "aoi_p90";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p95[] = "aoi_p95";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p99[] = "aoi_p99";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__system_healthy[] = "system_healthy";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__health_status[] = "health_status";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__health_score[] = "health_score";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__update_rate_hz[] = "update_rate_hz";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__dropped_updates[] = "dropped_updates";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__processing_latency[] = "processing_latency";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__active_alerts[] = "active_alerts";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__alert_count[] = "alert_count";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__last_alert_time[] = "last_alert_time";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__serial_sensors[] = "serial_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__can_sensors[] = "can_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__ethernet_sensors[] = "ethernet_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__local_sensors[] = "local_sensors";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__avg_network_latency[] = "avg_network_latency";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__max_network_latency[] = "max_network_latency";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__congested_links[] = "congested_links";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__network_recommendations[] = "network_recommendations";
static char autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__network_health_score[] = "network_health_score";

static rosidl_runtime_c__type_description__Field autonomy_interfaces__msg__AOIMetrics__FIELDS[] = {
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__system_average_aoi, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__total_sensors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__fresh_sensors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__stale_sensors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__critical_sensors, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p50, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p90, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p95, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__aoi_p99, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__system_healthy, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__health_status, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__health_score, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__update_rate_hz, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__dropped_updates, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__processing_latency, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__active_alerts, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__alert_count, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__last_alert_time, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__serial_sensors, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__can_sensors, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__ethernet_sensors, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__local_sensors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__avg_network_latency, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__max_network_latency, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__congested_links, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__network_recommendations, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {autonomy_interfaces__msg__AOIMetrics__FIELD_NAME__network_health_score, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription autonomy_interfaces__msg__AOIMetrics__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
autonomy_interfaces__msg__AOIMetrics__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
      {autonomy_interfaces__msg__AOIMetrics__FIELDS, 28, 28},
    },
    {autonomy_interfaces__msg__AOIMetrics__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# AOIMetrics.msg\n"
  "# Detailed AoI metrics for system monitoring and diagnostics\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# System-wide AoI summary\n"
  "float64 system_average_aoi    # Average AoI across all monitored components\n"
  "uint32 total_sensors         # Total number of sensors being monitored\n"
  "uint32 fresh_sensors         # Number of sensors with fresh data\n"
  "uint32 stale_sensors         # Number of sensors with stale data\n"
  "uint32 critical_sensors      # Number of sensors with critical AoI\n"
  "\n"
  "# AoI distribution (percentiles)\n"
  "float64 aoi_p50             # 50th percentile AoI (median)\n"
  "float64 aoi_p90             # 90th percentile AoI\n"
  "float64 aoi_p95             # 95th percentile AoI\n"
  "float64 aoi_p99             # 99th percentile AoI\n"
  "\n"
  "# System health indicators\n"
  "bool system_healthy         # Overall system AoI health\n"
  "string health_status        # \"HEALTHY\", \"WARNING\", \"CRITICAL\"\n"
  "float64 health_score        # 0.0-1.0 overall health score\n"
  "\n"
  "# Performance metrics\n"
  "float64 update_rate_hz      # Rate of AoI status updates\n"
  "uint32 dropped_updates      # Number of updates dropped due to high load\n"
  "float64 processing_latency  # Average processing latency (seconds)\n"
  "\n"
  "# Alert information\n"
  "string[] active_alerts      # List of active AoI alerts\n"
  "uint32 alert_count          # Number of active alerts\n"
  "builtin_interfaces/Time last_alert_time  # Time of last alert\n"
  "\n"
  "# Network health metrics\n"
  "uint32 serial_sensors       # Number of sensors using serial transport\n"
  "uint32 can_sensors         # Number of sensors using CAN transport\n"
  "uint32 ethernet_sensors    # Number of sensors using ethernet transport\n"
  "uint32 local_sensors       # Number of sensors using local transport\n"
  "\n"
  "float64 avg_network_latency # Average network latency across all sensors\n"
  "float64 max_network_latency # Maximum network latency observed\n"
  "uint32 congested_links     # Number of transport links with congestion\n"
  "\n"
  "# System recommendations\n"
  "string[] network_recommendations # Recommendations for network optimization\n"
  "float64 network_health_score    # 0.0-1.0 network health score";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__AOIMetrics__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {autonomy_interfaces__msg__AOIMetrics__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2013, 2013},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__AOIMetrics__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *autonomy_interfaces__msg__AOIMetrics__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
