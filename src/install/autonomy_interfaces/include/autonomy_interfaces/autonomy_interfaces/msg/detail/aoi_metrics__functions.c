// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/aoi_metrics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `health_status`
// Member `active_alerts`
// Member `network_recommendations`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_alert_time`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__AOIMetrics__init(autonomy_interfaces__msg__AOIMetrics * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
    return false;
  }
  // system_average_aoi
  // total_sensors
  // fresh_sensors
  // stale_sensors
  // critical_sensors
  // aoi_p50
  // aoi_p90
  // aoi_p95
  // aoi_p99
  // system_healthy
  // health_status
  if (!rosidl_runtime_c__String__init(&msg->health_status)) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
    return false;
  }
  // health_score
  // update_rate_hz
  // dropped_updates
  // processing_latency
  // active_alerts
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_alerts, 0)) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
    return false;
  }
  // alert_count
  // last_alert_time
  if (!builtin_interfaces__msg__Time__init(&msg->last_alert_time)) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
    return false;
  }
  // serial_sensors
  // can_sensors
  // ethernet_sensors
  // local_sensors
  // avg_network_latency
  // max_network_latency
  // congested_links
  // network_recommendations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->network_recommendations, 0)) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
    return false;
  }
  // network_health_score
  return true;
}

void
autonomy_interfaces__msg__AOIMetrics__fini(autonomy_interfaces__msg__AOIMetrics * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // system_average_aoi
  // total_sensors
  // fresh_sensors
  // stale_sensors
  // critical_sensors
  // aoi_p50
  // aoi_p90
  // aoi_p95
  // aoi_p99
  // system_healthy
  // health_status
  rosidl_runtime_c__String__fini(&msg->health_status);
  // health_score
  // update_rate_hz
  // dropped_updates
  // processing_latency
  // active_alerts
  rosidl_runtime_c__String__Sequence__fini(&msg->active_alerts);
  // alert_count
  // last_alert_time
  builtin_interfaces__msg__Time__fini(&msg->last_alert_time);
  // serial_sensors
  // can_sensors
  // ethernet_sensors
  // local_sensors
  // avg_network_latency
  // max_network_latency
  // congested_links
  // network_recommendations
  rosidl_runtime_c__String__Sequence__fini(&msg->network_recommendations);
  // network_health_score
}

bool
autonomy_interfaces__msg__AOIMetrics__are_equal(const autonomy_interfaces__msg__AOIMetrics * lhs, const autonomy_interfaces__msg__AOIMetrics * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // system_average_aoi
  if (lhs->system_average_aoi != rhs->system_average_aoi) {
    return false;
  }
  // total_sensors
  if (lhs->total_sensors != rhs->total_sensors) {
    return false;
  }
  // fresh_sensors
  if (lhs->fresh_sensors != rhs->fresh_sensors) {
    return false;
  }
  // stale_sensors
  if (lhs->stale_sensors != rhs->stale_sensors) {
    return false;
  }
  // critical_sensors
  if (lhs->critical_sensors != rhs->critical_sensors) {
    return false;
  }
  // aoi_p50
  if (lhs->aoi_p50 != rhs->aoi_p50) {
    return false;
  }
  // aoi_p90
  if (lhs->aoi_p90 != rhs->aoi_p90) {
    return false;
  }
  // aoi_p95
  if (lhs->aoi_p95 != rhs->aoi_p95) {
    return false;
  }
  // aoi_p99
  if (lhs->aoi_p99 != rhs->aoi_p99) {
    return false;
  }
  // system_healthy
  if (lhs->system_healthy != rhs->system_healthy) {
    return false;
  }
  // health_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->health_status), &(rhs->health_status)))
  {
    return false;
  }
  // health_score
  if (lhs->health_score != rhs->health_score) {
    return false;
  }
  // update_rate_hz
  if (lhs->update_rate_hz != rhs->update_rate_hz) {
    return false;
  }
  // dropped_updates
  if (lhs->dropped_updates != rhs->dropped_updates) {
    return false;
  }
  // processing_latency
  if (lhs->processing_latency != rhs->processing_latency) {
    return false;
  }
  // active_alerts
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_alerts), &(rhs->active_alerts)))
  {
    return false;
  }
  // alert_count
  if (lhs->alert_count != rhs->alert_count) {
    return false;
  }
  // last_alert_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->last_alert_time), &(rhs->last_alert_time)))
  {
    return false;
  }
  // serial_sensors
  if (lhs->serial_sensors != rhs->serial_sensors) {
    return false;
  }
  // can_sensors
  if (lhs->can_sensors != rhs->can_sensors) {
    return false;
  }
  // ethernet_sensors
  if (lhs->ethernet_sensors != rhs->ethernet_sensors) {
    return false;
  }
  // local_sensors
  if (lhs->local_sensors != rhs->local_sensors) {
    return false;
  }
  // avg_network_latency
  if (lhs->avg_network_latency != rhs->avg_network_latency) {
    return false;
  }
  // max_network_latency
  if (lhs->max_network_latency != rhs->max_network_latency) {
    return false;
  }
  // congested_links
  if (lhs->congested_links != rhs->congested_links) {
    return false;
  }
  // network_recommendations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->network_recommendations), &(rhs->network_recommendations)))
  {
    return false;
  }
  // network_health_score
  if (lhs->network_health_score != rhs->network_health_score) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__AOIMetrics__copy(
  const autonomy_interfaces__msg__AOIMetrics * input,
  autonomy_interfaces__msg__AOIMetrics * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // system_average_aoi
  output->system_average_aoi = input->system_average_aoi;
  // total_sensors
  output->total_sensors = input->total_sensors;
  // fresh_sensors
  output->fresh_sensors = input->fresh_sensors;
  // stale_sensors
  output->stale_sensors = input->stale_sensors;
  // critical_sensors
  output->critical_sensors = input->critical_sensors;
  // aoi_p50
  output->aoi_p50 = input->aoi_p50;
  // aoi_p90
  output->aoi_p90 = input->aoi_p90;
  // aoi_p95
  output->aoi_p95 = input->aoi_p95;
  // aoi_p99
  output->aoi_p99 = input->aoi_p99;
  // system_healthy
  output->system_healthy = input->system_healthy;
  // health_status
  if (!rosidl_runtime_c__String__copy(
      &(input->health_status), &(output->health_status)))
  {
    return false;
  }
  // health_score
  output->health_score = input->health_score;
  // update_rate_hz
  output->update_rate_hz = input->update_rate_hz;
  // dropped_updates
  output->dropped_updates = input->dropped_updates;
  // processing_latency
  output->processing_latency = input->processing_latency;
  // active_alerts
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_alerts), &(output->active_alerts)))
  {
    return false;
  }
  // alert_count
  output->alert_count = input->alert_count;
  // last_alert_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->last_alert_time), &(output->last_alert_time)))
  {
    return false;
  }
  // serial_sensors
  output->serial_sensors = input->serial_sensors;
  // can_sensors
  output->can_sensors = input->can_sensors;
  // ethernet_sensors
  output->ethernet_sensors = input->ethernet_sensors;
  // local_sensors
  output->local_sensors = input->local_sensors;
  // avg_network_latency
  output->avg_network_latency = input->avg_network_latency;
  // max_network_latency
  output->max_network_latency = input->max_network_latency;
  // congested_links
  output->congested_links = input->congested_links;
  // network_recommendations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->network_recommendations), &(output->network_recommendations)))
  {
    return false;
  }
  // network_health_score
  output->network_health_score = input->network_health_score;
  return true;
}

autonomy_interfaces__msg__AOIMetrics *
autonomy_interfaces__msg__AOIMetrics__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIMetrics * msg = (autonomy_interfaces__msg__AOIMetrics *)allocator.allocate(sizeof(autonomy_interfaces__msg__AOIMetrics), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__AOIMetrics));
  bool success = autonomy_interfaces__msg__AOIMetrics__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__AOIMetrics__destroy(autonomy_interfaces__msg__AOIMetrics * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__AOIMetrics__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__AOIMetrics__Sequence__init(autonomy_interfaces__msg__AOIMetrics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIMetrics * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__AOIMetrics *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__AOIMetrics), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__AOIMetrics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__AOIMetrics__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
autonomy_interfaces__msg__AOIMetrics__Sequence__fini(autonomy_interfaces__msg__AOIMetrics__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      autonomy_interfaces__msg__AOIMetrics__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

autonomy_interfaces__msg__AOIMetrics__Sequence *
autonomy_interfaces__msg__AOIMetrics__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIMetrics__Sequence * array = (autonomy_interfaces__msg__AOIMetrics__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__AOIMetrics__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__AOIMetrics__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__AOIMetrics__Sequence__destroy(autonomy_interfaces__msg__AOIMetrics__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__AOIMetrics__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__AOIMetrics__Sequence__are_equal(const autonomy_interfaces__msg__AOIMetrics__Sequence * lhs, const autonomy_interfaces__msg__AOIMetrics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__AOIMetrics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__AOIMetrics__Sequence__copy(
  const autonomy_interfaces__msg__AOIMetrics__Sequence * input,
  autonomy_interfaces__msg__AOIMetrics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__AOIMetrics);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__AOIMetrics * data =
      (autonomy_interfaces__msg__AOIMetrics *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__AOIMetrics__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__AOIMetrics__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__AOIMetrics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
