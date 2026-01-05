// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/aoi_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sensor_name`
// Member `sensor_type`
// Member `freshness_status`
// Member `transport_type`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__AOIStatus__init(autonomy_interfaces__msg__AOIStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__init(&msg->sensor_name)) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
    return false;
  }
  // sensor_type
  if (!rosidl_runtime_c__String__init(&msg->sensor_type)) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
    return false;
  }
  // current_aoi
  // average_aoi
  // max_aoi
  // min_aoi
  // is_fresh
  // quality_score
  // freshness_status
  if (!rosidl_runtime_c__String__init(&msg->freshness_status)) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
    return false;
  }
  // acceptable_threshold
  // optimal_threshold
  // sample_count
  // freshness_ratio
  // transport_type
  if (!rosidl_runtime_c__String__init(&msg->transport_type)) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
    return false;
  }
  // network_latency
  // transport_latency
  // congestion_detected
  // congestion_factor
  // predicted_aoi
  // aoi_trend
  return true;
}

void
autonomy_interfaces__msg__AOIStatus__fini(autonomy_interfaces__msg__AOIStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sensor_name
  rosidl_runtime_c__String__fini(&msg->sensor_name);
  // sensor_type
  rosidl_runtime_c__String__fini(&msg->sensor_type);
  // current_aoi
  // average_aoi
  // max_aoi
  // min_aoi
  // is_fresh
  // quality_score
  // freshness_status
  rosidl_runtime_c__String__fini(&msg->freshness_status);
  // acceptable_threshold
  // optimal_threshold
  // sample_count
  // freshness_ratio
  // transport_type
  rosidl_runtime_c__String__fini(&msg->transport_type);
  // network_latency
  // transport_latency
  // congestion_detected
  // congestion_factor
  // predicted_aoi
  // aoi_trend
}

bool
autonomy_interfaces__msg__AOIStatus__are_equal(const autonomy_interfaces__msg__AOIStatus * lhs, const autonomy_interfaces__msg__AOIStatus * rhs)
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
  // sensor_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sensor_name), &(rhs->sensor_name)))
  {
    return false;
  }
  // sensor_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sensor_type), &(rhs->sensor_type)))
  {
    return false;
  }
  // current_aoi
  if (lhs->current_aoi != rhs->current_aoi) {
    return false;
  }
  // average_aoi
  if (lhs->average_aoi != rhs->average_aoi) {
    return false;
  }
  // max_aoi
  if (lhs->max_aoi != rhs->max_aoi) {
    return false;
  }
  // min_aoi
  if (lhs->min_aoi != rhs->min_aoi) {
    return false;
  }
  // is_fresh
  if (lhs->is_fresh != rhs->is_fresh) {
    return false;
  }
  // quality_score
  if (lhs->quality_score != rhs->quality_score) {
    return false;
  }
  // freshness_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->freshness_status), &(rhs->freshness_status)))
  {
    return false;
  }
  // acceptable_threshold
  if (lhs->acceptable_threshold != rhs->acceptable_threshold) {
    return false;
  }
  // optimal_threshold
  if (lhs->optimal_threshold != rhs->optimal_threshold) {
    return false;
  }
  // sample_count
  if (lhs->sample_count != rhs->sample_count) {
    return false;
  }
  // freshness_ratio
  if (lhs->freshness_ratio != rhs->freshness_ratio) {
    return false;
  }
  // transport_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->transport_type), &(rhs->transport_type)))
  {
    return false;
  }
  // network_latency
  if (lhs->network_latency != rhs->network_latency) {
    return false;
  }
  // transport_latency
  if (lhs->transport_latency != rhs->transport_latency) {
    return false;
  }
  // congestion_detected
  if (lhs->congestion_detected != rhs->congestion_detected) {
    return false;
  }
  // congestion_factor
  if (lhs->congestion_factor != rhs->congestion_factor) {
    return false;
  }
  // predicted_aoi
  if (lhs->predicted_aoi != rhs->predicted_aoi) {
    return false;
  }
  // aoi_trend
  if (lhs->aoi_trend != rhs->aoi_trend) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__AOIStatus__copy(
  const autonomy_interfaces__msg__AOIStatus * input,
  autonomy_interfaces__msg__AOIStatus * output)
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
  // sensor_name
  if (!rosidl_runtime_c__String__copy(
      &(input->sensor_name), &(output->sensor_name)))
  {
    return false;
  }
  // sensor_type
  if (!rosidl_runtime_c__String__copy(
      &(input->sensor_type), &(output->sensor_type)))
  {
    return false;
  }
  // current_aoi
  output->current_aoi = input->current_aoi;
  // average_aoi
  output->average_aoi = input->average_aoi;
  // max_aoi
  output->max_aoi = input->max_aoi;
  // min_aoi
  output->min_aoi = input->min_aoi;
  // is_fresh
  output->is_fresh = input->is_fresh;
  // quality_score
  output->quality_score = input->quality_score;
  // freshness_status
  if (!rosidl_runtime_c__String__copy(
      &(input->freshness_status), &(output->freshness_status)))
  {
    return false;
  }
  // acceptable_threshold
  output->acceptable_threshold = input->acceptable_threshold;
  // optimal_threshold
  output->optimal_threshold = input->optimal_threshold;
  // sample_count
  output->sample_count = input->sample_count;
  // freshness_ratio
  output->freshness_ratio = input->freshness_ratio;
  // transport_type
  if (!rosidl_runtime_c__String__copy(
      &(input->transport_type), &(output->transport_type)))
  {
    return false;
  }
  // network_latency
  output->network_latency = input->network_latency;
  // transport_latency
  output->transport_latency = input->transport_latency;
  // congestion_detected
  output->congestion_detected = input->congestion_detected;
  // congestion_factor
  output->congestion_factor = input->congestion_factor;
  // predicted_aoi
  output->predicted_aoi = input->predicted_aoi;
  // aoi_trend
  output->aoi_trend = input->aoi_trend;
  return true;
}

autonomy_interfaces__msg__AOIStatus *
autonomy_interfaces__msg__AOIStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIStatus * msg = (autonomy_interfaces__msg__AOIStatus *)allocator.allocate(sizeof(autonomy_interfaces__msg__AOIStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__AOIStatus));
  bool success = autonomy_interfaces__msg__AOIStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__AOIStatus__destroy(autonomy_interfaces__msg__AOIStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__AOIStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__AOIStatus__Sequence__init(autonomy_interfaces__msg__AOIStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIStatus * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__AOIStatus *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__AOIStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__AOIStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__AOIStatus__fini(&data[i - 1]);
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
autonomy_interfaces__msg__AOIStatus__Sequence__fini(autonomy_interfaces__msg__AOIStatus__Sequence * array)
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
      autonomy_interfaces__msg__AOIStatus__fini(&array->data[i]);
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

autonomy_interfaces__msg__AOIStatus__Sequence *
autonomy_interfaces__msg__AOIStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AOIStatus__Sequence * array = (autonomy_interfaces__msg__AOIStatus__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__AOIStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__AOIStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__AOIStatus__Sequence__destroy(autonomy_interfaces__msg__AOIStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__AOIStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__AOIStatus__Sequence__are_equal(const autonomy_interfaces__msg__AOIStatus__Sequence * lhs, const autonomy_interfaces__msg__AOIStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__AOIStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__AOIStatus__Sequence__copy(
  const autonomy_interfaces__msg__AOIStatus__Sequence * input,
  autonomy_interfaces__msg__AOIStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__AOIStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__AOIStatus * data =
      (autonomy_interfaces__msg__AOIStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__AOIStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__AOIStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__AOIStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
