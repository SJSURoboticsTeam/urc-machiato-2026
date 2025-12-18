// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/context_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `mission_type`
// Member `mission_status`
// Member `safety_reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__ContextState__init(autonomy_interfaces__msg__ContextState * msg)
{
  if (!msg) {
    return false;
  }
  // battery_level
  // battery_voltage
  // battery_critical
  // battery_warning
  // mission_type
  if (!rosidl_runtime_c__String__init(&msg->mission_type)) {
    autonomy_interfaces__msg__ContextState__fini(msg);
    return false;
  }
  // mission_status
  if (!rosidl_runtime_c__String__init(&msg->mission_status)) {
    autonomy_interfaces__msg__ContextState__fini(msg);
    return false;
  }
  // mission_progress
  // mission_time_remaining
  // communication_active
  // communication_latency
  // communication_quality
  // cpu_usage
  // memory_usage
  // temperature
  // obstacle_detected
  // obstacle_distance
  // terrain_difficulty
  // weather_adverse
  // safety_active
  // safety_reason
  if (!rosidl_runtime_c__String__init(&msg->safety_reason)) {
    autonomy_interfaces__msg__ContextState__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    autonomy_interfaces__msg__ContextState__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__ContextState__fini(autonomy_interfaces__msg__ContextState * msg)
{
  if (!msg) {
    return;
  }
  // battery_level
  // battery_voltage
  // battery_critical
  // battery_warning
  // mission_type
  rosidl_runtime_c__String__fini(&msg->mission_type);
  // mission_status
  rosidl_runtime_c__String__fini(&msg->mission_status);
  // mission_progress
  // mission_time_remaining
  // communication_active
  // communication_latency
  // communication_quality
  // cpu_usage
  // memory_usage
  // temperature
  // obstacle_detected
  // obstacle_distance
  // terrain_difficulty
  // weather_adverse
  // safety_active
  // safety_reason
  rosidl_runtime_c__String__fini(&msg->safety_reason);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
}

bool
autonomy_interfaces__msg__ContextState__are_equal(const autonomy_interfaces__msg__ContextState * lhs, const autonomy_interfaces__msg__ContextState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // battery_critical
  if (lhs->battery_critical != rhs->battery_critical) {
    return false;
  }
  // battery_warning
  if (lhs->battery_warning != rhs->battery_warning) {
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_type), &(rhs->mission_type)))
  {
    return false;
  }
  // mission_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_status), &(rhs->mission_status)))
  {
    return false;
  }
  // mission_progress
  if (lhs->mission_progress != rhs->mission_progress) {
    return false;
  }
  // mission_time_remaining
  if (lhs->mission_time_remaining != rhs->mission_time_remaining) {
    return false;
  }
  // communication_active
  if (lhs->communication_active != rhs->communication_active) {
    return false;
  }
  // communication_latency
  if (lhs->communication_latency != rhs->communication_latency) {
    return false;
  }
  // communication_quality
  if (lhs->communication_quality != rhs->communication_quality) {
    return false;
  }
  // cpu_usage
  if (lhs->cpu_usage != rhs->cpu_usage) {
    return false;
  }
  // memory_usage
  if (lhs->memory_usage != rhs->memory_usage) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // obstacle_detected
  if (lhs->obstacle_detected != rhs->obstacle_detected) {
    return false;
  }
  // obstacle_distance
  if (lhs->obstacle_distance != rhs->obstacle_distance) {
    return false;
  }
  // terrain_difficulty
  if (lhs->terrain_difficulty != rhs->terrain_difficulty) {
    return false;
  }
  // weather_adverse
  if (lhs->weather_adverse != rhs->weather_adverse) {
    return false;
  }
  // safety_active
  if (lhs->safety_active != rhs->safety_active) {
    return false;
  }
  // safety_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->safety_reason), &(rhs->safety_reason)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__ContextState__copy(
  const autonomy_interfaces__msg__ContextState * input,
  autonomy_interfaces__msg__ContextState * output)
{
  if (!input || !output) {
    return false;
  }
  // battery_level
  output->battery_level = input->battery_level;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // battery_critical
  output->battery_critical = input->battery_critical;
  // battery_warning
  output->battery_warning = input->battery_warning;
  // mission_type
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_type), &(output->mission_type)))
  {
    return false;
  }
  // mission_status
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_status), &(output->mission_status)))
  {
    return false;
  }
  // mission_progress
  output->mission_progress = input->mission_progress;
  // mission_time_remaining
  output->mission_time_remaining = input->mission_time_remaining;
  // communication_active
  output->communication_active = input->communication_active;
  // communication_latency
  output->communication_latency = input->communication_latency;
  // communication_quality
  output->communication_quality = input->communication_quality;
  // cpu_usage
  output->cpu_usage = input->cpu_usage;
  // memory_usage
  output->memory_usage = input->memory_usage;
  // temperature
  output->temperature = input->temperature;
  // obstacle_detected
  output->obstacle_detected = input->obstacle_detected;
  // obstacle_distance
  output->obstacle_distance = input->obstacle_distance;
  // terrain_difficulty
  output->terrain_difficulty = input->terrain_difficulty;
  // weather_adverse
  output->weather_adverse = input->weather_adverse;
  // safety_active
  output->safety_active = input->safety_active;
  // safety_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->safety_reason), &(output->safety_reason)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__ContextState *
autonomy_interfaces__msg__ContextState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextState * msg = (autonomy_interfaces__msg__ContextState *)allocator.allocate(sizeof(autonomy_interfaces__msg__ContextState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__ContextState));
  bool success = autonomy_interfaces__msg__ContextState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__ContextState__destroy(autonomy_interfaces__msg__ContextState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__ContextState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__ContextState__Sequence__init(autonomy_interfaces__msg__ContextState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextState * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__ContextState *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__ContextState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__ContextState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__ContextState__fini(&data[i - 1]);
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
autonomy_interfaces__msg__ContextState__Sequence__fini(autonomy_interfaces__msg__ContextState__Sequence * array)
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
      autonomy_interfaces__msg__ContextState__fini(&array->data[i]);
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

autonomy_interfaces__msg__ContextState__Sequence *
autonomy_interfaces__msg__ContextState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextState__Sequence * array = (autonomy_interfaces__msg__ContextState__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__ContextState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__ContextState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__ContextState__Sequence__destroy(autonomy_interfaces__msg__ContextState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__ContextState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__ContextState__Sequence__are_equal(const autonomy_interfaces__msg__ContextState__Sequence * lhs, const autonomy_interfaces__msg__ContextState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__ContextState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__ContextState__Sequence__copy(
  const autonomy_interfaces__msg__ContextState__Sequence * input,
  autonomy_interfaces__msg__ContextState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__ContextState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__ContextState * data =
      (autonomy_interfaces__msg__ContextState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__ContextState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__ContextState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__ContextState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
