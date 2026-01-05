// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/monitoring_stats__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
autonomy_interfaces__msg__MonitoringStats__init(autonomy_interfaces__msg__MonitoringStats * msg)
{
  if (!msg) {
    return false;
  }
  // total_evaluations
  // total_violations
  // evaluation_rate
  return true;
}

void
autonomy_interfaces__msg__MonitoringStats__fini(autonomy_interfaces__msg__MonitoringStats * msg)
{
  if (!msg) {
    return;
  }
  // total_evaluations
  // total_violations
  // evaluation_rate
}

bool
autonomy_interfaces__msg__MonitoringStats__are_equal(const autonomy_interfaces__msg__MonitoringStats * lhs, const autonomy_interfaces__msg__MonitoringStats * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // total_evaluations
  if (lhs->total_evaluations != rhs->total_evaluations) {
    return false;
  }
  // total_violations
  if (lhs->total_violations != rhs->total_violations) {
    return false;
  }
  // evaluation_rate
  if (lhs->evaluation_rate != rhs->evaluation_rate) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__MonitoringStats__copy(
  const autonomy_interfaces__msg__MonitoringStats * input,
  autonomy_interfaces__msg__MonitoringStats * output)
{
  if (!input || !output) {
    return false;
  }
  // total_evaluations
  output->total_evaluations = input->total_evaluations;
  // total_violations
  output->total_violations = input->total_violations;
  // evaluation_rate
  output->evaluation_rate = input->evaluation_rate;
  return true;
}

autonomy_interfaces__msg__MonitoringStats *
autonomy_interfaces__msg__MonitoringStats__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__MonitoringStats * msg = (autonomy_interfaces__msg__MonitoringStats *)allocator.allocate(sizeof(autonomy_interfaces__msg__MonitoringStats), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__MonitoringStats));
  bool success = autonomy_interfaces__msg__MonitoringStats__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__MonitoringStats__destroy(autonomy_interfaces__msg__MonitoringStats * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__MonitoringStats__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__MonitoringStats__Sequence__init(autonomy_interfaces__msg__MonitoringStats__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__MonitoringStats * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__MonitoringStats *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__MonitoringStats), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__MonitoringStats__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__MonitoringStats__fini(&data[i - 1]);
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
autonomy_interfaces__msg__MonitoringStats__Sequence__fini(autonomy_interfaces__msg__MonitoringStats__Sequence * array)
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
      autonomy_interfaces__msg__MonitoringStats__fini(&array->data[i]);
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

autonomy_interfaces__msg__MonitoringStats__Sequence *
autonomy_interfaces__msg__MonitoringStats__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__MonitoringStats__Sequence * array = (autonomy_interfaces__msg__MonitoringStats__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__MonitoringStats__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__MonitoringStats__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__MonitoringStats__Sequence__destroy(autonomy_interfaces__msg__MonitoringStats__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__MonitoringStats__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__MonitoringStats__Sequence__are_equal(const autonomy_interfaces__msg__MonitoringStats__Sequence * lhs, const autonomy_interfaces__msg__MonitoringStats__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__MonitoringStats__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__MonitoringStats__Sequence__copy(
  const autonomy_interfaces__msg__MonitoringStats__Sequence * input,
  autonomy_interfaces__msg__MonitoringStats__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__MonitoringStats);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__MonitoringStats * data =
      (autonomy_interfaces__msg__MonitoringStats *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__MonitoringStats__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__MonitoringStats__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__MonitoringStats__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
