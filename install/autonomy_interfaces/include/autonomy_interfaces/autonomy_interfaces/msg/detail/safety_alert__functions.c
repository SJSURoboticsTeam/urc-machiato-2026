// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/SafetyAlert.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/safety_alert__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `property`
// Member `severity`
// Member `details`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__SafetyAlert__init(autonomy_interfaces__msg__SafetyAlert * msg)
{
  if (!msg) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__init(&msg->property)) {
    autonomy_interfaces__msg__SafetyAlert__fini(msg);
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__init(&msg->severity)) {
    autonomy_interfaces__msg__SafetyAlert__fini(msg);
    return false;
  }
  // details
  if (!rosidl_runtime_c__String__init(&msg->details)) {
    autonomy_interfaces__msg__SafetyAlert__fini(msg);
    return false;
  }
  // timestamp
  // acknowledged
  return true;
}

void
autonomy_interfaces__msg__SafetyAlert__fini(autonomy_interfaces__msg__SafetyAlert * msg)
{
  if (!msg) {
    return;
  }
  // property
  rosidl_runtime_c__String__fini(&msg->property);
  // severity
  rosidl_runtime_c__String__fini(&msg->severity);
  // details
  rosidl_runtime_c__String__fini(&msg->details);
  // timestamp
  // acknowledged
}

bool
autonomy_interfaces__msg__SafetyAlert__are_equal(const autonomy_interfaces__msg__SafetyAlert * lhs, const autonomy_interfaces__msg__SafetyAlert * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->property), &(rhs->property)))
  {
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->severity), &(rhs->severity)))
  {
    return false;
  }
  // details
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->details), &(rhs->details)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // acknowledged
  if (lhs->acknowledged != rhs->acknowledged) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__SafetyAlert__copy(
  const autonomy_interfaces__msg__SafetyAlert * input,
  autonomy_interfaces__msg__SafetyAlert * output)
{
  if (!input || !output) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__copy(
      &(input->property), &(output->property)))
  {
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__copy(
      &(input->severity), &(output->severity)))
  {
    return false;
  }
  // details
  if (!rosidl_runtime_c__String__copy(
      &(input->details), &(output->details)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // acknowledged
  output->acknowledged = input->acknowledged;
  return true;
}

autonomy_interfaces__msg__SafetyAlert *
autonomy_interfaces__msg__SafetyAlert__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyAlert * msg = (autonomy_interfaces__msg__SafetyAlert *)allocator.allocate(sizeof(autonomy_interfaces__msg__SafetyAlert), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__SafetyAlert));
  bool success = autonomy_interfaces__msg__SafetyAlert__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__SafetyAlert__destroy(autonomy_interfaces__msg__SafetyAlert * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__SafetyAlert__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__SafetyAlert__Sequence__init(autonomy_interfaces__msg__SafetyAlert__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyAlert * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__SafetyAlert *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__SafetyAlert), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__SafetyAlert__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__SafetyAlert__fini(&data[i - 1]);
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
autonomy_interfaces__msg__SafetyAlert__Sequence__fini(autonomy_interfaces__msg__SafetyAlert__Sequence * array)
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
      autonomy_interfaces__msg__SafetyAlert__fini(&array->data[i]);
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

autonomy_interfaces__msg__SafetyAlert__Sequence *
autonomy_interfaces__msg__SafetyAlert__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyAlert__Sequence * array = (autonomy_interfaces__msg__SafetyAlert__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__SafetyAlert__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__SafetyAlert__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__SafetyAlert__Sequence__destroy(autonomy_interfaces__msg__SafetyAlert__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__SafetyAlert__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__SafetyAlert__Sequence__are_equal(const autonomy_interfaces__msg__SafetyAlert__Sequence * lhs, const autonomy_interfaces__msg__SafetyAlert__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__SafetyAlert__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__SafetyAlert__Sequence__copy(
  const autonomy_interfaces__msg__SafetyAlert__Sequence * input,
  autonomy_interfaces__msg__SafetyAlert__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__SafetyAlert);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__SafetyAlert * data =
      (autonomy_interfaces__msg__SafetyAlert *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__SafetyAlert__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__SafetyAlert__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__SafetyAlert__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
