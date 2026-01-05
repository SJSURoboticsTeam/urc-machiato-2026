// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/LedCommand.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/led_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pattern`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__LedCommand__init(autonomy_interfaces__msg__LedCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__LedCommand__fini(msg);
    return false;
  }
  // status_code
  // red
  // green
  // blue
  // pattern
  if (!rosidl_runtime_c__String__init(&msg->pattern)) {
    autonomy_interfaces__msg__LedCommand__fini(msg);
    return false;
  }
  // frequency
  // priority
  // duration
  // override
  return true;
}

void
autonomy_interfaces__msg__LedCommand__fini(autonomy_interfaces__msg__LedCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status_code
  // red
  // green
  // blue
  // pattern
  rosidl_runtime_c__String__fini(&msg->pattern);
  // frequency
  // priority
  // duration
  // override
}

bool
autonomy_interfaces__msg__LedCommand__are_equal(const autonomy_interfaces__msg__LedCommand * lhs, const autonomy_interfaces__msg__LedCommand * rhs)
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
  // status_code
  if (lhs->status_code != rhs->status_code) {
    return false;
  }
  // red
  if (lhs->red != rhs->red) {
    return false;
  }
  // green
  if (lhs->green != rhs->green) {
    return false;
  }
  // blue
  if (lhs->blue != rhs->blue) {
    return false;
  }
  // pattern
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pattern), &(rhs->pattern)))
  {
    return false;
  }
  // frequency
  if (lhs->frequency != rhs->frequency) {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // override
  if (lhs->override != rhs->override) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__LedCommand__copy(
  const autonomy_interfaces__msg__LedCommand * input,
  autonomy_interfaces__msg__LedCommand * output)
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
  // status_code
  output->status_code = input->status_code;
  // red
  output->red = input->red;
  // green
  output->green = input->green;
  // blue
  output->blue = input->blue;
  // pattern
  if (!rosidl_runtime_c__String__copy(
      &(input->pattern), &(output->pattern)))
  {
    return false;
  }
  // frequency
  output->frequency = input->frequency;
  // priority
  output->priority = input->priority;
  // duration
  output->duration = input->duration;
  // override
  output->override = input->override;
  return true;
}

autonomy_interfaces__msg__LedCommand *
autonomy_interfaces__msg__LedCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__LedCommand * msg = (autonomy_interfaces__msg__LedCommand *)allocator.allocate(sizeof(autonomy_interfaces__msg__LedCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__LedCommand));
  bool success = autonomy_interfaces__msg__LedCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__LedCommand__destroy(autonomy_interfaces__msg__LedCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__LedCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__LedCommand__Sequence__init(autonomy_interfaces__msg__LedCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__LedCommand * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__LedCommand *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__LedCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__LedCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__LedCommand__fini(&data[i - 1]);
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
autonomy_interfaces__msg__LedCommand__Sequence__fini(autonomy_interfaces__msg__LedCommand__Sequence * array)
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
      autonomy_interfaces__msg__LedCommand__fini(&array->data[i]);
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

autonomy_interfaces__msg__LedCommand__Sequence *
autonomy_interfaces__msg__LedCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__LedCommand__Sequence * array = (autonomy_interfaces__msg__LedCommand__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__LedCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__LedCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__LedCommand__Sequence__destroy(autonomy_interfaces__msg__LedCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__LedCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__LedCommand__Sequence__are_equal(const autonomy_interfaces__msg__LedCommand__Sequence * lhs, const autonomy_interfaces__msg__LedCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__LedCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__LedCommand__Sequence__copy(
  const autonomy_interfaces__msg__LedCommand__Sequence * input,
  autonomy_interfaces__msg__LedCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__LedCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__LedCommand * data =
      (autonomy_interfaces__msg__LedCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__LedCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__LedCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__LedCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
