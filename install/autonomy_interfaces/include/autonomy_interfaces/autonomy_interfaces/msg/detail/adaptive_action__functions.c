// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/adaptive_action__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `action_type`
// Member `parameters`
// Member `success_criteria`
#include "rosidl_runtime_c/string_functions.h"
// Member `trigger_context`
#include "autonomy_interfaces/msg/detail/context_state__functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__AdaptiveAction__init(autonomy_interfaces__msg__AdaptiveAction * msg)
{
  if (!msg) {
    return false;
  }
  // action_type
  if (!rosidl_runtime_c__String__init(&msg->action_type)) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
    return false;
  }
  // parameters
  if (!rosidl_runtime_c__String__Sequence__init(&msg->parameters, 0)) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
    return false;
  }
  // trigger_context
  if (!autonomy_interfaces__msg__ContextState__init(&msg->trigger_context)) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
    return false;
  }
  // priority
  // expected_duration
  // success_criteria
  if (!rosidl_runtime_c__String__init(&msg->success_criteria)) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__AdaptiveAction__fini(autonomy_interfaces__msg__AdaptiveAction * msg)
{
  if (!msg) {
    return;
  }
  // action_type
  rosidl_runtime_c__String__fini(&msg->action_type);
  // parameters
  rosidl_runtime_c__String__Sequence__fini(&msg->parameters);
  // trigger_context
  autonomy_interfaces__msg__ContextState__fini(&msg->trigger_context);
  // priority
  // expected_duration
  // success_criteria
  rosidl_runtime_c__String__fini(&msg->success_criteria);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
}

bool
autonomy_interfaces__msg__AdaptiveAction__are_equal(const autonomy_interfaces__msg__AdaptiveAction * lhs, const autonomy_interfaces__msg__AdaptiveAction * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action_type), &(rhs->action_type)))
  {
    return false;
  }
  // parameters
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->parameters), &(rhs->parameters)))
  {
    return false;
  }
  // trigger_context
  if (!autonomy_interfaces__msg__ContextState__are_equal(
      &(lhs->trigger_context), &(rhs->trigger_context)))
  {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  // expected_duration
  if (lhs->expected_duration != rhs->expected_duration) {
    return false;
  }
  // success_criteria
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->success_criteria), &(rhs->success_criteria)))
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
autonomy_interfaces__msg__AdaptiveAction__copy(
  const autonomy_interfaces__msg__AdaptiveAction * input,
  autonomy_interfaces__msg__AdaptiveAction * output)
{
  if (!input || !output) {
    return false;
  }
  // action_type
  if (!rosidl_runtime_c__String__copy(
      &(input->action_type), &(output->action_type)))
  {
    return false;
  }
  // parameters
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->parameters), &(output->parameters)))
  {
    return false;
  }
  // trigger_context
  if (!autonomy_interfaces__msg__ContextState__copy(
      &(input->trigger_context), &(output->trigger_context)))
  {
    return false;
  }
  // priority
  output->priority = input->priority;
  // expected_duration
  output->expected_duration = input->expected_duration;
  // success_criteria
  if (!rosidl_runtime_c__String__copy(
      &(input->success_criteria), &(output->success_criteria)))
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

autonomy_interfaces__msg__AdaptiveAction *
autonomy_interfaces__msg__AdaptiveAction__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AdaptiveAction * msg = (autonomy_interfaces__msg__AdaptiveAction *)allocator.allocate(sizeof(autonomy_interfaces__msg__AdaptiveAction), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__AdaptiveAction));
  bool success = autonomy_interfaces__msg__AdaptiveAction__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__AdaptiveAction__destroy(autonomy_interfaces__msg__AdaptiveAction * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__AdaptiveAction__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__AdaptiveAction__Sequence__init(autonomy_interfaces__msg__AdaptiveAction__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AdaptiveAction * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__AdaptiveAction *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__AdaptiveAction), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__AdaptiveAction__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__AdaptiveAction__fini(&data[i - 1]);
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
autonomy_interfaces__msg__AdaptiveAction__Sequence__fini(autonomy_interfaces__msg__AdaptiveAction__Sequence * array)
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
      autonomy_interfaces__msg__AdaptiveAction__fini(&array->data[i]);
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

autonomy_interfaces__msg__AdaptiveAction__Sequence *
autonomy_interfaces__msg__AdaptiveAction__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__AdaptiveAction__Sequence * array = (autonomy_interfaces__msg__AdaptiveAction__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__AdaptiveAction__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__AdaptiveAction__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__AdaptiveAction__Sequence__destroy(autonomy_interfaces__msg__AdaptiveAction__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__AdaptiveAction__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__AdaptiveAction__Sequence__are_equal(const autonomy_interfaces__msg__AdaptiveAction__Sequence * lhs, const autonomy_interfaces__msg__AdaptiveAction__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__AdaptiveAction__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__AdaptiveAction__Sequence__copy(
  const autonomy_interfaces__msg__AdaptiveAction__Sequence * input,
  autonomy_interfaces__msg__AdaptiveAction__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__AdaptiveAction);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__AdaptiveAction * data =
      (autonomy_interfaces__msg__AdaptiveAction *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__AdaptiveAction__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__AdaptiveAction__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__AdaptiveAction__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
