// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/context_update__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `mission_status`
// Member `active_adaptations`
// Member `alert_level`
// Member `available_actions`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__ContextUpdate__init(autonomy_interfaces__msg__ContextUpdate * msg)
{
  if (!msg) {
    return false;
  }
  // battery_level
  // mission_status
  if (!rosidl_runtime_c__String__init(&msg->mission_status)) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
    return false;
  }
  // mission_progress
  // communication_active
  // safety_active
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_adaptations, 0)) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
    return false;
  }
  // alert_level
  if (!rosidl_runtime_c__String__init(&msg->alert_level)) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
    return false;
  }
  // available_actions
  if (!rosidl_runtime_c__String__Sequence__init(&msg->available_actions, 0)) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__ContextUpdate__fini(autonomy_interfaces__msg__ContextUpdate * msg)
{
  if (!msg) {
    return;
  }
  // battery_level
  // mission_status
  rosidl_runtime_c__String__fini(&msg->mission_status);
  // mission_progress
  // communication_active
  // safety_active
  // active_adaptations
  rosidl_runtime_c__String__Sequence__fini(&msg->active_adaptations);
  // alert_level
  rosidl_runtime_c__String__fini(&msg->alert_level);
  // available_actions
  rosidl_runtime_c__String__Sequence__fini(&msg->available_actions);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
}

bool
autonomy_interfaces__msg__ContextUpdate__are_equal(const autonomy_interfaces__msg__ContextUpdate * lhs, const autonomy_interfaces__msg__ContextUpdate * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
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
  // communication_active
  if (lhs->communication_active != rhs->communication_active) {
    return false;
  }
  // safety_active
  if (lhs->safety_active != rhs->safety_active) {
    return false;
  }
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_adaptations), &(rhs->active_adaptations)))
  {
    return false;
  }
  // alert_level
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->alert_level), &(rhs->alert_level)))
  {
    return false;
  }
  // available_actions
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->available_actions), &(rhs->available_actions)))
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
autonomy_interfaces__msg__ContextUpdate__copy(
  const autonomy_interfaces__msg__ContextUpdate * input,
  autonomy_interfaces__msg__ContextUpdate * output)
{
  if (!input || !output) {
    return false;
  }
  // battery_level
  output->battery_level = input->battery_level;
  // mission_status
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_status), &(output->mission_status)))
  {
    return false;
  }
  // mission_progress
  output->mission_progress = input->mission_progress;
  // communication_active
  output->communication_active = input->communication_active;
  // safety_active
  output->safety_active = input->safety_active;
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_adaptations), &(output->active_adaptations)))
  {
    return false;
  }
  // alert_level
  if (!rosidl_runtime_c__String__copy(
      &(input->alert_level), &(output->alert_level)))
  {
    return false;
  }
  // available_actions
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->available_actions), &(output->available_actions)))
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

autonomy_interfaces__msg__ContextUpdate *
autonomy_interfaces__msg__ContextUpdate__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextUpdate * msg = (autonomy_interfaces__msg__ContextUpdate *)allocator.allocate(sizeof(autonomy_interfaces__msg__ContextUpdate), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__ContextUpdate));
  bool success = autonomy_interfaces__msg__ContextUpdate__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__ContextUpdate__destroy(autonomy_interfaces__msg__ContextUpdate * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__ContextUpdate__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__ContextUpdate__Sequence__init(autonomy_interfaces__msg__ContextUpdate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextUpdate * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__ContextUpdate *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__ContextUpdate), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__ContextUpdate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__ContextUpdate__fini(&data[i - 1]);
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
autonomy_interfaces__msg__ContextUpdate__Sequence__fini(autonomy_interfaces__msg__ContextUpdate__Sequence * array)
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
      autonomy_interfaces__msg__ContextUpdate__fini(&array->data[i]);
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

autonomy_interfaces__msg__ContextUpdate__Sequence *
autonomy_interfaces__msg__ContextUpdate__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ContextUpdate__Sequence * array = (autonomy_interfaces__msg__ContextUpdate__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__ContextUpdate__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__ContextUpdate__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__ContextUpdate__Sequence__destroy(autonomy_interfaces__msg__ContextUpdate__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__ContextUpdate__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__ContextUpdate__Sequence__are_equal(const autonomy_interfaces__msg__ContextUpdate__Sequence * lhs, const autonomy_interfaces__msg__ContextUpdate__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__ContextUpdate__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__ContextUpdate__Sequence__copy(
  const autonomy_interfaces__msg__ContextUpdate__Sequence * input,
  autonomy_interfaces__msg__ContextUpdate__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__ContextUpdate);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__ContextUpdate * data =
      (autonomy_interfaces__msg__ContextUpdate *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__ContextUpdate__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__ContextUpdate__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__ContextUpdate__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
