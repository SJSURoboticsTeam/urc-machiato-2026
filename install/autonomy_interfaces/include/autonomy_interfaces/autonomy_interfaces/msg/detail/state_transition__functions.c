// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/state_transition__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `from_state`
// Member `to_state`
// Member `reason`
// Member `initiated_by`
// Member `failure_reason`
// Member `preconditions_checked`
// Member `entry_actions_executed`
// Member `exit_actions_executed`
#include "rosidl_runtime_c/string_functions.h"
// Member `start_time`
// Member `end_time`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__StateTransition__init(autonomy_interfaces__msg__StateTransition * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // from_state
  if (!rosidl_runtime_c__String__init(&msg->from_state)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // to_state
  if (!rosidl_runtime_c__String__init(&msg->to_state)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // start_time
  if (!builtin_interfaces__msg__Time__init(&msg->start_time)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // end_time
  if (!builtin_interfaces__msg__Time__init(&msg->end_time)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // transition_duration
  // success
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // initiated_by
  if (!rosidl_runtime_c__String__init(&msg->initiated_by)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__init(&msg->failure_reason)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // preconditions_checked
  if (!rosidl_runtime_c__String__Sequence__init(&msg->preconditions_checked, 0)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // entry_actions_executed
  if (!rosidl_runtime_c__String__Sequence__init(&msg->entry_actions_executed, 0)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  // exit_actions_executed
  if (!rosidl_runtime_c__String__Sequence__init(&msg->exit_actions_executed, 0)) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__StateTransition__fini(autonomy_interfaces__msg__StateTransition * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // from_state
  rosidl_runtime_c__String__fini(&msg->from_state);
  // to_state
  rosidl_runtime_c__String__fini(&msg->to_state);
  // start_time
  builtin_interfaces__msg__Time__fini(&msg->start_time);
  // end_time
  builtin_interfaces__msg__Time__fini(&msg->end_time);
  // transition_duration
  // success
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // initiated_by
  rosidl_runtime_c__String__fini(&msg->initiated_by);
  // failure_reason
  rosidl_runtime_c__String__fini(&msg->failure_reason);
  // preconditions_checked
  rosidl_runtime_c__String__Sequence__fini(&msg->preconditions_checked);
  // entry_actions_executed
  rosidl_runtime_c__String__Sequence__fini(&msg->entry_actions_executed);
  // exit_actions_executed
  rosidl_runtime_c__String__Sequence__fini(&msg->exit_actions_executed);
}

bool
autonomy_interfaces__msg__StateTransition__are_equal(const autonomy_interfaces__msg__StateTransition * lhs, const autonomy_interfaces__msg__StateTransition * rhs)
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
  // from_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->from_state), &(rhs->from_state)))
  {
    return false;
  }
  // to_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->to_state), &(rhs->to_state)))
  {
    return false;
  }
  // start_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->start_time), &(rhs->start_time)))
  {
    return false;
  }
  // end_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->end_time), &(rhs->end_time)))
  {
    return false;
  }
  // transition_duration
  if (lhs->transition_duration != rhs->transition_duration) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // initiated_by
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->initiated_by), &(rhs->initiated_by)))
  {
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->failure_reason), &(rhs->failure_reason)))
  {
    return false;
  }
  // preconditions_checked
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->preconditions_checked), &(rhs->preconditions_checked)))
  {
    return false;
  }
  // entry_actions_executed
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->entry_actions_executed), &(rhs->entry_actions_executed)))
  {
    return false;
  }
  // exit_actions_executed
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->exit_actions_executed), &(rhs->exit_actions_executed)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__StateTransition__copy(
  const autonomy_interfaces__msg__StateTransition * input,
  autonomy_interfaces__msg__StateTransition * output)
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
  // from_state
  if (!rosidl_runtime_c__String__copy(
      &(input->from_state), &(output->from_state)))
  {
    return false;
  }
  // to_state
  if (!rosidl_runtime_c__String__copy(
      &(input->to_state), &(output->to_state)))
  {
    return false;
  }
  // start_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->start_time), &(output->start_time)))
  {
    return false;
  }
  // end_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->end_time), &(output->end_time)))
  {
    return false;
  }
  // transition_duration
  output->transition_duration = input->transition_duration;
  // success
  output->success = input->success;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // initiated_by
  if (!rosidl_runtime_c__String__copy(
      &(input->initiated_by), &(output->initiated_by)))
  {
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->failure_reason), &(output->failure_reason)))
  {
    return false;
  }
  // preconditions_checked
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->preconditions_checked), &(output->preconditions_checked)))
  {
    return false;
  }
  // entry_actions_executed
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->entry_actions_executed), &(output->entry_actions_executed)))
  {
    return false;
  }
  // exit_actions_executed
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->exit_actions_executed), &(output->exit_actions_executed)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__StateTransition *
autonomy_interfaces__msg__StateTransition__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__StateTransition * msg = (autonomy_interfaces__msg__StateTransition *)allocator.allocate(sizeof(autonomy_interfaces__msg__StateTransition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__StateTransition));
  bool success = autonomy_interfaces__msg__StateTransition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__StateTransition__destroy(autonomy_interfaces__msg__StateTransition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__StateTransition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__StateTransition__Sequence__init(autonomy_interfaces__msg__StateTransition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__StateTransition * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__StateTransition *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__StateTransition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__StateTransition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__StateTransition__fini(&data[i - 1]);
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
autonomy_interfaces__msg__StateTransition__Sequence__fini(autonomy_interfaces__msg__StateTransition__Sequence * array)
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
      autonomy_interfaces__msg__StateTransition__fini(&array->data[i]);
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

autonomy_interfaces__msg__StateTransition__Sequence *
autonomy_interfaces__msg__StateTransition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__StateTransition__Sequence * array = (autonomy_interfaces__msg__StateTransition__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__StateTransition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__StateTransition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__StateTransition__Sequence__destroy(autonomy_interfaces__msg__StateTransition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__StateTransition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__StateTransition__Sequence__are_equal(const autonomy_interfaces__msg__StateTransition__Sequence * lhs, const autonomy_interfaces__msg__StateTransition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__StateTransition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__StateTransition__Sequence__copy(
  const autonomy_interfaces__msg__StateTransition__Sequence * input,
  autonomy_interfaces__msg__StateTransition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__StateTransition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__StateTransition * data =
      (autonomy_interfaces__msg__StateTransition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__StateTransition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__StateTransition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__StateTransition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
