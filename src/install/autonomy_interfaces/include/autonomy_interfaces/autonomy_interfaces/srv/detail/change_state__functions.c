// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/change_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `desired_state`
// Member `desired_substate`
// Member `desired_calibration_substate`
// Member `reason`
// Member `operator_id`
// Member `metadata`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__ChangeState_Request__init(autonomy_interfaces__srv__ChangeState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__init(&msg->desired_state)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  // desired_substate
  if (!rosidl_runtime_c__String__init(&msg->desired_substate)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  // desired_calibration_substate
  if (!rosidl_runtime_c__String__init(&msg->desired_calibration_substate)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  // force
  // metadata
  if (!rosidl_runtime_c__String__Sequence__init(&msg->metadata, 0)) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__ChangeState_Request__fini(autonomy_interfaces__srv__ChangeState_Request * msg)
{
  if (!msg) {
    return;
  }
  // desired_state
  rosidl_runtime_c__String__fini(&msg->desired_state);
  // desired_substate
  rosidl_runtime_c__String__fini(&msg->desired_substate);
  // desired_calibration_substate
  rosidl_runtime_c__String__fini(&msg->desired_calibration_substate);
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
  // force
  // metadata
  rosidl_runtime_c__String__Sequence__fini(&msg->metadata);
}

bool
autonomy_interfaces__srv__ChangeState_Request__are_equal(const autonomy_interfaces__srv__ChangeState_Request * lhs, const autonomy_interfaces__srv__ChangeState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->desired_state), &(rhs->desired_state)))
  {
    return false;
  }
  // desired_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->desired_substate), &(rhs->desired_substate)))
  {
    return false;
  }
  // desired_calibration_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->desired_calibration_substate), &(rhs->desired_calibration_substate)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  // metadata
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->metadata), &(rhs->metadata)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__ChangeState_Request__copy(
  const autonomy_interfaces__srv__ChangeState_Request * input,
  autonomy_interfaces__srv__ChangeState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__copy(
      &(input->desired_state), &(output->desired_state)))
  {
    return false;
  }
  // desired_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->desired_substate), &(output->desired_substate)))
  {
    return false;
  }
  // desired_calibration_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->desired_calibration_substate), &(output->desired_calibration_substate)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  // force
  output->force = input->force;
  // metadata
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->metadata), &(output->metadata)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__ChangeState_Request *
autonomy_interfaces__srv__ChangeState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Request * msg = (autonomy_interfaces__srv__ChangeState_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__ChangeState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ChangeState_Request));
  bool success = autonomy_interfaces__srv__ChangeState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ChangeState_Request__destroy(autonomy_interfaces__srv__ChangeState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ChangeState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ChangeState_Request__Sequence__init(autonomy_interfaces__srv__ChangeState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ChangeState_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ChangeState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ChangeState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ChangeState_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ChangeState_Request__Sequence__fini(autonomy_interfaces__srv__ChangeState_Request__Sequence * array)
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
      autonomy_interfaces__srv__ChangeState_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__ChangeState_Request__Sequence *
autonomy_interfaces__srv__ChangeState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Request__Sequence * array = (autonomy_interfaces__srv__ChangeState_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ChangeState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ChangeState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ChangeState_Request__Sequence__destroy(autonomy_interfaces__srv__ChangeState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ChangeState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ChangeState_Request__Sequence__are_equal(const autonomy_interfaces__srv__ChangeState_Request__Sequence * lhs, const autonomy_interfaces__srv__ChangeState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ChangeState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ChangeState_Request__Sequence__copy(
  const autonomy_interfaces__srv__ChangeState_Request__Sequence * input,
  autonomy_interfaces__srv__ChangeState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ChangeState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ChangeState_Request * data =
      (autonomy_interfaces__srv__ChangeState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ChangeState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ChangeState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ChangeState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `actual_state`
// Member `actual_substate`
// Member `actual_calibration_substate`
// Member `message`
// Member `failed_preconditions`
// Member `warnings`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__ChangeState_Response__init(autonomy_interfaces__srv__ChangeState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // actual_state
  if (!rosidl_runtime_c__String__init(&msg->actual_state)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  // actual_substate
  if (!rosidl_runtime_c__String__init(&msg->actual_substate)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  // actual_calibration_substate
  if (!rosidl_runtime_c__String__init(&msg->actual_calibration_substate)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  // transition_time
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  // preconditions_met
  // failed_preconditions
  if (!rosidl_runtime_c__String__Sequence__init(&msg->failed_preconditions, 0)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__init(&msg->warnings, 0)) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__ChangeState_Response__fini(autonomy_interfaces__srv__ChangeState_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // actual_state
  rosidl_runtime_c__String__fini(&msg->actual_state);
  // actual_substate
  rosidl_runtime_c__String__fini(&msg->actual_substate);
  // actual_calibration_substate
  rosidl_runtime_c__String__fini(&msg->actual_calibration_substate);
  // transition_time
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // preconditions_met
  // failed_preconditions
  rosidl_runtime_c__String__Sequence__fini(&msg->failed_preconditions);
  // warnings
  rosidl_runtime_c__String__Sequence__fini(&msg->warnings);
}

bool
autonomy_interfaces__srv__ChangeState_Response__are_equal(const autonomy_interfaces__srv__ChangeState_Response * lhs, const autonomy_interfaces__srv__ChangeState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // actual_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->actual_state), &(rhs->actual_state)))
  {
    return false;
  }
  // actual_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->actual_substate), &(rhs->actual_substate)))
  {
    return false;
  }
  // actual_calibration_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->actual_calibration_substate), &(rhs->actual_calibration_substate)))
  {
    return false;
  }
  // transition_time
  if (lhs->transition_time != rhs->transition_time) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // preconditions_met
  if (lhs->preconditions_met != rhs->preconditions_met) {
    return false;
  }
  // failed_preconditions
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->failed_preconditions), &(rhs->failed_preconditions)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->warnings), &(rhs->warnings)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__ChangeState_Response__copy(
  const autonomy_interfaces__srv__ChangeState_Response * input,
  autonomy_interfaces__srv__ChangeState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // actual_state
  if (!rosidl_runtime_c__String__copy(
      &(input->actual_state), &(output->actual_state)))
  {
    return false;
  }
  // actual_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->actual_substate), &(output->actual_substate)))
  {
    return false;
  }
  // actual_calibration_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->actual_calibration_substate), &(output->actual_calibration_substate)))
  {
    return false;
  }
  // transition_time
  output->transition_time = input->transition_time;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // preconditions_met
  output->preconditions_met = input->preconditions_met;
  // failed_preconditions
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->failed_preconditions), &(output->failed_preconditions)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->warnings), &(output->warnings)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__ChangeState_Response *
autonomy_interfaces__srv__ChangeState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Response * msg = (autonomy_interfaces__srv__ChangeState_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__ChangeState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ChangeState_Response));
  bool success = autonomy_interfaces__srv__ChangeState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ChangeState_Response__destroy(autonomy_interfaces__srv__ChangeState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ChangeState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ChangeState_Response__Sequence__init(autonomy_interfaces__srv__ChangeState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ChangeState_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ChangeState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ChangeState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ChangeState_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ChangeState_Response__Sequence__fini(autonomy_interfaces__srv__ChangeState_Response__Sequence * array)
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
      autonomy_interfaces__srv__ChangeState_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__ChangeState_Response__Sequence *
autonomy_interfaces__srv__ChangeState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ChangeState_Response__Sequence * array = (autonomy_interfaces__srv__ChangeState_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ChangeState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ChangeState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ChangeState_Response__Sequence__destroy(autonomy_interfaces__srv__ChangeState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ChangeState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ChangeState_Response__Sequence__are_equal(const autonomy_interfaces__srv__ChangeState_Response__Sequence * lhs, const autonomy_interfaces__srv__ChangeState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ChangeState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ChangeState_Response__Sequence__copy(
  const autonomy_interfaces__srv__ChangeState_Response__Sequence * input,
  autonomy_interfaces__srv__ChangeState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ChangeState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ChangeState_Response * data =
      (autonomy_interfaces__srv__ChangeState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ChangeState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ChangeState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ChangeState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
