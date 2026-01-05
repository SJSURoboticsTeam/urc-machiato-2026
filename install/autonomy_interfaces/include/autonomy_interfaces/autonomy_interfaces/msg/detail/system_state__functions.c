// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/system_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `current_state`
// Member `substate`
// Member `sub_substate`
// Member `previous_state`
// Member `active_subsystems`
// Member `failed_subsystems`
// Member `mission_phase`
// Member `operator_id`
// Member `state_reason`
// Member `active_adaptations`
#include "rosidl_runtime_c/string_functions.h"
// Member `transition_timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__SystemState__init(autonomy_interfaces__msg__SystemState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__init(&msg->substate)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // sub_substate
  if (!rosidl_runtime_c__String__init(&msg->sub_substate)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // time_in_state
  // state_timeout
  // previous_state
  if (!rosidl_runtime_c__String__init(&msg->previous_state)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // transition_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->transition_timestamp)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // is_transitioning
  // preconditions_met
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_subsystems, 0)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->failed_subsystems, 0)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__init(&msg->mission_phase)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // state_reason
  if (!rosidl_runtime_c__String__init(&msg->state_reason)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  // adaptive_enabled
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_adaptations, 0)) {
    autonomy_interfaces__msg__SystemState__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__SystemState__fini(autonomy_interfaces__msg__SystemState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // substate
  rosidl_runtime_c__String__fini(&msg->substate);
  // sub_substate
  rosidl_runtime_c__String__fini(&msg->sub_substate);
  // time_in_state
  // state_timeout
  // previous_state
  rosidl_runtime_c__String__fini(&msg->previous_state);
  // transition_timestamp
  builtin_interfaces__msg__Time__fini(&msg->transition_timestamp);
  // is_transitioning
  // preconditions_met
  // active_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->active_subsystems);
  // failed_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->failed_subsystems);
  // mission_phase
  rosidl_runtime_c__String__fini(&msg->mission_phase);
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
  // state_reason
  rosidl_runtime_c__String__fini(&msg->state_reason);
  // adaptive_enabled
  // active_adaptations
  rosidl_runtime_c__String__Sequence__fini(&msg->active_adaptations);
}

bool
autonomy_interfaces__msg__SystemState__are_equal(const autonomy_interfaces__msg__SystemState * lhs, const autonomy_interfaces__msg__SystemState * rhs)
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
  // current_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_state), &(rhs->current_state)))
  {
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->substate), &(rhs->substate)))
  {
    return false;
  }
  // sub_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sub_substate), &(rhs->sub_substate)))
  {
    return false;
  }
  // time_in_state
  if (lhs->time_in_state != rhs->time_in_state) {
    return false;
  }
  // state_timeout
  if (lhs->state_timeout != rhs->state_timeout) {
    return false;
  }
  // previous_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->previous_state), &(rhs->previous_state)))
  {
    return false;
  }
  // transition_timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->transition_timestamp), &(rhs->transition_timestamp)))
  {
    return false;
  }
  // is_transitioning
  if (lhs->is_transitioning != rhs->is_transitioning) {
    return false;
  }
  // preconditions_met
  if (lhs->preconditions_met != rhs->preconditions_met) {
    return false;
  }
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_subsystems), &(rhs->active_subsystems)))
  {
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->failed_subsystems), &(rhs->failed_subsystems)))
  {
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_phase), &(rhs->mission_phase)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  // state_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state_reason), &(rhs->state_reason)))
  {
    return false;
  }
  // adaptive_enabled
  if (lhs->adaptive_enabled != rhs->adaptive_enabled) {
    return false;
  }
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_adaptations), &(rhs->active_adaptations)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__SystemState__copy(
  const autonomy_interfaces__msg__SystemState * input,
  autonomy_interfaces__msg__SystemState * output)
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
  // current_state
  if (!rosidl_runtime_c__String__copy(
      &(input->current_state), &(output->current_state)))
  {
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__copy(
      &(input->substate), &(output->substate)))
  {
    return false;
  }
  // sub_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->sub_substate), &(output->sub_substate)))
  {
    return false;
  }
  // time_in_state
  output->time_in_state = input->time_in_state;
  // state_timeout
  output->state_timeout = input->state_timeout;
  // previous_state
  if (!rosidl_runtime_c__String__copy(
      &(input->previous_state), &(output->previous_state)))
  {
    return false;
  }
  // transition_timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->transition_timestamp), &(output->transition_timestamp)))
  {
    return false;
  }
  // is_transitioning
  output->is_transitioning = input->is_transitioning;
  // preconditions_met
  output->preconditions_met = input->preconditions_met;
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_subsystems), &(output->active_subsystems)))
  {
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->failed_subsystems), &(output->failed_subsystems)))
  {
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_phase), &(output->mission_phase)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  // state_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->state_reason), &(output->state_reason)))
  {
    return false;
  }
  // adaptive_enabled
  output->adaptive_enabled = input->adaptive_enabled;
  // active_adaptations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_adaptations), &(output->active_adaptations)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__SystemState *
autonomy_interfaces__msg__SystemState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SystemState * msg = (autonomy_interfaces__msg__SystemState *)allocator.allocate(sizeof(autonomy_interfaces__msg__SystemState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__SystemState));
  bool success = autonomy_interfaces__msg__SystemState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__SystemState__destroy(autonomy_interfaces__msg__SystemState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__SystemState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__SystemState__Sequence__init(autonomy_interfaces__msg__SystemState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SystemState * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__SystemState *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__SystemState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__SystemState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__SystemState__fini(&data[i - 1]);
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
autonomy_interfaces__msg__SystemState__Sequence__fini(autonomy_interfaces__msg__SystemState__Sequence * array)
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
      autonomy_interfaces__msg__SystemState__fini(&array->data[i]);
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

autonomy_interfaces__msg__SystemState__Sequence *
autonomy_interfaces__msg__SystemState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SystemState__Sequence * array = (autonomy_interfaces__msg__SystemState__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__SystemState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__SystemState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__SystemState__Sequence__destroy(autonomy_interfaces__msg__SystemState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__SystemState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__SystemState__Sequence__are_equal(const autonomy_interfaces__msg__SystemState__Sequence * lhs, const autonomy_interfaces__msg__SystemState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__SystemState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__SystemState__Sequence__copy(
  const autonomy_interfaces__msg__SystemState__Sequence * input,
  autonomy_interfaces__msg__SystemState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__SystemState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__SystemState * data =
      (autonomy_interfaces__msg__SystemState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__SystemState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__SystemState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__SystemState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
