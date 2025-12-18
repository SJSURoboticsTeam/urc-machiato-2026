// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/safety_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `safety_level`
// Member `active_triggers`
// Member `trigger_type`
// Member `trigger_source`
// Member `trigger_description`
// Member `recovery_steps`
// Member `context_state`
// Member `mission_phase`
// Member `degraded_capabilities`
#include "rosidl_runtime_c/string_functions.h"
// Member `trigger_time`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__msg__SafetyStatus__init(autonomy_interfaces__msg__SafetyStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // is_safe
  // safety_level
  if (!rosidl_runtime_c__String__init(&msg->safety_level)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // active_triggers
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_triggers, 0)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // trigger_type
  if (!rosidl_runtime_c__String__init(&msg->trigger_type)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // trigger_source
  if (!rosidl_runtime_c__String__init(&msg->trigger_source)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // trigger_time
  if (!builtin_interfaces__msg__Time__init(&msg->trigger_time)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // trigger_description
  if (!rosidl_runtime_c__String__init(&msg->trigger_description)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // requires_manual_intervention
  // can_auto_recover
  // recovery_steps
  if (!rosidl_runtime_c__String__Sequence__init(&msg->recovery_steps, 0)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // estimated_recovery_time
  // context_state
  if (!rosidl_runtime_c__String__init(&msg->context_state)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__init(&msg->mission_phase)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  // safe_to_retry
  // battery_level
  // temperature
  // communication_ok
  // sensors_ok
  // degraded_capabilities
  if (!rosidl_runtime_c__String__Sequence__init(&msg->degraded_capabilities, 0)) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__SafetyStatus__fini(autonomy_interfaces__msg__SafetyStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_safe
  // safety_level
  rosidl_runtime_c__String__fini(&msg->safety_level);
  // active_triggers
  rosidl_runtime_c__String__Sequence__fini(&msg->active_triggers);
  // trigger_type
  rosidl_runtime_c__String__fini(&msg->trigger_type);
  // trigger_source
  rosidl_runtime_c__String__fini(&msg->trigger_source);
  // trigger_time
  builtin_interfaces__msg__Time__fini(&msg->trigger_time);
  // trigger_description
  rosidl_runtime_c__String__fini(&msg->trigger_description);
  // requires_manual_intervention
  // can_auto_recover
  // recovery_steps
  rosidl_runtime_c__String__Sequence__fini(&msg->recovery_steps);
  // estimated_recovery_time
  // context_state
  rosidl_runtime_c__String__fini(&msg->context_state);
  // mission_phase
  rosidl_runtime_c__String__fini(&msg->mission_phase);
  // safe_to_retry
  // battery_level
  // temperature
  // communication_ok
  // sensors_ok
  // degraded_capabilities
  rosidl_runtime_c__String__Sequence__fini(&msg->degraded_capabilities);
}

bool
autonomy_interfaces__msg__SafetyStatus__are_equal(const autonomy_interfaces__msg__SafetyStatus * lhs, const autonomy_interfaces__msg__SafetyStatus * rhs)
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
  // is_safe
  if (lhs->is_safe != rhs->is_safe) {
    return false;
  }
  // safety_level
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->safety_level), &(rhs->safety_level)))
  {
    return false;
  }
  // active_triggers
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_triggers), &(rhs->active_triggers)))
  {
    return false;
  }
  // trigger_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->trigger_type), &(rhs->trigger_type)))
  {
    return false;
  }
  // trigger_source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->trigger_source), &(rhs->trigger_source)))
  {
    return false;
  }
  // trigger_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->trigger_time), &(rhs->trigger_time)))
  {
    return false;
  }
  // trigger_description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->trigger_description), &(rhs->trigger_description)))
  {
    return false;
  }
  // requires_manual_intervention
  if (lhs->requires_manual_intervention != rhs->requires_manual_intervention) {
    return false;
  }
  // can_auto_recover
  if (lhs->can_auto_recover != rhs->can_auto_recover) {
    return false;
  }
  // recovery_steps
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->recovery_steps), &(rhs->recovery_steps)))
  {
    return false;
  }
  // estimated_recovery_time
  if (lhs->estimated_recovery_time != rhs->estimated_recovery_time) {
    return false;
  }
  // context_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->context_state), &(rhs->context_state)))
  {
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_phase), &(rhs->mission_phase)))
  {
    return false;
  }
  // safe_to_retry
  if (lhs->safe_to_retry != rhs->safe_to_retry) {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // communication_ok
  if (lhs->communication_ok != rhs->communication_ok) {
    return false;
  }
  // sensors_ok
  if (lhs->sensors_ok != rhs->sensors_ok) {
    return false;
  }
  // degraded_capabilities
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->degraded_capabilities), &(rhs->degraded_capabilities)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__SafetyStatus__copy(
  const autonomy_interfaces__msg__SafetyStatus * input,
  autonomy_interfaces__msg__SafetyStatus * output)
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
  // is_safe
  output->is_safe = input->is_safe;
  // safety_level
  if (!rosidl_runtime_c__String__copy(
      &(input->safety_level), &(output->safety_level)))
  {
    return false;
  }
  // active_triggers
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_triggers), &(output->active_triggers)))
  {
    return false;
  }
  // trigger_type
  if (!rosidl_runtime_c__String__copy(
      &(input->trigger_type), &(output->trigger_type)))
  {
    return false;
  }
  // trigger_source
  if (!rosidl_runtime_c__String__copy(
      &(input->trigger_source), &(output->trigger_source)))
  {
    return false;
  }
  // trigger_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->trigger_time), &(output->trigger_time)))
  {
    return false;
  }
  // trigger_description
  if (!rosidl_runtime_c__String__copy(
      &(input->trigger_description), &(output->trigger_description)))
  {
    return false;
  }
  // requires_manual_intervention
  output->requires_manual_intervention = input->requires_manual_intervention;
  // can_auto_recover
  output->can_auto_recover = input->can_auto_recover;
  // recovery_steps
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->recovery_steps), &(output->recovery_steps)))
  {
    return false;
  }
  // estimated_recovery_time
  output->estimated_recovery_time = input->estimated_recovery_time;
  // context_state
  if (!rosidl_runtime_c__String__copy(
      &(input->context_state), &(output->context_state)))
  {
    return false;
  }
  // mission_phase
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_phase), &(output->mission_phase)))
  {
    return false;
  }
  // safe_to_retry
  output->safe_to_retry = input->safe_to_retry;
  // battery_level
  output->battery_level = input->battery_level;
  // temperature
  output->temperature = input->temperature;
  // communication_ok
  output->communication_ok = input->communication_ok;
  // sensors_ok
  output->sensors_ok = input->sensors_ok;
  // degraded_capabilities
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->degraded_capabilities), &(output->degraded_capabilities)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__SafetyStatus *
autonomy_interfaces__msg__SafetyStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyStatus * msg = (autonomy_interfaces__msg__SafetyStatus *)allocator.allocate(sizeof(autonomy_interfaces__msg__SafetyStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__SafetyStatus));
  bool success = autonomy_interfaces__msg__SafetyStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__SafetyStatus__destroy(autonomy_interfaces__msg__SafetyStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__SafetyStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__SafetyStatus__Sequence__init(autonomy_interfaces__msg__SafetyStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyStatus * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__SafetyStatus *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__SafetyStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__SafetyStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__SafetyStatus__fini(&data[i - 1]);
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
autonomy_interfaces__msg__SafetyStatus__Sequence__fini(autonomy_interfaces__msg__SafetyStatus__Sequence * array)
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
      autonomy_interfaces__msg__SafetyStatus__fini(&array->data[i]);
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

autonomy_interfaces__msg__SafetyStatus__Sequence *
autonomy_interfaces__msg__SafetyStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SafetyStatus__Sequence * array = (autonomy_interfaces__msg__SafetyStatus__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__SafetyStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__SafetyStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__SafetyStatus__Sequence__destroy(autonomy_interfaces__msg__SafetyStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__SafetyStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__SafetyStatus__Sequence__are_equal(const autonomy_interfaces__msg__SafetyStatus__Sequence * lhs, const autonomy_interfaces__msg__SafetyStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__SafetyStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__SafetyStatus__Sequence__copy(
  const autonomy_interfaces__msg__SafetyStatus__Sequence * input,
  autonomy_interfaces__msg__SafetyStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__SafetyStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__SafetyStatus * data =
      (autonomy_interfaces__msg__SafetyStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__SafetyStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__SafetyStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__SafetyStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
