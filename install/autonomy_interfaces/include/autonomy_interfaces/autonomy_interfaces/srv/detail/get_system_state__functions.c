// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/get_system_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
autonomy_interfaces__srv__GetSystemState_Request__init(autonomy_interfaces__srv__GetSystemState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // include_history
  // include_subsystems
  // history_limit
  return true;
}

void
autonomy_interfaces__srv__GetSystemState_Request__fini(autonomy_interfaces__srv__GetSystemState_Request * msg)
{
  if (!msg) {
    return;
  }
  // include_history
  // include_subsystems
  // history_limit
}

bool
autonomy_interfaces__srv__GetSystemState_Request__are_equal(const autonomy_interfaces__srv__GetSystemState_Request * lhs, const autonomy_interfaces__srv__GetSystemState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // include_history
  if (lhs->include_history != rhs->include_history) {
    return false;
  }
  // include_subsystems
  if (lhs->include_subsystems != rhs->include_subsystems) {
    return false;
  }
  // history_limit
  if (lhs->history_limit != rhs->history_limit) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Request__copy(
  const autonomy_interfaces__srv__GetSystemState_Request * input,
  autonomy_interfaces__srv__GetSystemState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // include_history
  output->include_history = input->include_history;
  // include_subsystems
  output->include_subsystems = input->include_subsystems;
  // history_limit
  output->history_limit = input->history_limit;
  return true;
}

autonomy_interfaces__srv__GetSystemState_Request *
autonomy_interfaces__srv__GetSystemState_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Request * msg = (autonomy_interfaces__srv__GetSystemState_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSystemState_Request));
  bool success = autonomy_interfaces__srv__GetSystemState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSystemState_Request__destroy(autonomy_interfaces__srv__GetSystemState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSystemState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSystemState_Request__Sequence__init(autonomy_interfaces__srv__GetSystemState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSystemState_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSystemState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSystemState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSystemState_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSystemState_Request__Sequence__fini(autonomy_interfaces__srv__GetSystemState_Request__Sequence * array)
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
      autonomy_interfaces__srv__GetSystemState_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSystemState_Request__Sequence *
autonomy_interfaces__srv__GetSystemState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Request__Sequence * array = (autonomy_interfaces__srv__GetSystemState_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSystemState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSystemState_Request__Sequence__destroy(autonomy_interfaces__srv__GetSystemState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSystemState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSystemState_Request__Sequence__are_equal(const autonomy_interfaces__srv__GetSystemState_Request__Sequence * lhs, const autonomy_interfaces__srv__GetSystemState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Request__Sequence__copy(
  const autonomy_interfaces__srv__GetSystemState_Request__Sequence * input,
  autonomy_interfaces__srv__GetSystemState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSystemState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSystemState_Request * data =
      (autonomy_interfaces__srv__GetSystemState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSystemState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSystemState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `current_state`
// Member `substate`
// Member `sub_substate`
// Member `recent_states`
// Member `transition_reasons`
// Member `active_subsystems`
// Member `inactive_subsystems`
// Member `failed_subsystems`
#include "rosidl_runtime_c/string_functions.h"
// Member `state_entered`
// Member `state_timestamps`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__srv__GetSystemState_Response__init(autonomy_interfaces__srv__GetSystemState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__init(&msg->substate)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // sub_substate
  if (!rosidl_runtime_c__String__init(&msg->sub_substate)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // time_in_state
  // state_entered
  if (!builtin_interfaces__msg__Time__init(&msg->state_entered)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // recent_states
  if (!rosidl_runtime_c__String__Sequence__init(&msg->recent_states, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // state_timestamps
  if (!builtin_interfaces__msg__Time__Sequence__init(&msg->state_timestamps, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // transition_reasons
  if (!rosidl_runtime_c__String__Sequence__init(&msg->transition_reasons, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_subsystems, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // inactive_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->inactive_subsystems, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->failed_subsystems, 0)) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetSystemState_Response__fini(autonomy_interfaces__srv__GetSystemState_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // substate
  rosidl_runtime_c__String__fini(&msg->substate);
  // sub_substate
  rosidl_runtime_c__String__fini(&msg->sub_substate);
  // time_in_state
  // state_entered
  builtin_interfaces__msg__Time__fini(&msg->state_entered);
  // recent_states
  rosidl_runtime_c__String__Sequence__fini(&msg->recent_states);
  // state_timestamps
  builtin_interfaces__msg__Time__Sequence__fini(&msg->state_timestamps);
  // transition_reasons
  rosidl_runtime_c__String__Sequence__fini(&msg->transition_reasons);
  // active_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->active_subsystems);
  // inactive_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->inactive_subsystems);
  // failed_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->failed_subsystems);
}

bool
autonomy_interfaces__srv__GetSystemState_Response__are_equal(const autonomy_interfaces__srv__GetSystemState_Response * lhs, const autonomy_interfaces__srv__GetSystemState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
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
  // state_entered
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->state_entered), &(rhs->state_entered)))
  {
    return false;
  }
  // recent_states
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->recent_states), &(rhs->recent_states)))
  {
    return false;
  }
  // state_timestamps
  if (!builtin_interfaces__msg__Time__Sequence__are_equal(
      &(lhs->state_timestamps), &(rhs->state_timestamps)))
  {
    return false;
  }
  // transition_reasons
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->transition_reasons), &(rhs->transition_reasons)))
  {
    return false;
  }
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_subsystems), &(rhs->active_subsystems)))
  {
    return false;
  }
  // inactive_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->inactive_subsystems), &(rhs->inactive_subsystems)))
  {
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->failed_subsystems), &(rhs->failed_subsystems)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Response__copy(
  const autonomy_interfaces__srv__GetSystemState_Response * input,
  autonomy_interfaces__srv__GetSystemState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
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
  // state_entered
  if (!builtin_interfaces__msg__Time__copy(
      &(input->state_entered), &(output->state_entered)))
  {
    return false;
  }
  // recent_states
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->recent_states), &(output->recent_states)))
  {
    return false;
  }
  // state_timestamps
  if (!builtin_interfaces__msg__Time__Sequence__copy(
      &(input->state_timestamps), &(output->state_timestamps)))
  {
    return false;
  }
  // transition_reasons
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->transition_reasons), &(output->transition_reasons)))
  {
    return false;
  }
  // active_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_subsystems), &(output->active_subsystems)))
  {
    return false;
  }
  // inactive_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->inactive_subsystems), &(output->inactive_subsystems)))
  {
    return false;
  }
  // failed_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->failed_subsystems), &(output->failed_subsystems)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetSystemState_Response *
autonomy_interfaces__srv__GetSystemState_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Response * msg = (autonomy_interfaces__srv__GetSystemState_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSystemState_Response));
  bool success = autonomy_interfaces__srv__GetSystemState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSystemState_Response__destroy(autonomy_interfaces__srv__GetSystemState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSystemState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSystemState_Response__Sequence__init(autonomy_interfaces__srv__GetSystemState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSystemState_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSystemState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSystemState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSystemState_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSystemState_Response__Sequence__fini(autonomy_interfaces__srv__GetSystemState_Response__Sequence * array)
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
      autonomy_interfaces__srv__GetSystemState_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSystemState_Response__Sequence *
autonomy_interfaces__srv__GetSystemState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Response__Sequence * array = (autonomy_interfaces__srv__GetSystemState_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSystemState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSystemState_Response__Sequence__destroy(autonomy_interfaces__srv__GetSystemState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSystemState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSystemState_Response__Sequence__are_equal(const autonomy_interfaces__srv__GetSystemState_Response__Sequence * lhs, const autonomy_interfaces__srv__GetSystemState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Response__Sequence__copy(
  const autonomy_interfaces__srv__GetSystemState_Response__Sequence * input,
  autonomy_interfaces__srv__GetSystemState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSystemState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSystemState_Response * data =
      (autonomy_interfaces__srv__GetSystemState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSystemState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSystemState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "autonomy_interfaces/srv/detail/get_system_state__functions.h"

bool
autonomy_interfaces__srv__GetSystemState_Event__init(autonomy_interfaces__srv__GetSystemState_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__GetSystemState_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__GetSystemState_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__GetSystemState_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSystemState_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__GetSystemState_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetSystemState_Event__fini(autonomy_interfaces__srv__GetSystemState_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__GetSystemState_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__GetSystemState_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__GetSystemState_Event__are_equal(const autonomy_interfaces__srv__GetSystemState_Event * lhs, const autonomy_interfaces__srv__GetSystemState_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__GetSystemState_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSystemState_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Event__copy(
  const autonomy_interfaces__srv__GetSystemState_Event * input,
  autonomy_interfaces__srv__GetSystemState_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__GetSystemState_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSystemState_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetSystemState_Event *
autonomy_interfaces__srv__GetSystemState_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Event * msg = (autonomy_interfaces__srv__GetSystemState_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSystemState_Event));
  bool success = autonomy_interfaces__srv__GetSystemState_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSystemState_Event__destroy(autonomy_interfaces__srv__GetSystemState_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSystemState_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSystemState_Event__Sequence__init(autonomy_interfaces__srv__GetSystemState_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSystemState_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSystemState_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSystemState_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSystemState_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSystemState_Event__Sequence__fini(autonomy_interfaces__srv__GetSystemState_Event__Sequence * array)
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
      autonomy_interfaces__srv__GetSystemState_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSystemState_Event__Sequence *
autonomy_interfaces__srv__GetSystemState_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSystemState_Event__Sequence * array = (autonomy_interfaces__srv__GetSystemState_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSystemState_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSystemState_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSystemState_Event__Sequence__destroy(autonomy_interfaces__srv__GetSystemState_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSystemState_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSystemState_Event__Sequence__are_equal(const autonomy_interfaces__srv__GetSystemState_Event__Sequence * lhs, const autonomy_interfaces__srv__GetSystemState_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSystemState_Event__Sequence__copy(
  const autonomy_interfaces__srv__GetSystemState_Event__Sequence * input,
  autonomy_interfaces__srv__GetSystemState_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSystemState_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSystemState_Event * data =
      (autonomy_interfaces__srv__GetSystemState_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSystemState_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSystemState_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSystemState_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
