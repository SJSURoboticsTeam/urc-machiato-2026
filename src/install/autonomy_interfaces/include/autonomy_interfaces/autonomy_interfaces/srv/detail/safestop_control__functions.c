// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/SafestopControl.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/safestop_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
// Member `operator_id`
// Member `reason`
// Member `affected_subsystems`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__SafestopControl_Request__init(autonomy_interfaces__srv__SafestopControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    autonomy_interfaces__srv__SafestopControl_Request__fini(msg);
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__srv__SafestopControl_Request__fini(msg);
    return false;
  }
  // acknowledge_risks
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    autonomy_interfaces__srv__SafestopControl_Request__fini(msg);
    return false;
  }
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->affected_subsystems, 0)) {
    autonomy_interfaces__srv__SafestopControl_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__SafestopControl_Request__fini(autonomy_interfaces__srv__SafestopControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
  // acknowledge_risks
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // affected_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->affected_subsystems);
}

bool
autonomy_interfaces__srv__SafestopControl_Request__are_equal(const autonomy_interfaces__srv__SafestopControl_Request * lhs, const autonomy_interfaces__srv__SafestopControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  // acknowledge_risks
  if (lhs->acknowledge_risks != rhs->acknowledge_risks) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->affected_subsystems), &(rhs->affected_subsystems)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__SafestopControl_Request__copy(
  const autonomy_interfaces__srv__SafestopControl_Request * input,
  autonomy_interfaces__srv__SafestopControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  // acknowledge_risks
  output->acknowledge_risks = input->acknowledge_risks;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->affected_subsystems), &(output->affected_subsystems)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__SafestopControl_Request *
autonomy_interfaces__srv__SafestopControl_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Request * msg = (autonomy_interfaces__srv__SafestopControl_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__SafestopControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__SafestopControl_Request));
  bool success = autonomy_interfaces__srv__SafestopControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__SafestopControl_Request__destroy(autonomy_interfaces__srv__SafestopControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__SafestopControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__SafestopControl_Request__Sequence__init(autonomy_interfaces__srv__SafestopControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__SafestopControl_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__SafestopControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__SafestopControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__SafestopControl_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__SafestopControl_Request__Sequence__fini(autonomy_interfaces__srv__SafestopControl_Request__Sequence * array)
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
      autonomy_interfaces__srv__SafestopControl_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__SafestopControl_Request__Sequence *
autonomy_interfaces__srv__SafestopControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Request__Sequence * array = (autonomy_interfaces__srv__SafestopControl_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__SafestopControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__SafestopControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__SafestopControl_Request__Sequence__destroy(autonomy_interfaces__srv__SafestopControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__SafestopControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__SafestopControl_Request__Sequence__are_equal(const autonomy_interfaces__srv__SafestopControl_Request__Sequence * lhs, const autonomy_interfaces__srv__SafestopControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__SafestopControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__SafestopControl_Request__Sequence__copy(
  const autonomy_interfaces__srv__SafestopControl_Request__Sequence * input,
  autonomy_interfaces__srv__SafestopControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__SafestopControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__SafestopControl_Request * data =
      (autonomy_interfaces__srv__SafestopControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__SafestopControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__SafestopControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__SafestopControl_Request__copy(
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
// Member `affected_subsystems`
// Member `operator_id`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__SafestopControl_Response__init(autonomy_interfaces__srv__SafestopControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__SafestopControl_Response__fini(msg);
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    autonomy_interfaces__srv__SafestopControl_Response__fini(msg);
    return false;
  }
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->affected_subsystems, 0)) {
    autonomy_interfaces__srv__SafestopControl_Response__fini(msg);
    return false;
  }
  // timestamp
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__srv__SafestopControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__SafestopControl_Response__fini(autonomy_interfaces__srv__SafestopControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // affected_subsystems
  rosidl_runtime_c__String__Sequence__fini(&msg->affected_subsystems);
  // timestamp
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
}

bool
autonomy_interfaces__srv__SafestopControl_Response__are_equal(const autonomy_interfaces__srv__SafestopControl_Response * lhs, const autonomy_interfaces__srv__SafestopControl_Response * rhs)
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
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->affected_subsystems), &(rhs->affected_subsystems)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__SafestopControl_Response__copy(
  const autonomy_interfaces__srv__SafestopControl_Response * input,
  autonomy_interfaces__srv__SafestopControl_Response * output)
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
  // affected_subsystems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->affected_subsystems), &(output->affected_subsystems)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__SafestopControl_Response *
autonomy_interfaces__srv__SafestopControl_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Response * msg = (autonomy_interfaces__srv__SafestopControl_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__SafestopControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__SafestopControl_Response));
  bool success = autonomy_interfaces__srv__SafestopControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__SafestopControl_Response__destroy(autonomy_interfaces__srv__SafestopControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__SafestopControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__SafestopControl_Response__Sequence__init(autonomy_interfaces__srv__SafestopControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__SafestopControl_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__SafestopControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__SafestopControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__SafestopControl_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__SafestopControl_Response__Sequence__fini(autonomy_interfaces__srv__SafestopControl_Response__Sequence * array)
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
      autonomy_interfaces__srv__SafestopControl_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__SafestopControl_Response__Sequence *
autonomy_interfaces__srv__SafestopControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__SafestopControl_Response__Sequence * array = (autonomy_interfaces__srv__SafestopControl_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__SafestopControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__SafestopControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__SafestopControl_Response__Sequence__destroy(autonomy_interfaces__srv__SafestopControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__SafestopControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__SafestopControl_Response__Sequence__are_equal(const autonomy_interfaces__srv__SafestopControl_Response__Sequence * lhs, const autonomy_interfaces__srv__SafestopControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__SafestopControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__SafestopControl_Response__Sequence__copy(
  const autonomy_interfaces__srv__SafestopControl_Response__Sequence * input,
  autonomy_interfaces__srv__SafestopControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__SafestopControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__SafestopControl_Response * data =
      (autonomy_interfaces__srv__SafestopControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__SafestopControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__SafestopControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__SafestopControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
