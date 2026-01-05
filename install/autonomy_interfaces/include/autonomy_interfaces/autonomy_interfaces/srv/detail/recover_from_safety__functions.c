// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/recover_from_safety__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `recovery_method`
// Member `operator_id`
// Member `completed_steps`
// Member `notes`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__RecoverFromSafety_Request__init(autonomy_interfaces__srv__RecoverFromSafety_Request * msg)
{
  if (!msg) {
    return false;
  }
  // recovery_method
  if (!rosidl_runtime_c__String__init(&msg->recovery_method)) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__fini(msg);
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__fini(msg);
    return false;
  }
  // acknowledge_risks
  // completed_steps
  if (!rosidl_runtime_c__String__Sequence__init(&msg->completed_steps, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__fini(msg);
    return false;
  }
  // notes
  if (!rosidl_runtime_c__String__init(&msg->notes)) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Request__fini(autonomy_interfaces__srv__RecoverFromSafety_Request * msg)
{
  if (!msg) {
    return;
  }
  // recovery_method
  rosidl_runtime_c__String__fini(&msg->recovery_method);
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
  // acknowledge_risks
  // completed_steps
  rosidl_runtime_c__String__Sequence__fini(&msg->completed_steps);
  // notes
  rosidl_runtime_c__String__fini(&msg->notes);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Request__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Request * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // recovery_method
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recovery_method), &(rhs->recovery_method)))
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
  // completed_steps
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->completed_steps), &(rhs->completed_steps)))
  {
    return false;
  }
  // notes
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->notes), &(rhs->notes)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Request__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Request * input,
  autonomy_interfaces__srv__RecoverFromSafety_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // recovery_method
  if (!rosidl_runtime_c__String__copy(
      &(input->recovery_method), &(output->recovery_method)))
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
  // completed_steps
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->completed_steps), &(output->completed_steps)))
  {
    return false;
  }
  // notes
  if (!rosidl_runtime_c__String__copy(
      &(input->notes), &(output->notes)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__RecoverFromSafety_Request *
autonomy_interfaces__srv__RecoverFromSafety_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Request * msg = (autonomy_interfaces__srv__RecoverFromSafety_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Request));
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Request__destroy(autonomy_interfaces__srv__RecoverFromSafety_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__init(autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__RecoverFromSafety_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__RecoverFromSafety_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__RecoverFromSafety_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__fini(autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * array)
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
      autonomy_interfaces__srv__RecoverFromSafety_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * array = (autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__destroy(autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * input,
  autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__RecoverFromSafety_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__RecoverFromSafety_Request * data =
      (autonomy_interfaces__srv__RecoverFromSafety_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__RecoverFromSafety_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__RecoverFromSafety_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `recovery_state`
// Member `remaining_steps`
// Member `verified_systems`
// Member `failed_systems`
// Member `recommended_next_state`
// Member `restrictions`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__RecoverFromSafety_Response__init(autonomy_interfaces__srv__RecoverFromSafety_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // recovery_state
  if (!rosidl_runtime_c__String__init(&msg->recovery_state)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // is_safe_to_proceed
  // remaining_steps
  if (!rosidl_runtime_c__String__Sequence__init(&msg->remaining_steps, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // verified_systems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->verified_systems, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // failed_systems
  if (!rosidl_runtime_c__String__Sequence__init(&msg->failed_systems, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // estimated_time
  // recommended_next_state
  if (!rosidl_runtime_c__String__init(&msg->recommended_next_state)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  // restrictions
  if (!rosidl_runtime_c__String__Sequence__init(&msg->restrictions, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Response__fini(autonomy_interfaces__srv__RecoverFromSafety_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // recovery_state
  rosidl_runtime_c__String__fini(&msg->recovery_state);
  // is_safe_to_proceed
  // remaining_steps
  rosidl_runtime_c__String__Sequence__fini(&msg->remaining_steps);
  // verified_systems
  rosidl_runtime_c__String__Sequence__fini(&msg->verified_systems);
  // failed_systems
  rosidl_runtime_c__String__Sequence__fini(&msg->failed_systems);
  // estimated_time
  // recommended_next_state
  rosidl_runtime_c__String__fini(&msg->recommended_next_state);
  // restrictions
  rosidl_runtime_c__String__Sequence__fini(&msg->restrictions);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Response__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Response * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Response * rhs)
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
  // recovery_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recovery_state), &(rhs->recovery_state)))
  {
    return false;
  }
  // is_safe_to_proceed
  if (lhs->is_safe_to_proceed != rhs->is_safe_to_proceed) {
    return false;
  }
  // remaining_steps
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->remaining_steps), &(rhs->remaining_steps)))
  {
    return false;
  }
  // verified_systems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->verified_systems), &(rhs->verified_systems)))
  {
    return false;
  }
  // failed_systems
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->failed_systems), &(rhs->failed_systems)))
  {
    return false;
  }
  // estimated_time
  if (lhs->estimated_time != rhs->estimated_time) {
    return false;
  }
  // recommended_next_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recommended_next_state), &(rhs->recommended_next_state)))
  {
    return false;
  }
  // restrictions
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->restrictions), &(rhs->restrictions)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Response__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Response * input,
  autonomy_interfaces__srv__RecoverFromSafety_Response * output)
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
  // recovery_state
  if (!rosidl_runtime_c__String__copy(
      &(input->recovery_state), &(output->recovery_state)))
  {
    return false;
  }
  // is_safe_to_proceed
  output->is_safe_to_proceed = input->is_safe_to_proceed;
  // remaining_steps
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->remaining_steps), &(output->remaining_steps)))
  {
    return false;
  }
  // verified_systems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->verified_systems), &(output->verified_systems)))
  {
    return false;
  }
  // failed_systems
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->failed_systems), &(output->failed_systems)))
  {
    return false;
  }
  // estimated_time
  output->estimated_time = input->estimated_time;
  // recommended_next_state
  if (!rosidl_runtime_c__String__copy(
      &(input->recommended_next_state), &(output->recommended_next_state)))
  {
    return false;
  }
  // restrictions
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->restrictions), &(output->restrictions)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__RecoverFromSafety_Response *
autonomy_interfaces__srv__RecoverFromSafety_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Response * msg = (autonomy_interfaces__srv__RecoverFromSafety_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Response));
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Response__destroy(autonomy_interfaces__srv__RecoverFromSafety_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__init(autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__RecoverFromSafety_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__RecoverFromSafety_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__RecoverFromSafety_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__fini(autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * array)
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
      autonomy_interfaces__srv__RecoverFromSafety_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * array = (autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__destroy(autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * input,
  autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__RecoverFromSafety_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__RecoverFromSafety_Response * data =
      (autonomy_interfaces__srv__RecoverFromSafety_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__RecoverFromSafety_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__RecoverFromSafety_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/recover_from_safety__functions.h"

bool
autonomy_interfaces__srv__RecoverFromSafety_Event__init(autonomy_interfaces__srv__RecoverFromSafety_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__RecoverFromSafety_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__RecoverFromSafety_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Event__fini(autonomy_interfaces__srv__RecoverFromSafety_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Event__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Event * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Event * rhs)
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
  if (!autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Event__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Event * input,
  autonomy_interfaces__srv__RecoverFromSafety_Event * output)
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
  if (!autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__RecoverFromSafety_Event *
autonomy_interfaces__srv__RecoverFromSafety_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Event * msg = (autonomy_interfaces__srv__RecoverFromSafety_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Event));
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Event__destroy(autonomy_interfaces__srv__RecoverFromSafety_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__RecoverFromSafety_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__init(autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__RecoverFromSafety_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__RecoverFromSafety_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__RecoverFromSafety_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__RecoverFromSafety_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__fini(autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * array)
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
      autonomy_interfaces__srv__RecoverFromSafety_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence *
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * array = (autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__destroy(autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__are_equal(const autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * lhs, const autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence__copy(
  const autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * input,
  autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__RecoverFromSafety_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__RecoverFromSafety_Event * data =
      (autonomy_interfaces__srv__RecoverFromSafety_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__RecoverFromSafety_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__RecoverFromSafety_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__RecoverFromSafety_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
