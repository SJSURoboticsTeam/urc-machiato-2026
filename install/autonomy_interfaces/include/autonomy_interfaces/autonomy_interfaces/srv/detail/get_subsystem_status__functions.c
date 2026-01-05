// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/GetSubsystemStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/get_subsystem_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `subsystem_name`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__init(autonomy_interfaces__srv__GetSubsystemStatus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // subsystem_name
  if (!rosidl_runtime_c__String__init(&msg->subsystem_name)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(autonomy_interfaces__srv__GetSubsystemStatus_Request * msg)
{
  if (!msg) {
    return;
  }
  // subsystem_name
  rosidl_runtime_c__String__fini(&msg->subsystem_name);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Request * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // subsystem_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->subsystem_name), &(rhs->subsystem_name)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Request * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // subsystem_name
  if (!rosidl_runtime_c__String__copy(
      &(input->subsystem_name), &(output->subsystem_name)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetSubsystemStatus_Request *
autonomy_interfaces__srv__GetSubsystemStatus_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Request * msg = (autonomy_interfaces__srv__GetSubsystemStatus_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Request));
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Request__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__init(autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSubsystemStatus_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSubsystemStatus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__fini(autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * array)
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
      autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * array = (autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSubsystemStatus_Request * data =
      (autonomy_interfaces__srv__GetSubsystemStatus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSubsystemStatus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_message`
// Member `subsystem_names`
// Member `subsystem_states`
// Member `status_messages`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `subsystem_health`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__init(autonomy_interfaces__srv__GetSubsystemStatus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
    return false;
  }
  // subsystem_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->subsystem_names, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
    return false;
  }
  // subsystem_states
  if (!rosidl_runtime_c__String__Sequence__init(&msg->subsystem_states, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
    return false;
  }
  // subsystem_health
  if (!rosidl_runtime_c__float__Sequence__init(&msg->subsystem_health, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
    return false;
  }
  // status_messages
  if (!rosidl_runtime_c__String__Sequence__init(&msg->status_messages, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(autonomy_interfaces__srv__GetSubsystemStatus_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // subsystem_names
  rosidl_runtime_c__String__Sequence__fini(&msg->subsystem_names);
  // subsystem_states
  rosidl_runtime_c__String__Sequence__fini(&msg->subsystem_states);
  // subsystem_health
  rosidl_runtime_c__float__Sequence__fini(&msg->subsystem_health);
  // status_messages
  rosidl_runtime_c__String__Sequence__fini(&msg->status_messages);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Response * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // subsystem_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->subsystem_names), &(rhs->subsystem_names)))
  {
    return false;
  }
  // subsystem_states
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->subsystem_states), &(rhs->subsystem_states)))
  {
    return false;
  }
  // subsystem_health
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->subsystem_health), &(rhs->subsystem_health)))
  {
    return false;
  }
  // status_messages
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->status_messages), &(rhs->status_messages)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Response * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // subsystem_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->subsystem_names), &(output->subsystem_names)))
  {
    return false;
  }
  // subsystem_states
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->subsystem_states), &(output->subsystem_states)))
  {
    return false;
  }
  // subsystem_health
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->subsystem_health), &(output->subsystem_health)))
  {
    return false;
  }
  // status_messages
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->status_messages), &(output->status_messages)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetSubsystemStatus_Response *
autonomy_interfaces__srv__GetSubsystemStatus_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Response * msg = (autonomy_interfaces__srv__GetSubsystemStatus_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Response));
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Response__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__init(autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSubsystemStatus_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSubsystemStatus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__fini(autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * array)
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
      autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * array = (autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSubsystemStatus_Response * data =
      (autonomy_interfaces__srv__GetSubsystemStatus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSubsystemStatus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/get_subsystem_status__functions.h"

bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__init(autonomy_interfaces__srv__GetSubsystemStatus_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(autonomy_interfaces__srv__GetSubsystemStatus_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Event * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Event * rhs)
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
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Event * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Event * output)
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
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetSubsystemStatus_Event *
autonomy_interfaces__srv__GetSubsystemStatus_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Event * msg = (autonomy_interfaces__srv__GetSubsystemStatus_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Event));
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Event__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__init(autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetSubsystemStatus_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetSubsystemStatus_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__fini(autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * array)
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
      autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence *
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * array = (autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__destroy(autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__are_equal(const autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * lhs, const autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence__copy(
  const autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * input,
  autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetSubsystemStatus_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetSubsystemStatus_Event * data =
      (autonomy_interfaces__srv__GetSubsystemStatus_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetSubsystemStatus_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetSubsystemStatus_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetSubsystemStatus_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
