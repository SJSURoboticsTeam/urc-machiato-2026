// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/get_aoi_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `sensor_name`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__GetAOIStatus_Request__init(autonomy_interfaces__srv__GetAOIStatus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__init(&msg->sensor_name)) {
    autonomy_interfaces__srv__GetAOIStatus_Request__fini(msg);
    return false;
  }
  // include_history
  // history_samples
  return true;
}

void
autonomy_interfaces__srv__GetAOIStatus_Request__fini(autonomy_interfaces__srv__GetAOIStatus_Request * msg)
{
  if (!msg) {
    return;
  }
  // sensor_name
  rosidl_runtime_c__String__fini(&msg->sensor_name);
  // include_history
  // history_samples
}

bool
autonomy_interfaces__srv__GetAOIStatus_Request__are_equal(const autonomy_interfaces__srv__GetAOIStatus_Request * lhs, const autonomy_interfaces__srv__GetAOIStatus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sensor_name), &(rhs->sensor_name)))
  {
    return false;
  }
  // include_history
  if (lhs->include_history != rhs->include_history) {
    return false;
  }
  // history_samples
  if (lhs->history_samples != rhs->history_samples) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetAOIStatus_Request__copy(
  const autonomy_interfaces__srv__GetAOIStatus_Request * input,
  autonomy_interfaces__srv__GetAOIStatus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__copy(
      &(input->sensor_name), &(output->sensor_name)))
  {
    return false;
  }
  // include_history
  output->include_history = input->include_history;
  // history_samples
  output->history_samples = input->history_samples;
  return true;
}

autonomy_interfaces__srv__GetAOIStatus_Request *
autonomy_interfaces__srv__GetAOIStatus_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Request * msg = (autonomy_interfaces__srv__GetAOIStatus_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetAOIStatus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetAOIStatus_Request));
  bool success = autonomy_interfaces__srv__GetAOIStatus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetAOIStatus_Request__destroy(autonomy_interfaces__srv__GetAOIStatus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetAOIStatus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__init(autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetAOIStatus_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetAOIStatus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetAOIStatus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetAOIStatus_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__fini(autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * array)
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
      autonomy_interfaces__srv__GetAOIStatus_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * array = (autonomy_interfaces__srv__GetAOIStatus_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetAOIStatus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__destroy(autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__are_equal(const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * lhs, const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetAOIStatus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetAOIStatus_Request__Sequence__copy(
  const autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * input,
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetAOIStatus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetAOIStatus_Request * data =
      (autonomy_interfaces__srv__GetAOIStatus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetAOIStatus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetAOIStatus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetAOIStatus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `sensor_status`
#include "autonomy_interfaces/msg/detail/aoi_status__functions.h"
// Member `aoi_history`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `timestamp_history`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `system_metrics`
#include "autonomy_interfaces/msg/detail/aoi_metrics__functions.h"

bool
autonomy_interfaces__srv__GetAOIStatus_Response__init(autonomy_interfaces__srv__GetAOIStatus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
    return false;
  }
  // sensor_status
  if (!autonomy_interfaces__msg__AOIStatus__Sequence__init(&msg->sensor_status, 0)) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
    return false;
  }
  // aoi_history
  if (!rosidl_runtime_c__double__Sequence__init(&msg->aoi_history, 0)) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
    return false;
  }
  // timestamp_history
  if (!builtin_interfaces__msg__Time__Sequence__init(&msg->timestamp_history, 0)) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
    return false;
  }
  // system_metrics
  if (!autonomy_interfaces__msg__AOIMetrics__init(&msg->system_metrics)) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetAOIStatus_Response__fini(autonomy_interfaces__srv__GetAOIStatus_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // sensor_status
  autonomy_interfaces__msg__AOIStatus__Sequence__fini(&msg->sensor_status);
  // aoi_history
  rosidl_runtime_c__double__Sequence__fini(&msg->aoi_history);
  // timestamp_history
  builtin_interfaces__msg__Time__Sequence__fini(&msg->timestamp_history);
  // system_metrics
  autonomy_interfaces__msg__AOIMetrics__fini(&msg->system_metrics);
}

bool
autonomy_interfaces__srv__GetAOIStatus_Response__are_equal(const autonomy_interfaces__srv__GetAOIStatus_Response * lhs, const autonomy_interfaces__srv__GetAOIStatus_Response * rhs)
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
  // sensor_status
  if (!autonomy_interfaces__msg__AOIStatus__Sequence__are_equal(
      &(lhs->sensor_status), &(rhs->sensor_status)))
  {
    return false;
  }
  // aoi_history
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->aoi_history), &(rhs->aoi_history)))
  {
    return false;
  }
  // timestamp_history
  if (!builtin_interfaces__msg__Time__Sequence__are_equal(
      &(lhs->timestamp_history), &(rhs->timestamp_history)))
  {
    return false;
  }
  // system_metrics
  if (!autonomy_interfaces__msg__AOIMetrics__are_equal(
      &(lhs->system_metrics), &(rhs->system_metrics)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetAOIStatus_Response__copy(
  const autonomy_interfaces__srv__GetAOIStatus_Response * input,
  autonomy_interfaces__srv__GetAOIStatus_Response * output)
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
  // sensor_status
  if (!autonomy_interfaces__msg__AOIStatus__Sequence__copy(
      &(input->sensor_status), &(output->sensor_status)))
  {
    return false;
  }
  // aoi_history
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->aoi_history), &(output->aoi_history)))
  {
    return false;
  }
  // timestamp_history
  if (!builtin_interfaces__msg__Time__Sequence__copy(
      &(input->timestamp_history), &(output->timestamp_history)))
  {
    return false;
  }
  // system_metrics
  if (!autonomy_interfaces__msg__AOIMetrics__copy(
      &(input->system_metrics), &(output->system_metrics)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetAOIStatus_Response *
autonomy_interfaces__srv__GetAOIStatus_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Response * msg = (autonomy_interfaces__srv__GetAOIStatus_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetAOIStatus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetAOIStatus_Response));
  bool success = autonomy_interfaces__srv__GetAOIStatus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetAOIStatus_Response__destroy(autonomy_interfaces__srv__GetAOIStatus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetAOIStatus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__init(autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetAOIStatus_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetAOIStatus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetAOIStatus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetAOIStatus_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__fini(autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * array)
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
      autonomy_interfaces__srv__GetAOIStatus_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * array = (autonomy_interfaces__srv__GetAOIStatus_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetAOIStatus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__destroy(autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__are_equal(const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * lhs, const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetAOIStatus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetAOIStatus_Response__Sequence__copy(
  const autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * input,
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetAOIStatus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetAOIStatus_Response * data =
      (autonomy_interfaces__srv__GetAOIStatus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetAOIStatus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetAOIStatus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetAOIStatus_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
