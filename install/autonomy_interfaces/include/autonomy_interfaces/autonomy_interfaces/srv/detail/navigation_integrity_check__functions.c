// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/NavigationIntegrityCheck.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `check_components`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Request * msg)
{
  if (!msg) {
    return false;
  }
  // detailed_check
  // check_components
  if (!rosidl_runtime_c__String__Sequence__init(&msg->check_components, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Request * msg)
{
  if (!msg) {
    return;
  }
  // detailed_check
  // check_components
  rosidl_runtime_c__String__Sequence__fini(&msg->check_components);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Request * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // detailed_check
  if (lhs->detailed_check != rhs->detailed_check) {
    return false;
  }
  // check_components
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->check_components), &(rhs->check_components)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // detailed_check
  output->detailed_check = input->detailed_check;
  // check_components
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->check_components), &(output->check_components)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__NavigationIntegrityCheck_Request *
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * msg = (autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request));
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * array)
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
      autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * array = (autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__NavigationIntegrityCheck_Request * data =
      (autonomy_interfaces__srv__NavigationIntegrityCheck_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__NavigationIntegrityCheck_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `integrity_level`
// Member `checked_components`
// Member `component_details`
// Member `recommendations`
// Member `timestamp`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `component_status`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Response * msg)
{
  if (!msg) {
    return false;
  }
  // integrity_ok
  // integrity_score
  // integrity_level
  if (!rosidl_runtime_c__String__init(&msg->integrity_level)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  // checked_components
  if (!rosidl_runtime_c__String__Sequence__init(&msg->checked_components, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  // component_status
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->component_status, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  // component_details
  if (!rosidl_runtime_c__String__Sequence__init(&msg->component_details, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  // position_accuracy
  // heading_accuracy
  // velocity_consistency
  // satellite_count
  // hdop
  // recommendations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->recommendations, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__init(&msg->timestamp)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Response * msg)
{
  if (!msg) {
    return;
  }
  // integrity_ok
  // integrity_score
  // integrity_level
  rosidl_runtime_c__String__fini(&msg->integrity_level);
  // checked_components
  rosidl_runtime_c__String__Sequence__fini(&msg->checked_components);
  // component_status
  rosidl_runtime_c__boolean__Sequence__fini(&msg->component_status);
  // component_details
  rosidl_runtime_c__String__Sequence__fini(&msg->component_details);
  // position_accuracy
  // heading_accuracy
  // velocity_consistency
  // satellite_count
  // hdop
  // recommendations
  rosidl_runtime_c__String__Sequence__fini(&msg->recommendations);
  // timestamp
  rosidl_runtime_c__String__fini(&msg->timestamp);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Response * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // integrity_ok
  if (lhs->integrity_ok != rhs->integrity_ok) {
    return false;
  }
  // integrity_score
  if (lhs->integrity_score != rhs->integrity_score) {
    return false;
  }
  // integrity_level
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->integrity_level), &(rhs->integrity_level)))
  {
    return false;
  }
  // checked_components
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->checked_components), &(rhs->checked_components)))
  {
    return false;
  }
  // component_status
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->component_status), &(rhs->component_status)))
  {
    return false;
  }
  // component_details
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->component_details), &(rhs->component_details)))
  {
    return false;
  }
  // position_accuracy
  if (lhs->position_accuracy != rhs->position_accuracy) {
    return false;
  }
  // heading_accuracy
  if (lhs->heading_accuracy != rhs->heading_accuracy) {
    return false;
  }
  // velocity_consistency
  if (lhs->velocity_consistency != rhs->velocity_consistency) {
    return false;
  }
  // satellite_count
  if (lhs->satellite_count != rhs->satellite_count) {
    return false;
  }
  // hdop
  if (lhs->hdop != rhs->hdop) {
    return false;
  }
  // recommendations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->recommendations), &(rhs->recommendations)))
  {
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // integrity_ok
  output->integrity_ok = input->integrity_ok;
  // integrity_score
  output->integrity_score = input->integrity_score;
  // integrity_level
  if (!rosidl_runtime_c__String__copy(
      &(input->integrity_level), &(output->integrity_level)))
  {
    return false;
  }
  // checked_components
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->checked_components), &(output->checked_components)))
  {
    return false;
  }
  // component_status
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->component_status), &(output->component_status)))
  {
    return false;
  }
  // component_details
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->component_details), &(output->component_details)))
  {
    return false;
  }
  // position_accuracy
  output->position_accuracy = input->position_accuracy;
  // heading_accuracy
  output->heading_accuracy = input->heading_accuracy;
  // velocity_consistency
  output->velocity_consistency = input->velocity_consistency;
  // satellite_count
  output->satellite_count = input->satellite_count;
  // hdop
  output->hdop = input->hdop;
  // recommendations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->recommendations), &(output->recommendations)))
  {
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__NavigationIntegrityCheck_Response *
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * msg = (autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response));
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * array)
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
      autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * array = (autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__NavigationIntegrityCheck_Response * data =
      (autonomy_interfaces__srv__NavigationIntegrityCheck_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__NavigationIntegrityCheck_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/navigation_integrity_check__functions.h"

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Event * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Event * rhs)
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
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Event * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event * output)
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
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__NavigationIntegrityCheck_Event *
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event * msg = (autonomy_interfaces__srv__NavigationIntegrityCheck_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event));
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__init(autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__NavigationIntegrityCheck_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__fini(autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * array)
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
      autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence *
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * array = (autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__destroy(autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__are_equal(const autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * lhs, const autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence__copy(
  const autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * input,
  autonomy_interfaces__srv__NavigationIntegrityCheck_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__NavigationIntegrityCheck_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__NavigationIntegrityCheck_Event * data =
      (autonomy_interfaces__srv__NavigationIntegrityCheck_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__NavigationIntegrityCheck_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__NavigationIntegrityCheck_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
