// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/GetQoSProfile.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/get_qo_s_profile__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
autonomy_interfaces__srv__GetQoSProfile_Request__init(autonomy_interfaces__srv__GetQoSProfile_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
autonomy_interfaces__srv__GetQoSProfile_Request__fini(autonomy_interfaces__srv__GetQoSProfile_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
autonomy_interfaces__srv__GetQoSProfile_Request__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Request * lhs, const autonomy_interfaces__srv__GetQoSProfile_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Request__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Request * input,
  autonomy_interfaces__srv__GetQoSProfile_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

autonomy_interfaces__srv__GetQoSProfile_Request *
autonomy_interfaces__srv__GetQoSProfile_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Request * msg = (autonomy_interfaces__srv__GetQoSProfile_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetQoSProfile_Request));
  bool success = autonomy_interfaces__srv__GetQoSProfile_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetQoSProfile_Request__destroy(autonomy_interfaces__srv__GetQoSProfile_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetQoSProfile_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__init(autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetQoSProfile_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetQoSProfile_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetQoSProfile_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetQoSProfile_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__fini(autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * array)
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
      autonomy_interfaces__srv__GetQoSProfile_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetQoSProfile_Request__Sequence *
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * array = (autonomy_interfaces__srv__GetQoSProfile_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__destroy(autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * lhs, const autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * input,
  autonomy_interfaces__srv__GetQoSProfile_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetQoSProfile_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetQoSProfile_Request * data =
      (autonomy_interfaces__srv__GetQoSProfile_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetQoSProfile_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetQoSProfile_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `topic_profiles`
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__functions.h"
// Member `network_stats`
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__functions.h"

bool
autonomy_interfaces__srv__GetQoSProfile_Response__init(autonomy_interfaces__srv__GetQoSProfile_Response * msg)
{
  if (!msg) {
    return false;
  }
  // topic_profiles
  if (!autonomy_interfaces__msg__QoSTopicProfile__Sequence__init(&msg->topic_profiles, 0)) {
    autonomy_interfaces__srv__GetQoSProfile_Response__fini(msg);
    return false;
  }
  // network_stats
  if (!autonomy_interfaces__msg__QoSNetworkStats__init(&msg->network_stats)) {
    autonomy_interfaces__srv__GetQoSProfile_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetQoSProfile_Response__fini(autonomy_interfaces__srv__GetQoSProfile_Response * msg)
{
  if (!msg) {
    return;
  }
  // topic_profiles
  autonomy_interfaces__msg__QoSTopicProfile__Sequence__fini(&msg->topic_profiles);
  // network_stats
  autonomy_interfaces__msg__QoSNetworkStats__fini(&msg->network_stats);
}

bool
autonomy_interfaces__srv__GetQoSProfile_Response__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Response * lhs, const autonomy_interfaces__srv__GetQoSProfile_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic_profiles
  if (!autonomy_interfaces__msg__QoSTopicProfile__Sequence__are_equal(
      &(lhs->topic_profiles), &(rhs->topic_profiles)))
  {
    return false;
  }
  // network_stats
  if (!autonomy_interfaces__msg__QoSNetworkStats__are_equal(
      &(lhs->network_stats), &(rhs->network_stats)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Response__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Response * input,
  autonomy_interfaces__srv__GetQoSProfile_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // topic_profiles
  if (!autonomy_interfaces__msg__QoSTopicProfile__Sequence__copy(
      &(input->topic_profiles), &(output->topic_profiles)))
  {
    return false;
  }
  // network_stats
  if (!autonomy_interfaces__msg__QoSNetworkStats__copy(
      &(input->network_stats), &(output->network_stats)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetQoSProfile_Response *
autonomy_interfaces__srv__GetQoSProfile_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Response * msg = (autonomy_interfaces__srv__GetQoSProfile_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetQoSProfile_Response));
  bool success = autonomy_interfaces__srv__GetQoSProfile_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetQoSProfile_Response__destroy(autonomy_interfaces__srv__GetQoSProfile_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetQoSProfile_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__init(autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetQoSProfile_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetQoSProfile_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetQoSProfile_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetQoSProfile_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__fini(autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * array)
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
      autonomy_interfaces__srv__GetQoSProfile_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetQoSProfile_Response__Sequence *
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * array = (autonomy_interfaces__srv__GetQoSProfile_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__destroy(autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * lhs, const autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * input,
  autonomy_interfaces__srv__GetQoSProfile_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetQoSProfile_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetQoSProfile_Response * data =
      (autonomy_interfaces__srv__GetQoSProfile_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetQoSProfile_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetQoSProfile_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/get_qo_s_profile__functions.h"

bool
autonomy_interfaces__srv__GetQoSProfile_Event__init(autonomy_interfaces__srv__GetQoSProfile_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__GetQoSProfile_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__GetQoSProfile_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__GetQoSProfile_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__GetQoSProfile_Event__fini(autonomy_interfaces__srv__GetQoSProfile_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__GetQoSProfile_Event__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Event * lhs, const autonomy_interfaces__srv__GetQoSProfile_Event * rhs)
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
  if (!autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Event__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Event * input,
  autonomy_interfaces__srv__GetQoSProfile_Event * output)
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
  if (!autonomy_interfaces__srv__GetQoSProfile_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__GetQoSProfile_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__GetQoSProfile_Event *
autonomy_interfaces__srv__GetQoSProfile_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Event * msg = (autonomy_interfaces__srv__GetQoSProfile_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__GetQoSProfile_Event));
  bool success = autonomy_interfaces__srv__GetQoSProfile_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__GetQoSProfile_Event__destroy(autonomy_interfaces__srv__GetQoSProfile_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__GetQoSProfile_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__init(autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__GetQoSProfile_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__GetQoSProfile_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__GetQoSProfile_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__GetQoSProfile_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__fini(autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * array)
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
      autonomy_interfaces__srv__GetQoSProfile_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__GetQoSProfile_Event__Sequence *
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * array = (autonomy_interfaces__srv__GetQoSProfile_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__GetQoSProfile_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__destroy(autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__are_equal(const autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * lhs, const autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__GetQoSProfile_Event__Sequence__copy(
  const autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * input,
  autonomy_interfaces__srv__GetQoSProfile_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__GetQoSProfile_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__GetQoSProfile_Event * data =
      (autonomy_interfaces__srv__GetQoSProfile_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__GetQoSProfile_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__GetQoSProfile_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__GetQoSProfile_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
