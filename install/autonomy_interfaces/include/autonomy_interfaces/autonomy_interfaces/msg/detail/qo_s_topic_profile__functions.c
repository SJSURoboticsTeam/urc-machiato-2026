// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `topic_name`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__QoSTopicProfile__init(autonomy_interfaces__msg__QoSTopicProfile * msg)
{
  if (!msg) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__init(&msg->topic_name)) {
    autonomy_interfaces__msg__QoSTopicProfile__fini(msg);
    return false;
  }
  // avg_latency_ms
  // jitter_ms
  // packet_loss_rate
  // samples_count
  return true;
}

void
autonomy_interfaces__msg__QoSTopicProfile__fini(autonomy_interfaces__msg__QoSTopicProfile * msg)
{
  if (!msg) {
    return;
  }
  // topic_name
  rosidl_runtime_c__String__fini(&msg->topic_name);
  // avg_latency_ms
  // jitter_ms
  // packet_loss_rate
  // samples_count
}

bool
autonomy_interfaces__msg__QoSTopicProfile__are_equal(const autonomy_interfaces__msg__QoSTopicProfile * lhs, const autonomy_interfaces__msg__QoSTopicProfile * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic_name), &(rhs->topic_name)))
  {
    return false;
  }
  // avg_latency_ms
  if (lhs->avg_latency_ms != rhs->avg_latency_ms) {
    return false;
  }
  // jitter_ms
  if (lhs->jitter_ms != rhs->jitter_ms) {
    return false;
  }
  // packet_loss_rate
  if (lhs->packet_loss_rate != rhs->packet_loss_rate) {
    return false;
  }
  // samples_count
  if (lhs->samples_count != rhs->samples_count) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__QoSTopicProfile__copy(
  const autonomy_interfaces__msg__QoSTopicProfile * input,
  autonomy_interfaces__msg__QoSTopicProfile * output)
{
  if (!input || !output) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__copy(
      &(input->topic_name), &(output->topic_name)))
  {
    return false;
  }
  // avg_latency_ms
  output->avg_latency_ms = input->avg_latency_ms;
  // jitter_ms
  output->jitter_ms = input->jitter_ms;
  // packet_loss_rate
  output->packet_loss_rate = input->packet_loss_rate;
  // samples_count
  output->samples_count = input->samples_count;
  return true;
}

autonomy_interfaces__msg__QoSTopicProfile *
autonomy_interfaces__msg__QoSTopicProfile__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSTopicProfile * msg = (autonomy_interfaces__msg__QoSTopicProfile *)allocator.allocate(sizeof(autonomy_interfaces__msg__QoSTopicProfile), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__QoSTopicProfile));
  bool success = autonomy_interfaces__msg__QoSTopicProfile__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__QoSTopicProfile__destroy(autonomy_interfaces__msg__QoSTopicProfile * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__QoSTopicProfile__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__QoSTopicProfile__Sequence__init(autonomy_interfaces__msg__QoSTopicProfile__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSTopicProfile * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__QoSTopicProfile *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__QoSTopicProfile), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__QoSTopicProfile__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__QoSTopicProfile__fini(&data[i - 1]);
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
autonomy_interfaces__msg__QoSTopicProfile__Sequence__fini(autonomy_interfaces__msg__QoSTopicProfile__Sequence * array)
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
      autonomy_interfaces__msg__QoSTopicProfile__fini(&array->data[i]);
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

autonomy_interfaces__msg__QoSTopicProfile__Sequence *
autonomy_interfaces__msg__QoSTopicProfile__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSTopicProfile__Sequence * array = (autonomy_interfaces__msg__QoSTopicProfile__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__QoSTopicProfile__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__QoSTopicProfile__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__QoSTopicProfile__Sequence__destroy(autonomy_interfaces__msg__QoSTopicProfile__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__QoSTopicProfile__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__QoSTopicProfile__Sequence__are_equal(const autonomy_interfaces__msg__QoSTopicProfile__Sequence * lhs, const autonomy_interfaces__msg__QoSTopicProfile__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__QoSTopicProfile__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__QoSTopicProfile__Sequence__copy(
  const autonomy_interfaces__msg__QoSTopicProfile__Sequence * input,
  autonomy_interfaces__msg__QoSTopicProfile__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__QoSTopicProfile);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__QoSTopicProfile * data =
      (autonomy_interfaces__msg__QoSTopicProfile *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__QoSTopicProfile__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__QoSTopicProfile__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__QoSTopicProfile__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
