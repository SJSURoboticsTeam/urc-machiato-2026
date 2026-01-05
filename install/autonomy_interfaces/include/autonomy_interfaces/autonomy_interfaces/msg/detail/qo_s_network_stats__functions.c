// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/qo_s_network_stats__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `current_band`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__QoSNetworkStats__init(autonomy_interfaces__msg__QoSNetworkStats * msg)
{
  if (!msg) {
    return false;
  }
  // bandwidth_up_mbps
  // bandwidth_down_mbps
  // latency_ms
  // packet_loss_rate
  // current_band
  if (!rosidl_runtime_c__String__init(&msg->current_band)) {
    autonomy_interfaces__msg__QoSNetworkStats__fini(msg);
    return false;
  }
  // signal_strength
  return true;
}

void
autonomy_interfaces__msg__QoSNetworkStats__fini(autonomy_interfaces__msg__QoSNetworkStats * msg)
{
  if (!msg) {
    return;
  }
  // bandwidth_up_mbps
  // bandwidth_down_mbps
  // latency_ms
  // packet_loss_rate
  // current_band
  rosidl_runtime_c__String__fini(&msg->current_band);
  // signal_strength
}

bool
autonomy_interfaces__msg__QoSNetworkStats__are_equal(const autonomy_interfaces__msg__QoSNetworkStats * lhs, const autonomy_interfaces__msg__QoSNetworkStats * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bandwidth_up_mbps
  if (lhs->bandwidth_up_mbps != rhs->bandwidth_up_mbps) {
    return false;
  }
  // bandwidth_down_mbps
  if (lhs->bandwidth_down_mbps != rhs->bandwidth_down_mbps) {
    return false;
  }
  // latency_ms
  if (lhs->latency_ms != rhs->latency_ms) {
    return false;
  }
  // packet_loss_rate
  if (lhs->packet_loss_rate != rhs->packet_loss_rate) {
    return false;
  }
  // current_band
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_band), &(rhs->current_band)))
  {
    return false;
  }
  // signal_strength
  if (lhs->signal_strength != rhs->signal_strength) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__QoSNetworkStats__copy(
  const autonomy_interfaces__msg__QoSNetworkStats * input,
  autonomy_interfaces__msg__QoSNetworkStats * output)
{
  if (!input || !output) {
    return false;
  }
  // bandwidth_up_mbps
  output->bandwidth_up_mbps = input->bandwidth_up_mbps;
  // bandwidth_down_mbps
  output->bandwidth_down_mbps = input->bandwidth_down_mbps;
  // latency_ms
  output->latency_ms = input->latency_ms;
  // packet_loss_rate
  output->packet_loss_rate = input->packet_loss_rate;
  // current_band
  if (!rosidl_runtime_c__String__copy(
      &(input->current_band), &(output->current_band)))
  {
    return false;
  }
  // signal_strength
  output->signal_strength = input->signal_strength;
  return true;
}

autonomy_interfaces__msg__QoSNetworkStats *
autonomy_interfaces__msg__QoSNetworkStats__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSNetworkStats * msg = (autonomy_interfaces__msg__QoSNetworkStats *)allocator.allocate(sizeof(autonomy_interfaces__msg__QoSNetworkStats), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__QoSNetworkStats));
  bool success = autonomy_interfaces__msg__QoSNetworkStats__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__QoSNetworkStats__destroy(autonomy_interfaces__msg__QoSNetworkStats * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__QoSNetworkStats__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__QoSNetworkStats__Sequence__init(autonomy_interfaces__msg__QoSNetworkStats__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSNetworkStats * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__QoSNetworkStats *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__QoSNetworkStats), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__QoSNetworkStats__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__QoSNetworkStats__fini(&data[i - 1]);
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
autonomy_interfaces__msg__QoSNetworkStats__Sequence__fini(autonomy_interfaces__msg__QoSNetworkStats__Sequence * array)
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
      autonomy_interfaces__msg__QoSNetworkStats__fini(&array->data[i]);
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

autonomy_interfaces__msg__QoSNetworkStats__Sequence *
autonomy_interfaces__msg__QoSNetworkStats__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__QoSNetworkStats__Sequence * array = (autonomy_interfaces__msg__QoSNetworkStats__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__QoSNetworkStats__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__QoSNetworkStats__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__QoSNetworkStats__Sequence__destroy(autonomy_interfaces__msg__QoSNetworkStats__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__QoSNetworkStats__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__QoSNetworkStats__Sequence__are_equal(const autonomy_interfaces__msg__QoSNetworkStats__Sequence * lhs, const autonomy_interfaces__msg__QoSNetworkStats__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__QoSNetworkStats__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__QoSNetworkStats__Sequence__copy(
  const autonomy_interfaces__msg__QoSNetworkStats__Sequence * input,
  autonomy_interfaces__msg__QoSNetworkStats__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__QoSNetworkStats);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__QoSNetworkStats * data =
      (autonomy_interfaces__msg__QoSNetworkStats *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__QoSNetworkStats__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__QoSNetworkStats__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__QoSNetworkStats__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
