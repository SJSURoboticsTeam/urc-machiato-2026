// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `monitored_components`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__init(autonomy_interfaces__srv__TimingSafetyCheck_Request * msg)
{
  if (!msg) {
    return false;
  }
  // real_time_check
  // time_window
  // monitored_components
  if (!rosidl_runtime_c__String__Sequence__init(&msg->monitored_components, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(autonomy_interfaces__srv__TimingSafetyCheck_Request * msg)
{
  if (!msg) {
    return;
  }
  // real_time_check
  // time_window
  // monitored_components
  rosidl_runtime_c__String__Sequence__fini(&msg->monitored_components);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Request * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // real_time_check
  if (lhs->real_time_check != rhs->real_time_check) {
    return false;
  }
  // time_window
  if (lhs->time_window != rhs->time_window) {
    return false;
  }
  // monitored_components
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->monitored_components), &(rhs->monitored_components)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Request * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // real_time_check
  output->real_time_check = input->real_time_check;
  // time_window
  output->time_window = input->time_window;
  // monitored_components
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->monitored_components), &(output->monitored_components)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__TimingSafetyCheck_Request *
autonomy_interfaces__srv__TimingSafetyCheck_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Request * msg = (autonomy_interfaces__srv__TimingSafetyCheck_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request));
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Request__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__init(autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__TimingSafetyCheck_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__TimingSafetyCheck_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__fini(autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * array)
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
      autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * array = (autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__TimingSafetyCheck_Request * data =
      (autonomy_interfaces__srv__TimingSafetyCheck_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__TimingSafetyCheck_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `timing_status`
// Member `components_checked`
// Member `timing_recommendations`
// Member `timestamp`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `component_avg_times`
// Member `component_deadlines_missed`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__srv__TimingSafetyCheck_Response__init(autonomy_interfaces__srv__TimingSafetyCheck_Response * msg)
{
  if (!msg) {
    return false;
  }
  // timing_safe
  // timing_status
  if (!rosidl_runtime_c__String__init(&msg->timing_status)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  // avg_response_time
  // max_response_time
  // min_response_time
  // jitter
  // deadline_misses
  // deadline_miss_rate
  // components_checked
  if (!rosidl_runtime_c__String__Sequence__init(&msg->components_checked, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  // component_avg_times
  if (!rosidl_runtime_c__double__Sequence__init(&msg->component_avg_times, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  // component_deadlines_missed
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->component_deadlines_missed, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  // cpu_utilization
  // memory_utilization
  // thread_count
  // real_time_scheduling
  // timing_recommendations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->timing_recommendations, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__init(&msg->timestamp)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(autonomy_interfaces__srv__TimingSafetyCheck_Response * msg)
{
  if (!msg) {
    return;
  }
  // timing_safe
  // timing_status
  rosidl_runtime_c__String__fini(&msg->timing_status);
  // avg_response_time
  // max_response_time
  // min_response_time
  // jitter
  // deadline_misses
  // deadline_miss_rate
  // components_checked
  rosidl_runtime_c__String__Sequence__fini(&msg->components_checked);
  // component_avg_times
  rosidl_runtime_c__double__Sequence__fini(&msg->component_avg_times);
  // component_deadlines_missed
  rosidl_runtime_c__int32__Sequence__fini(&msg->component_deadlines_missed);
  // cpu_utilization
  // memory_utilization
  // thread_count
  // real_time_scheduling
  // timing_recommendations
  rosidl_runtime_c__String__Sequence__fini(&msg->timing_recommendations);
  // timestamp
  rosidl_runtime_c__String__fini(&msg->timestamp);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Response__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Response * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timing_safe
  if (lhs->timing_safe != rhs->timing_safe) {
    return false;
  }
  // timing_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->timing_status), &(rhs->timing_status)))
  {
    return false;
  }
  // avg_response_time
  if (lhs->avg_response_time != rhs->avg_response_time) {
    return false;
  }
  // max_response_time
  if (lhs->max_response_time != rhs->max_response_time) {
    return false;
  }
  // min_response_time
  if (lhs->min_response_time != rhs->min_response_time) {
    return false;
  }
  // jitter
  if (lhs->jitter != rhs->jitter) {
    return false;
  }
  // deadline_misses
  if (lhs->deadline_misses != rhs->deadline_misses) {
    return false;
  }
  // deadline_miss_rate
  if (lhs->deadline_miss_rate != rhs->deadline_miss_rate) {
    return false;
  }
  // components_checked
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->components_checked), &(rhs->components_checked)))
  {
    return false;
  }
  // component_avg_times
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->component_avg_times), &(rhs->component_avg_times)))
  {
    return false;
  }
  // component_deadlines_missed
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->component_deadlines_missed), &(rhs->component_deadlines_missed)))
  {
    return false;
  }
  // cpu_utilization
  if (lhs->cpu_utilization != rhs->cpu_utilization) {
    return false;
  }
  // memory_utilization
  if (lhs->memory_utilization != rhs->memory_utilization) {
    return false;
  }
  // thread_count
  if (lhs->thread_count != rhs->thread_count) {
    return false;
  }
  // real_time_scheduling
  if (lhs->real_time_scheduling != rhs->real_time_scheduling) {
    return false;
  }
  // timing_recommendations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->timing_recommendations), &(rhs->timing_recommendations)))
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
autonomy_interfaces__srv__TimingSafetyCheck_Response__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Response * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // timing_safe
  output->timing_safe = input->timing_safe;
  // timing_status
  if (!rosidl_runtime_c__String__copy(
      &(input->timing_status), &(output->timing_status)))
  {
    return false;
  }
  // avg_response_time
  output->avg_response_time = input->avg_response_time;
  // max_response_time
  output->max_response_time = input->max_response_time;
  // min_response_time
  output->min_response_time = input->min_response_time;
  // jitter
  output->jitter = input->jitter;
  // deadline_misses
  output->deadline_misses = input->deadline_misses;
  // deadline_miss_rate
  output->deadline_miss_rate = input->deadline_miss_rate;
  // components_checked
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->components_checked), &(output->components_checked)))
  {
    return false;
  }
  // component_avg_times
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->component_avg_times), &(output->component_avg_times)))
  {
    return false;
  }
  // component_deadlines_missed
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->component_deadlines_missed), &(output->component_deadlines_missed)))
  {
    return false;
  }
  // cpu_utilization
  output->cpu_utilization = input->cpu_utilization;
  // memory_utilization
  output->memory_utilization = input->memory_utilization;
  // thread_count
  output->thread_count = input->thread_count;
  // real_time_scheduling
  output->real_time_scheduling = input->real_time_scheduling;
  // timing_recommendations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->timing_recommendations), &(output->timing_recommendations)))
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

autonomy_interfaces__srv__TimingSafetyCheck_Response *
autonomy_interfaces__srv__TimingSafetyCheck_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Response * msg = (autonomy_interfaces__srv__TimingSafetyCheck_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response));
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Response__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__init(autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__TimingSafetyCheck_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__TimingSafetyCheck_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__fini(autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * array)
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
      autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * array = (autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__TimingSafetyCheck_Response * data =
      (autonomy_interfaces__srv__TimingSafetyCheck_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__TimingSafetyCheck_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/timing_safety_check__functions.h"

bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__init(autonomy_interfaces__srv__TimingSafetyCheck_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(autonomy_interfaces__srv__TimingSafetyCheck_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Event * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Event * rhs)
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
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Event * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Event * output)
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
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__TimingSafetyCheck_Event *
autonomy_interfaces__srv__TimingSafetyCheck_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Event * msg = (autonomy_interfaces__srv__TimingSafetyCheck_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event));
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Event__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__init(autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__TimingSafetyCheck_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__TimingSafetyCheck_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__fini(autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * array)
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
      autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence *
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * array = (autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__destroy(autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__are_equal(const autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * lhs, const autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence__copy(
  const autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * input,
  autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__TimingSafetyCheck_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__TimingSafetyCheck_Event * data =
      (autonomy_interfaces__srv__TimingSafetyCheck_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__TimingSafetyCheck_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__TimingSafetyCheck_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__TimingSafetyCheck_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
