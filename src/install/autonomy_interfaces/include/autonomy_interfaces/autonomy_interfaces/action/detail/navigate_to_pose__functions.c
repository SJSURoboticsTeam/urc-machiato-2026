// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:action/NavigateToPose.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/action/detail/navigate_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_Goal__init(autonomy_interfaces__action__NavigateToPose_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->target_pose)) {
    autonomy_interfaces__action__NavigateToPose_Goal__fini(msg);
    return false;
  }
  // tolerance
  // timeout
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_Goal__fini(autonomy_interfaces__action__NavigateToPose_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->target_pose);
  // tolerance
  // timeout
}

bool
autonomy_interfaces__action__NavigateToPose_Goal__are_equal(const autonomy_interfaces__action__NavigateToPose_Goal * lhs, const autonomy_interfaces__action__NavigateToPose_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->target_pose), &(rhs->target_pose)))
  {
    return false;
  }
  // tolerance
  if (lhs->tolerance != rhs->tolerance) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Goal__copy(
  const autonomy_interfaces__action__NavigateToPose_Goal * input,
  autonomy_interfaces__action__NavigateToPose_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->target_pose), &(output->target_pose)))
  {
    return false;
  }
  // tolerance
  output->tolerance = input->tolerance;
  // timeout
  output->timeout = input->timeout;
  return true;
}

autonomy_interfaces__action__NavigateToPose_Goal *
autonomy_interfaces__action__NavigateToPose_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Goal * msg = (autonomy_interfaces__action__NavigateToPose_Goal *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_Goal));
  bool success = autonomy_interfaces__action__NavigateToPose_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_Goal__destroy(autonomy_interfaces__action__NavigateToPose_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__init(autonomy_interfaces__action__NavigateToPose_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Goal * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_Goal *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_Goal__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__fini(autonomy_interfaces__action__NavigateToPose_Goal__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_Goal__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_Goal__Sequence *
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Goal__Sequence * array = (autonomy_interfaces__action__NavigateToPose_Goal__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_Goal__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Goal__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_Goal__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_Goal * data =
      (autonomy_interfaces__action__NavigateToPose_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `navigation_state`
#include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_Result__init(autonomy_interfaces__action__NavigateToPose_Result * msg)
{
  if (!msg) {
    return false;
  }
  // distance_to_goal
  // estimated_time_remaining
  // navigation_state
  if (!rosidl_runtime_c__String__init(&msg->navigation_state)) {
    autonomy_interfaces__action__NavigateToPose_Result__fini(msg);
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    autonomy_interfaces__action__NavigateToPose_Result__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_Result__fini(autonomy_interfaces__action__NavigateToPose_Result * msg)
{
  if (!msg) {
    return;
  }
  // distance_to_goal
  // estimated_time_remaining
  // navigation_state
  rosidl_runtime_c__String__fini(&msg->navigation_state);
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
}

bool
autonomy_interfaces__action__NavigateToPose_Result__are_equal(const autonomy_interfaces__action__NavigateToPose_Result * lhs, const autonomy_interfaces__action__NavigateToPose_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance_to_goal
  if (lhs->distance_to_goal != rhs->distance_to_goal) {
    return false;
  }
  // estimated_time_remaining
  if (lhs->estimated_time_remaining != rhs->estimated_time_remaining) {
    return false;
  }
  // navigation_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->navigation_state), &(rhs->navigation_state)))
  {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Result__copy(
  const autonomy_interfaces__action__NavigateToPose_Result * input,
  autonomy_interfaces__action__NavigateToPose_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // distance_to_goal
  output->distance_to_goal = input->distance_to_goal;
  // estimated_time_remaining
  output->estimated_time_remaining = input->estimated_time_remaining;
  // navigation_state
  if (!rosidl_runtime_c__String__copy(
      &(input->navigation_state), &(output->navigation_state)))
  {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_Result *
autonomy_interfaces__action__NavigateToPose_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Result * msg = (autonomy_interfaces__action__NavigateToPose_Result *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_Result));
  bool success = autonomy_interfaces__action__NavigateToPose_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_Result__destroy(autonomy_interfaces__action__NavigateToPose_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_Result__Sequence__init(autonomy_interfaces__action__NavigateToPose_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Result * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_Result *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_Result__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_Result__Sequence__fini(autonomy_interfaces__action__NavigateToPose_Result__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_Result__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_Result__Sequence *
autonomy_interfaces__action__NavigateToPose_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Result__Sequence * array = (autonomy_interfaces__action__NavigateToPose_Result__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_Result__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_Result__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_Result__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Result__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_Result__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_Result * data =
      (autonomy_interfaces__action__NavigateToPose_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Result__copy(
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
// Member `final_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_Feedback__init(autonomy_interfaces__action__NavigateToPose_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__action__NavigateToPose_Feedback__fini(msg);
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_pose)) {
    autonomy_interfaces__action__NavigateToPose_Feedback__fini(msg);
    return false;
  }
  // total_distance_traveled
  // total_time
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_Feedback__fini(autonomy_interfaces__action__NavigateToPose_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // final_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_pose);
  // total_distance_traveled
  // total_time
}

bool
autonomy_interfaces__action__NavigateToPose_Feedback__are_equal(const autonomy_interfaces__action__NavigateToPose_Feedback * lhs, const autonomy_interfaces__action__NavigateToPose_Feedback * rhs)
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
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_pose), &(rhs->final_pose)))
  {
    return false;
  }
  // total_distance_traveled
  if (lhs->total_distance_traveled != rhs->total_distance_traveled) {
    return false;
  }
  // total_time
  if (lhs->total_time != rhs->total_time) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Feedback__copy(
  const autonomy_interfaces__action__NavigateToPose_Feedback * input,
  autonomy_interfaces__action__NavigateToPose_Feedback * output)
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
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_pose), &(output->final_pose)))
  {
    return false;
  }
  // total_distance_traveled
  output->total_distance_traveled = input->total_distance_traveled;
  // total_time
  output->total_time = input->total_time;
  return true;
}

autonomy_interfaces__action__NavigateToPose_Feedback *
autonomy_interfaces__action__NavigateToPose_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Feedback * msg = (autonomy_interfaces__action__NavigateToPose_Feedback *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_Feedback));
  bool success = autonomy_interfaces__action__NavigateToPose_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_Feedback__destroy(autonomy_interfaces__action__NavigateToPose_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__init(autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Feedback * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_Feedback *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_Feedback__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__fini(autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_Feedback__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_Feedback__Sequence *
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * array = (autonomy_interfaces__action__NavigateToPose_Feedback__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_Feedback__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_Feedback * data =
      (autonomy_interfaces__action__NavigateToPose_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "autonomy_interfaces/action/detail/navigate_to_pose__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__init(autonomy_interfaces__action__NavigateToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!autonomy_interfaces__action__NavigateToPose_Goal__init(&msg->goal)) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(autonomy_interfaces__action__NavigateToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  autonomy_interfaces__action__NavigateToPose_Goal__fini(&msg->goal);
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__are_equal(const autonomy_interfaces__action__NavigateToPose_SendGoal_Request * lhs, const autonomy_interfaces__action__NavigateToPose_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!autonomy_interfaces__action__NavigateToPose_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__copy(
  const autonomy_interfaces__action__NavigateToPose_SendGoal_Request * input,
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!autonomy_interfaces__action__NavigateToPose_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_SendGoal_Request *
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request * msg = (autonomy_interfaces__action__NavigateToPose_SendGoal_Request *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Request));
  bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__destroy(autonomy_interfaces__action__NavigateToPose_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__init(autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_SendGoal_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__fini(autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence *
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * array = (autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_SendGoal_Request * data =
      (autonomy_interfaces__action__NavigateToPose_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__init(autonomy_interfaces__action__NavigateToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(autonomy_interfaces__action__NavigateToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__are_equal(const autonomy_interfaces__action__NavigateToPose_SendGoal_Response * lhs, const autonomy_interfaces__action__NavigateToPose_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__copy(
  const autonomy_interfaces__action__NavigateToPose_SendGoal_Response * input,
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_SendGoal_Response *
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response * msg = (autonomy_interfaces__action__NavigateToPose_SendGoal_Response *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Response));
  bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__destroy(autonomy_interfaces__action__NavigateToPose_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__init(autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_SendGoal_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__fini(autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence *
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * array = (autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_SendGoal_Response * data =
      (autonomy_interfaces__action__NavigateToPose_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__init(autonomy_interfaces__action__NavigateToPose_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(autonomy_interfaces__action__NavigateToPose_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__are_equal(const autonomy_interfaces__action__NavigateToPose_GetResult_Request * lhs, const autonomy_interfaces__action__NavigateToPose_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__copy(
  const autonomy_interfaces__action__NavigateToPose_GetResult_Request * input,
  autonomy_interfaces__action__NavigateToPose_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_GetResult_Request *
autonomy_interfaces__action__NavigateToPose_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Request * msg = (autonomy_interfaces__action__NavigateToPose_GetResult_Request *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Request));
  bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Request__destroy(autonomy_interfaces__action__NavigateToPose_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__init(autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_GetResult_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__fini(autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence *
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * array = (autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_GetResult_Request * data =
      (autonomy_interfaces__action__NavigateToPose_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "autonomy_interfaces/action/detail/navigate_to_pose__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__init(autonomy_interfaces__action__NavigateToPose_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!autonomy_interfaces__action__NavigateToPose_Result__init(&msg->result)) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(autonomy_interfaces__action__NavigateToPose_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  autonomy_interfaces__action__NavigateToPose_Result__fini(&msg->result);
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__are_equal(const autonomy_interfaces__action__NavigateToPose_GetResult_Response * lhs, const autonomy_interfaces__action__NavigateToPose_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!autonomy_interfaces__action__NavigateToPose_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__copy(
  const autonomy_interfaces__action__NavigateToPose_GetResult_Response * input,
  autonomy_interfaces__action__NavigateToPose_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!autonomy_interfaces__action__NavigateToPose_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_GetResult_Response *
autonomy_interfaces__action__NavigateToPose_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Response * msg = (autonomy_interfaces__action__NavigateToPose_GetResult_Response *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Response));
  bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Response__destroy(autonomy_interfaces__action__NavigateToPose_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__init(autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_GetResult_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__fini(autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence *
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * array = (autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_GetResult_Response * data =
      (autonomy_interfaces__action__NavigateToPose_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "autonomy_interfaces/action/detail/navigate_to_pose__functions.h"

bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__init(autonomy_interfaces__action__NavigateToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!autonomy_interfaces__action__NavigateToPose_Feedback__init(&msg->feedback)) {
    autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(autonomy_interfaces__action__NavigateToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  autonomy_interfaces__action__NavigateToPose_Feedback__fini(&msg->feedback);
}

bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__are_equal(const autonomy_interfaces__action__NavigateToPose_FeedbackMessage * lhs, const autonomy_interfaces__action__NavigateToPose_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!autonomy_interfaces__action__NavigateToPose_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__copy(
  const autonomy_interfaces__action__NavigateToPose_FeedbackMessage * input,
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!autonomy_interfaces__action__NavigateToPose_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__action__NavigateToPose_FeedbackMessage *
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage * msg = (autonomy_interfaces__action__NavigateToPose_FeedbackMessage *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__action__NavigateToPose_FeedbackMessage));
  bool success = autonomy_interfaces__action__NavigateToPose_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__destroy(autonomy_interfaces__action__NavigateToPose_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__init(autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage * data = NULL;

  if (size) {
    data = (autonomy_interfaces__action__NavigateToPose_FeedbackMessage *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__action__NavigateToPose_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__action__NavigateToPose_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(&data[i - 1]);
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
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__fini(autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * array)
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
      autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(&array->data[i]);
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

autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence *
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * array = (autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__destroy(autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__are_equal(const autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * lhs, const autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence__copy(
  const autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * input,
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__action__NavigateToPose_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__action__NavigateToPose_FeedbackMessage * data =
      (autonomy_interfaces__action__NavigateToPose_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__action__NavigateToPose_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__action__NavigateToPose_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__action__NavigateToPose_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
