// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/SlamStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/slam_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__functions.h"
// Member `local_map`
#include "nav_msgs/msg/detail/occupancy_grid__functions.h"

bool
autonomy_interfaces__msg__SlamStatus__init(autonomy_interfaces__msg__SlamStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__SlamStatus__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    autonomy_interfaces__msg__SlamStatus__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovarianceStamped__init(&msg->pose)) {
    autonomy_interfaces__msg__SlamStatus__fini(msg);
    return false;
  }
  // local_map
  if (!nav_msgs__msg__OccupancyGrid__init(&msg->local_map)) {
    autonomy_interfaces__msg__SlamStatus__fini(msg);
    return false;
  }
  // map_width
  // map_height
  // map_resolution
  // loop_closure_confidence
  // keyframes_tracked
  // landmarks_tracked
  // tracking_quality
  // loop_closure_detected
  // drift_estimate
  return true;
}

void
autonomy_interfaces__msg__SlamStatus__fini(autonomy_interfaces__msg__SlamStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // pose
  geometry_msgs__msg__PoseWithCovarianceStamped__fini(&msg->pose);
  // local_map
  nav_msgs__msg__OccupancyGrid__fini(&msg->local_map);
  // map_width
  // map_height
  // map_resolution
  // loop_closure_confidence
  // keyframes_tracked
  // landmarks_tracked
  // tracking_quality
  // loop_closure_detected
  // drift_estimate
}

bool
autonomy_interfaces__msg__SlamStatus__are_equal(const autonomy_interfaces__msg__SlamStatus * lhs, const autonomy_interfaces__msg__SlamStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovarianceStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // local_map
  if (!nav_msgs__msg__OccupancyGrid__are_equal(
      &(lhs->local_map), &(rhs->local_map)))
  {
    return false;
  }
  // map_width
  if (lhs->map_width != rhs->map_width) {
    return false;
  }
  // map_height
  if (lhs->map_height != rhs->map_height) {
    return false;
  }
  // map_resolution
  if (lhs->map_resolution != rhs->map_resolution) {
    return false;
  }
  // loop_closure_confidence
  if (lhs->loop_closure_confidence != rhs->loop_closure_confidence) {
    return false;
  }
  // keyframes_tracked
  if (lhs->keyframes_tracked != rhs->keyframes_tracked) {
    return false;
  }
  // landmarks_tracked
  if (lhs->landmarks_tracked != rhs->landmarks_tracked) {
    return false;
  }
  // tracking_quality
  if (lhs->tracking_quality != rhs->tracking_quality) {
    return false;
  }
  // loop_closure_detected
  if (lhs->loop_closure_detected != rhs->loop_closure_detected) {
    return false;
  }
  // drift_estimate
  if (lhs->drift_estimate != rhs->drift_estimate) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__SlamStatus__copy(
  const autonomy_interfaces__msg__SlamStatus * input,
  autonomy_interfaces__msg__SlamStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovarianceStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // local_map
  if (!nav_msgs__msg__OccupancyGrid__copy(
      &(input->local_map), &(output->local_map)))
  {
    return false;
  }
  // map_width
  output->map_width = input->map_width;
  // map_height
  output->map_height = input->map_height;
  // map_resolution
  output->map_resolution = input->map_resolution;
  // loop_closure_confidence
  output->loop_closure_confidence = input->loop_closure_confidence;
  // keyframes_tracked
  output->keyframes_tracked = input->keyframes_tracked;
  // landmarks_tracked
  output->landmarks_tracked = input->landmarks_tracked;
  // tracking_quality
  output->tracking_quality = input->tracking_quality;
  // loop_closure_detected
  output->loop_closure_detected = input->loop_closure_detected;
  // drift_estimate
  output->drift_estimate = input->drift_estimate;
  return true;
}

autonomy_interfaces__msg__SlamStatus *
autonomy_interfaces__msg__SlamStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SlamStatus * msg = (autonomy_interfaces__msg__SlamStatus *)allocator.allocate(sizeof(autonomy_interfaces__msg__SlamStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__SlamStatus));
  bool success = autonomy_interfaces__msg__SlamStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__SlamStatus__destroy(autonomy_interfaces__msg__SlamStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__SlamStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__SlamStatus__Sequence__init(autonomy_interfaces__msg__SlamStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SlamStatus * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__SlamStatus *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__SlamStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__SlamStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__SlamStatus__fini(&data[i - 1]);
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
autonomy_interfaces__msg__SlamStatus__Sequence__fini(autonomy_interfaces__msg__SlamStatus__Sequence * array)
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
      autonomy_interfaces__msg__SlamStatus__fini(&array->data[i]);
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

autonomy_interfaces__msg__SlamStatus__Sequence *
autonomy_interfaces__msg__SlamStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__SlamStatus__Sequence * array = (autonomy_interfaces__msg__SlamStatus__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__SlamStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__SlamStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__SlamStatus__Sequence__destroy(autonomy_interfaces__msg__SlamStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__SlamStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__SlamStatus__Sequence__are_equal(const autonomy_interfaces__msg__SlamStatus__Sequence * lhs, const autonomy_interfaces__msg__SlamStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__SlamStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__SlamStatus__Sequence__copy(
  const autonomy_interfaces__msg__SlamStatus__Sequence * input,
  autonomy_interfaces__msg__SlamStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__SlamStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__SlamStatus * data =
      (autonomy_interfaces__msg__SlamStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__SlamStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__SlamStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__SlamStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
