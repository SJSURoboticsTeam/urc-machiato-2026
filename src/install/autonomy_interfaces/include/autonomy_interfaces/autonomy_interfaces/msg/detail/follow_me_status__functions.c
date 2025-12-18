// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/FollowMeStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/follow_me_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `target_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `operator_id`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__FollowMeStatus__init(autonomy_interfaces__msg__FollowMeStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__FollowMeStatus__fini(msg);
    return false;
  }
  // is_following
  // target_tag_id
  // target_distance
  // target_angle
  // safety_distance
  // safety_violation
  // current_speed
  // target_position
  if (!geometry_msgs__msg__Point__init(&msg->target_position)) {
    autonomy_interfaces__msg__FollowMeStatus__fini(msg);
    return false;
  }
  // target_visible
  // last_detection_time
  // max_speed
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__msg__FollowMeStatus__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__FollowMeStatus__fini(autonomy_interfaces__msg__FollowMeStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_following
  // target_tag_id
  // target_distance
  // target_angle
  // safety_distance
  // safety_violation
  // current_speed
  // target_position
  geometry_msgs__msg__Point__fini(&msg->target_position);
  // target_visible
  // last_detection_time
  // max_speed
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
}

bool
autonomy_interfaces__msg__FollowMeStatus__are_equal(const autonomy_interfaces__msg__FollowMeStatus * lhs, const autonomy_interfaces__msg__FollowMeStatus * rhs)
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
  // is_following
  if (lhs->is_following != rhs->is_following) {
    return false;
  }
  // target_tag_id
  if (lhs->target_tag_id != rhs->target_tag_id) {
    return false;
  }
  // target_distance
  if (lhs->target_distance != rhs->target_distance) {
    return false;
  }
  // target_angle
  if (lhs->target_angle != rhs->target_angle) {
    return false;
  }
  // safety_distance
  if (lhs->safety_distance != rhs->safety_distance) {
    return false;
  }
  // safety_violation
  if (lhs->safety_violation != rhs->safety_violation) {
    return false;
  }
  // current_speed
  if (lhs->current_speed != rhs->current_speed) {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_position), &(rhs->target_position)))
  {
    return false;
  }
  // target_visible
  if (lhs->target_visible != rhs->target_visible) {
    return false;
  }
  // last_detection_time
  if (lhs->last_detection_time != rhs->last_detection_time) {
    return false;
  }
  // max_speed
  if (lhs->max_speed != rhs->max_speed) {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__FollowMeStatus__copy(
  const autonomy_interfaces__msg__FollowMeStatus * input,
  autonomy_interfaces__msg__FollowMeStatus * output)
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
  // is_following
  output->is_following = input->is_following;
  // target_tag_id
  output->target_tag_id = input->target_tag_id;
  // target_distance
  output->target_distance = input->target_distance;
  // target_angle
  output->target_angle = input->target_angle;
  // safety_distance
  output->safety_distance = input->safety_distance;
  // safety_violation
  output->safety_violation = input->safety_violation;
  // current_speed
  output->current_speed = input->current_speed;
  // target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_position), &(output->target_position)))
  {
    return false;
  }
  // target_visible
  output->target_visible = input->target_visible;
  // last_detection_time
  output->last_detection_time = input->last_detection_time;
  // max_speed
  output->max_speed = input->max_speed;
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__FollowMeStatus *
autonomy_interfaces__msg__FollowMeStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__FollowMeStatus * msg = (autonomy_interfaces__msg__FollowMeStatus *)allocator.allocate(sizeof(autonomy_interfaces__msg__FollowMeStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__FollowMeStatus));
  bool success = autonomy_interfaces__msg__FollowMeStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__FollowMeStatus__destroy(autonomy_interfaces__msg__FollowMeStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__FollowMeStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__FollowMeStatus__Sequence__init(autonomy_interfaces__msg__FollowMeStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__FollowMeStatus * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__FollowMeStatus *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__FollowMeStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__FollowMeStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__FollowMeStatus__fini(&data[i - 1]);
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
autonomy_interfaces__msg__FollowMeStatus__Sequence__fini(autonomy_interfaces__msg__FollowMeStatus__Sequence * array)
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
      autonomy_interfaces__msg__FollowMeStatus__fini(&array->data[i]);
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

autonomy_interfaces__msg__FollowMeStatus__Sequence *
autonomy_interfaces__msg__FollowMeStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__FollowMeStatus__Sequence * array = (autonomy_interfaces__msg__FollowMeStatus__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__FollowMeStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__FollowMeStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__FollowMeStatus__Sequence__destroy(autonomy_interfaces__msg__FollowMeStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__FollowMeStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__FollowMeStatus__Sequence__are_equal(const autonomy_interfaces__msg__FollowMeStatus__Sequence * lhs, const autonomy_interfaces__msg__FollowMeStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__FollowMeStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__FollowMeStatus__Sequence__copy(
  const autonomy_interfaces__msg__FollowMeStatus__Sequence * input,
  autonomy_interfaces__msg__FollowMeStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__FollowMeStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__FollowMeStatus * data =
      (autonomy_interfaces__msg__FollowMeStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__FollowMeStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__FollowMeStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__FollowMeStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
