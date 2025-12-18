// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/NavigationStatus.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/navigation_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
// Member `status_message`
#include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
// Member `goal_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
autonomy_interfaces__msg__NavigationStatus__init(autonomy_interfaces__msg__NavigationStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
    return false;
  }
  // mission_progress
  // current_waypoint
  // total_waypoints
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->goal_pose)) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
    return false;
  }
  // distance_to_goal
  // speed
  // heading_error
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__msg__NavigationStatus__fini(autonomy_interfaces__msg__NavigationStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // mission_progress
  // current_waypoint
  // total_waypoints
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // goal_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->goal_pose);
  // distance_to_goal
  // speed
  // heading_error
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
}

bool
autonomy_interfaces__msg__NavigationStatus__are_equal(const autonomy_interfaces__msg__NavigationStatus * lhs, const autonomy_interfaces__msg__NavigationStatus * rhs)
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
  // mission_progress
  if (lhs->mission_progress != rhs->mission_progress) {
    return false;
  }
  // current_waypoint
  if (lhs->current_waypoint != rhs->current_waypoint) {
    return false;
  }
  // total_waypoints
  if (lhs->total_waypoints != rhs->total_waypoints) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->goal_pose), &(rhs->goal_pose)))
  {
    return false;
  }
  // distance_to_goal
  if (lhs->distance_to_goal != rhs->distance_to_goal) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // heading_error
  if (lhs->heading_error != rhs->heading_error) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__NavigationStatus__copy(
  const autonomy_interfaces__msg__NavigationStatus * input,
  autonomy_interfaces__msg__NavigationStatus * output)
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
  // mission_progress
  output->mission_progress = input->mission_progress;
  // current_waypoint
  output->current_waypoint = input->current_waypoint;
  // total_waypoints
  output->total_waypoints = input->total_waypoints;
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->goal_pose), &(output->goal_pose)))
  {
    return false;
  }
  // distance_to_goal
  output->distance_to_goal = input->distance_to_goal;
  // speed
  output->speed = input->speed;
  // heading_error
  output->heading_error = input->heading_error;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__msg__NavigationStatus *
autonomy_interfaces__msg__NavigationStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__NavigationStatus * msg = (autonomy_interfaces__msg__NavigationStatus *)allocator.allocate(sizeof(autonomy_interfaces__msg__NavigationStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__NavigationStatus));
  bool success = autonomy_interfaces__msg__NavigationStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__NavigationStatus__destroy(autonomy_interfaces__msg__NavigationStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__NavigationStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__NavigationStatus__Sequence__init(autonomy_interfaces__msg__NavigationStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__NavigationStatus * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__NavigationStatus *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__NavigationStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__NavigationStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__NavigationStatus__fini(&data[i - 1]);
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
autonomy_interfaces__msg__NavigationStatus__Sequence__fini(autonomy_interfaces__msg__NavigationStatus__Sequence * array)
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
      autonomy_interfaces__msg__NavigationStatus__fini(&array->data[i]);
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

autonomy_interfaces__msg__NavigationStatus__Sequence *
autonomy_interfaces__msg__NavigationStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__NavigationStatus__Sequence * array = (autonomy_interfaces__msg__NavigationStatus__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__NavigationStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__NavigationStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__NavigationStatus__Sequence__destroy(autonomy_interfaces__msg__NavigationStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__NavigationStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__NavigationStatus__Sequence__are_equal(const autonomy_interfaces__msg__NavigationStatus__Sequence * lhs, const autonomy_interfaces__msg__NavigationStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__NavigationStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__NavigationStatus__Sequence__copy(
  const autonomy_interfaces__msg__NavigationStatus__Sequence * input,
  autonomy_interfaces__msg__NavigationStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__NavigationStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__NavigationStatus * data =
      (autonomy_interfaces__msg__NavigationStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__NavigationStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__NavigationStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__NavigationStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
