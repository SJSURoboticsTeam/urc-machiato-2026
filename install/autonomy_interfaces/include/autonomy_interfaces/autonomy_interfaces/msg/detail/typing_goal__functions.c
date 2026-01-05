// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/typing_goal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `target_text`
// Member `keyboard_layout`
#include "rosidl_runtime_c/string_functions.h"
// Member `keyboard_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `key_dimensions`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
autonomy_interfaces__msg__TypingGoal__init(autonomy_interfaces__msg__TypingGoal * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
    return false;
  }
  // target_text
  if (!rosidl_runtime_c__String__init(&msg->target_text)) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
    return false;
  }
  // keyboard_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->keyboard_pose)) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
    return false;
  }
  // accuracy_requirement
  // speed_requirement
  // timeout
  // keyboard_layout
  if (!rosidl_runtime_c__String__init(&msg->keyboard_layout)) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
    return false;
  }
  // key_spacing_m
  // key_dimensions
  if (!geometry_msgs__msg__Vector3__init(&msg->key_dimensions)) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
    return false;
  }
  // standoff_distance
  // contact_force
  return true;
}

void
autonomy_interfaces__msg__TypingGoal__fini(autonomy_interfaces__msg__TypingGoal * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // target_text
  rosidl_runtime_c__String__fini(&msg->target_text);
  // keyboard_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->keyboard_pose);
  // accuracy_requirement
  // speed_requirement
  // timeout
  // keyboard_layout
  rosidl_runtime_c__String__fini(&msg->keyboard_layout);
  // key_spacing_m
  // key_dimensions
  geometry_msgs__msg__Vector3__fini(&msg->key_dimensions);
  // standoff_distance
  // contact_force
}

bool
autonomy_interfaces__msg__TypingGoal__are_equal(const autonomy_interfaces__msg__TypingGoal * lhs, const autonomy_interfaces__msg__TypingGoal * rhs)
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
  // target_text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_text), &(rhs->target_text)))
  {
    return false;
  }
  // keyboard_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->keyboard_pose), &(rhs->keyboard_pose)))
  {
    return false;
  }
  // accuracy_requirement
  if (lhs->accuracy_requirement != rhs->accuracy_requirement) {
    return false;
  }
  // speed_requirement
  if (lhs->speed_requirement != rhs->speed_requirement) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // keyboard_layout
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->keyboard_layout), &(rhs->keyboard_layout)))
  {
    return false;
  }
  // key_spacing_m
  if (lhs->key_spacing_m != rhs->key_spacing_m) {
    return false;
  }
  // key_dimensions
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->key_dimensions), &(rhs->key_dimensions)))
  {
    return false;
  }
  // standoff_distance
  if (lhs->standoff_distance != rhs->standoff_distance) {
    return false;
  }
  // contact_force
  if (lhs->contact_force != rhs->contact_force) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__TypingGoal__copy(
  const autonomy_interfaces__msg__TypingGoal * input,
  autonomy_interfaces__msg__TypingGoal * output)
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
  // target_text
  if (!rosidl_runtime_c__String__copy(
      &(input->target_text), &(output->target_text)))
  {
    return false;
  }
  // keyboard_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->keyboard_pose), &(output->keyboard_pose)))
  {
    return false;
  }
  // accuracy_requirement
  output->accuracy_requirement = input->accuracy_requirement;
  // speed_requirement
  output->speed_requirement = input->speed_requirement;
  // timeout
  output->timeout = input->timeout;
  // keyboard_layout
  if (!rosidl_runtime_c__String__copy(
      &(input->keyboard_layout), &(output->keyboard_layout)))
  {
    return false;
  }
  // key_spacing_m
  output->key_spacing_m = input->key_spacing_m;
  // key_dimensions
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->key_dimensions), &(output->key_dimensions)))
  {
    return false;
  }
  // standoff_distance
  output->standoff_distance = input->standoff_distance;
  // contact_force
  output->contact_force = input->contact_force;
  return true;
}

autonomy_interfaces__msg__TypingGoal *
autonomy_interfaces__msg__TypingGoal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__TypingGoal * msg = (autonomy_interfaces__msg__TypingGoal *)allocator.allocate(sizeof(autonomy_interfaces__msg__TypingGoal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__TypingGoal));
  bool success = autonomy_interfaces__msg__TypingGoal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__TypingGoal__destroy(autonomy_interfaces__msg__TypingGoal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__TypingGoal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__TypingGoal__Sequence__init(autonomy_interfaces__msg__TypingGoal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__TypingGoal * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__TypingGoal *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__TypingGoal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__TypingGoal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__TypingGoal__fini(&data[i - 1]);
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
autonomy_interfaces__msg__TypingGoal__Sequence__fini(autonomy_interfaces__msg__TypingGoal__Sequence * array)
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
      autonomy_interfaces__msg__TypingGoal__fini(&array->data[i]);
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

autonomy_interfaces__msg__TypingGoal__Sequence *
autonomy_interfaces__msg__TypingGoal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__TypingGoal__Sequence * array = (autonomy_interfaces__msg__TypingGoal__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__TypingGoal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__TypingGoal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__TypingGoal__Sequence__destroy(autonomy_interfaces__msg__TypingGoal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__TypingGoal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__TypingGoal__Sequence__are_equal(const autonomy_interfaces__msg__TypingGoal__Sequence * lhs, const autonomy_interfaces__msg__TypingGoal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__TypingGoal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__TypingGoal__Sequence__copy(
  const autonomy_interfaces__msg__TypingGoal__Sequence * input,
  autonomy_interfaces__msg__TypingGoal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__TypingGoal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__TypingGoal * data =
      (autonomy_interfaces__msg__TypingGoal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__TypingGoal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__TypingGoal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__TypingGoal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
