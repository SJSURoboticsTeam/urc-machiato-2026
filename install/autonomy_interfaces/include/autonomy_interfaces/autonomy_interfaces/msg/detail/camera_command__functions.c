// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/camera_command__functions.h"

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
// Member `scan_pattern`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__msg__CameraCommand__init(autonomy_interfaces__msg__CameraCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__CameraCommand__fini(msg);
    return false;
  }
  // command_type
  // pan_angle
  // tilt_angle
  // pan_speed
  // tilt_speed
  // zoom_level
  // autofocus
  // target_position
  if (!geometry_msgs__msg__Point__init(&msg->target_position)) {
    autonomy_interfaces__msg__CameraCommand__fini(msg);
    return false;
  }
  // tracking_timeout
  // scan_pattern
  if (!rosidl_runtime_c__String__init(&msg->scan_pattern)) {
    autonomy_interfaces__msg__CameraCommand__fini(msg);
    return false;
  }
  // scan_speed
  // scan_range
  // max_pan_speed
  // max_tilt_speed
  // priority
  // timeout
  return true;
}

void
autonomy_interfaces__msg__CameraCommand__fini(autonomy_interfaces__msg__CameraCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // command_type
  // pan_angle
  // tilt_angle
  // pan_speed
  // tilt_speed
  // zoom_level
  // autofocus
  // target_position
  geometry_msgs__msg__Point__fini(&msg->target_position);
  // tracking_timeout
  // scan_pattern
  rosidl_runtime_c__String__fini(&msg->scan_pattern);
  // scan_speed
  // scan_range
  // max_pan_speed
  // max_tilt_speed
  // priority
  // timeout
}

bool
autonomy_interfaces__msg__CameraCommand__are_equal(const autonomy_interfaces__msg__CameraCommand * lhs, const autonomy_interfaces__msg__CameraCommand * rhs)
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
  // command_type
  if (lhs->command_type != rhs->command_type) {
    return false;
  }
  // pan_angle
  if (lhs->pan_angle != rhs->pan_angle) {
    return false;
  }
  // tilt_angle
  if (lhs->tilt_angle != rhs->tilt_angle) {
    return false;
  }
  // pan_speed
  if (lhs->pan_speed != rhs->pan_speed) {
    return false;
  }
  // tilt_speed
  if (lhs->tilt_speed != rhs->tilt_speed) {
    return false;
  }
  // zoom_level
  if (lhs->zoom_level != rhs->zoom_level) {
    return false;
  }
  // autofocus
  if (lhs->autofocus != rhs->autofocus) {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_position), &(rhs->target_position)))
  {
    return false;
  }
  // tracking_timeout
  if (lhs->tracking_timeout != rhs->tracking_timeout) {
    return false;
  }
  // scan_pattern
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->scan_pattern), &(rhs->scan_pattern)))
  {
    return false;
  }
  // scan_speed
  if (lhs->scan_speed != rhs->scan_speed) {
    return false;
  }
  // scan_range
  if (lhs->scan_range != rhs->scan_range) {
    return false;
  }
  // max_pan_speed
  if (lhs->max_pan_speed != rhs->max_pan_speed) {
    return false;
  }
  // max_tilt_speed
  if (lhs->max_tilt_speed != rhs->max_tilt_speed) {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__CameraCommand__copy(
  const autonomy_interfaces__msg__CameraCommand * input,
  autonomy_interfaces__msg__CameraCommand * output)
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
  // command_type
  output->command_type = input->command_type;
  // pan_angle
  output->pan_angle = input->pan_angle;
  // tilt_angle
  output->tilt_angle = input->tilt_angle;
  // pan_speed
  output->pan_speed = input->pan_speed;
  // tilt_speed
  output->tilt_speed = input->tilt_speed;
  // zoom_level
  output->zoom_level = input->zoom_level;
  // autofocus
  output->autofocus = input->autofocus;
  // target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_position), &(output->target_position)))
  {
    return false;
  }
  // tracking_timeout
  output->tracking_timeout = input->tracking_timeout;
  // scan_pattern
  if (!rosidl_runtime_c__String__copy(
      &(input->scan_pattern), &(output->scan_pattern)))
  {
    return false;
  }
  // scan_speed
  output->scan_speed = input->scan_speed;
  // scan_range
  output->scan_range = input->scan_range;
  // max_pan_speed
  output->max_pan_speed = input->max_pan_speed;
  // max_tilt_speed
  output->max_tilt_speed = input->max_tilt_speed;
  // priority
  output->priority = input->priority;
  // timeout
  output->timeout = input->timeout;
  return true;
}

autonomy_interfaces__msg__CameraCommand *
autonomy_interfaces__msg__CameraCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__CameraCommand * msg = (autonomy_interfaces__msg__CameraCommand *)allocator.allocate(sizeof(autonomy_interfaces__msg__CameraCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__CameraCommand));
  bool success = autonomy_interfaces__msg__CameraCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__CameraCommand__destroy(autonomy_interfaces__msg__CameraCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__CameraCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__CameraCommand__Sequence__init(autonomy_interfaces__msg__CameraCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__CameraCommand * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__CameraCommand *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__CameraCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__CameraCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__CameraCommand__fini(&data[i - 1]);
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
autonomy_interfaces__msg__CameraCommand__Sequence__fini(autonomy_interfaces__msg__CameraCommand__Sequence * array)
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
      autonomy_interfaces__msg__CameraCommand__fini(&array->data[i]);
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

autonomy_interfaces__msg__CameraCommand__Sequence *
autonomy_interfaces__msg__CameraCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__CameraCommand__Sequence * array = (autonomy_interfaces__msg__CameraCommand__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__CameraCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__CameraCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__CameraCommand__Sequence__destroy(autonomy_interfaces__msg__CameraCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__CameraCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__CameraCommand__Sequence__are_equal(const autonomy_interfaces__msg__CameraCommand__Sequence * lhs, const autonomy_interfaces__msg__CameraCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__CameraCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__CameraCommand__Sequence__copy(
  const autonomy_interfaces__msg__CameraCommand__Sequence * input,
  autonomy_interfaces__msg__CameraCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__CameraCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__CameraCommand * data =
      (autonomy_interfaces__msg__CameraCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__CameraCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__CameraCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__CameraCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
