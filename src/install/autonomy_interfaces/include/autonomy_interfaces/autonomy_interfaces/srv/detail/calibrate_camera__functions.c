// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/calibrate_camera__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `image_directory`
// Member `board_type`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__CalibrateCamera_Request__init(autonomy_interfaces__srv__CalibrateCamera_Request * msg)
{
  if (!msg) {
    return false;
  }
  // image_directory
  if (!rosidl_runtime_c__String__init(&msg->image_directory)) {
    autonomy_interfaces__srv__CalibrateCamera_Request__fini(msg);
    return false;
  }
  // board_type
  if (!rosidl_runtime_c__String__init(&msg->board_type)) {
    autonomy_interfaces__srv__CalibrateCamera_Request__fini(msg);
    return false;
  }
  // squares_x
  // squares_y
  // square_size
  // marker_size
  return true;
}

void
autonomy_interfaces__srv__CalibrateCamera_Request__fini(autonomy_interfaces__srv__CalibrateCamera_Request * msg)
{
  if (!msg) {
    return;
  }
  // image_directory
  rosidl_runtime_c__String__fini(&msg->image_directory);
  // board_type
  rosidl_runtime_c__String__fini(&msg->board_type);
  // squares_x
  // squares_y
  // square_size
  // marker_size
}

bool
autonomy_interfaces__srv__CalibrateCamera_Request__are_equal(const autonomy_interfaces__srv__CalibrateCamera_Request * lhs, const autonomy_interfaces__srv__CalibrateCamera_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image_directory
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->image_directory), &(rhs->image_directory)))
  {
    return false;
  }
  // board_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->board_type), &(rhs->board_type)))
  {
    return false;
  }
  // squares_x
  if (lhs->squares_x != rhs->squares_x) {
    return false;
  }
  // squares_y
  if (lhs->squares_y != rhs->squares_y) {
    return false;
  }
  // square_size
  if (lhs->square_size != rhs->square_size) {
    return false;
  }
  // marker_size
  if (lhs->marker_size != rhs->marker_size) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__CalibrateCamera_Request__copy(
  const autonomy_interfaces__srv__CalibrateCamera_Request * input,
  autonomy_interfaces__srv__CalibrateCamera_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // image_directory
  if (!rosidl_runtime_c__String__copy(
      &(input->image_directory), &(output->image_directory)))
  {
    return false;
  }
  // board_type
  if (!rosidl_runtime_c__String__copy(
      &(input->board_type), &(output->board_type)))
  {
    return false;
  }
  // squares_x
  output->squares_x = input->squares_x;
  // squares_y
  output->squares_y = input->squares_y;
  // square_size
  output->square_size = input->square_size;
  // marker_size
  output->marker_size = input->marker_size;
  return true;
}

autonomy_interfaces__srv__CalibrateCamera_Request *
autonomy_interfaces__srv__CalibrateCamera_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Request * msg = (autonomy_interfaces__srv__CalibrateCamera_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__CalibrateCamera_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__CalibrateCamera_Request));
  bool success = autonomy_interfaces__srv__CalibrateCamera_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__CalibrateCamera_Request__destroy(autonomy_interfaces__srv__CalibrateCamera_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__CalibrateCamera_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__init(autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__CalibrateCamera_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__CalibrateCamera_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__CalibrateCamera_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__CalibrateCamera_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__fini(autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * array)
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
      autonomy_interfaces__srv__CalibrateCamera_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__CalibrateCamera_Request__Sequence *
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * array = (autonomy_interfaces__srv__CalibrateCamera_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__CalibrateCamera_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__destroy(autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__are_equal(const autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * lhs, const autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__CalibrateCamera_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__CalibrateCamera_Request__Sequence__copy(
  const autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * input,
  autonomy_interfaces__srv__CalibrateCamera_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__CalibrateCamera_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__CalibrateCamera_Request * data =
      (autonomy_interfaces__srv__CalibrateCamera_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__CalibrateCamera_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__CalibrateCamera_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__CalibrateCamera_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result_file`
// Member `calibration_summary`
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__CalibrateCamera_Response__init(autonomy_interfaces__srv__CalibrateCamera_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result_file
  if (!rosidl_runtime_c__String__init(&msg->result_file)) {
    autonomy_interfaces__srv__CalibrateCamera_Response__fini(msg);
    return false;
  }
  // calibration_summary
  if (!rosidl_runtime_c__String__init(&msg->calibration_summary)) {
    autonomy_interfaces__srv__CalibrateCamera_Response__fini(msg);
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    autonomy_interfaces__srv__CalibrateCamera_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__CalibrateCamera_Response__fini(autonomy_interfaces__srv__CalibrateCamera_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result_file
  rosidl_runtime_c__String__fini(&msg->result_file);
  // calibration_summary
  rosidl_runtime_c__String__fini(&msg->calibration_summary);
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
autonomy_interfaces__srv__CalibrateCamera_Response__are_equal(const autonomy_interfaces__srv__CalibrateCamera_Response * lhs, const autonomy_interfaces__srv__CalibrateCamera_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // result_file
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->result_file), &(rhs->result_file)))
  {
    return false;
  }
  // calibration_summary
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->calibration_summary), &(rhs->calibration_summary)))
  {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__CalibrateCamera_Response__copy(
  const autonomy_interfaces__srv__CalibrateCamera_Response * input,
  autonomy_interfaces__srv__CalibrateCamera_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // result_file
  if (!rosidl_runtime_c__String__copy(
      &(input->result_file), &(output->result_file)))
  {
    return false;
  }
  // calibration_summary
  if (!rosidl_runtime_c__String__copy(
      &(input->calibration_summary), &(output->calibration_summary)))
  {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__CalibrateCamera_Response *
autonomy_interfaces__srv__CalibrateCamera_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Response * msg = (autonomy_interfaces__srv__CalibrateCamera_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__CalibrateCamera_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__CalibrateCamera_Response));
  bool success = autonomy_interfaces__srv__CalibrateCamera_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__CalibrateCamera_Response__destroy(autonomy_interfaces__srv__CalibrateCamera_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__CalibrateCamera_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__init(autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__CalibrateCamera_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__CalibrateCamera_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__CalibrateCamera_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__CalibrateCamera_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__fini(autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * array)
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
      autonomy_interfaces__srv__CalibrateCamera_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__CalibrateCamera_Response__Sequence *
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * array = (autonomy_interfaces__srv__CalibrateCamera_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__CalibrateCamera_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__destroy(autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__are_equal(const autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * lhs, const autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__CalibrateCamera_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__CalibrateCamera_Response__Sequence__copy(
  const autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * input,
  autonomy_interfaces__srv__CalibrateCamera_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__CalibrateCamera_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__CalibrateCamera_Response * data =
      (autonomy_interfaces__srv__CalibrateCamera_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__CalibrateCamera_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__CalibrateCamera_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__CalibrateCamera_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
