// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/validate_calibration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `calibration_file`
// Member `test_images`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__ValidateCalibration_Request__init(autonomy_interfaces__srv__ValidateCalibration_Request * msg)
{
  if (!msg) {
    return false;
  }
  // calibration_file
  if (!rosidl_runtime_c__String__init(&msg->calibration_file)) {
    autonomy_interfaces__srv__ValidateCalibration_Request__fini(msg);
    return false;
  }
  // test_images
  if (!rosidl_runtime_c__String__Sequence__init(&msg->test_images, 0)) {
    autonomy_interfaces__srv__ValidateCalibration_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__ValidateCalibration_Request__fini(autonomy_interfaces__srv__ValidateCalibration_Request * msg)
{
  if (!msg) {
    return;
  }
  // calibration_file
  rosidl_runtime_c__String__fini(&msg->calibration_file);
  // test_images
  rosidl_runtime_c__String__Sequence__fini(&msg->test_images);
}

bool
autonomy_interfaces__srv__ValidateCalibration_Request__are_equal(const autonomy_interfaces__srv__ValidateCalibration_Request * lhs, const autonomy_interfaces__srv__ValidateCalibration_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // calibration_file
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->calibration_file), &(rhs->calibration_file)))
  {
    return false;
  }
  // test_images
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->test_images), &(rhs->test_images)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__ValidateCalibration_Request__copy(
  const autonomy_interfaces__srv__ValidateCalibration_Request * input,
  autonomy_interfaces__srv__ValidateCalibration_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // calibration_file
  if (!rosidl_runtime_c__String__copy(
      &(input->calibration_file), &(output->calibration_file)))
  {
    return false;
  }
  // test_images
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->test_images), &(output->test_images)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__ValidateCalibration_Request *
autonomy_interfaces__srv__ValidateCalibration_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Request * msg = (autonomy_interfaces__srv__ValidateCalibration_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__ValidateCalibration_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ValidateCalibration_Request));
  bool success = autonomy_interfaces__srv__ValidateCalibration_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ValidateCalibration_Request__destroy(autonomy_interfaces__srv__ValidateCalibration_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ValidateCalibration_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__init(autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ValidateCalibration_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ValidateCalibration_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ValidateCalibration_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ValidateCalibration_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__fini(autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * array)
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
      autonomy_interfaces__srv__ValidateCalibration_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__ValidateCalibration_Request__Sequence *
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * array = (autonomy_interfaces__srv__ValidateCalibration_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ValidateCalibration_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__destroy(autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__are_equal(const autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * lhs, const autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ValidateCalibration_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ValidateCalibration_Request__Sequence__copy(
  const autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * input,
  autonomy_interfaces__srv__ValidateCalibration_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ValidateCalibration_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ValidateCalibration_Request * data =
      (autonomy_interfaces__srv__ValidateCalibration_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ValidateCalibration_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ValidateCalibration_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ValidateCalibration_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `quality_assessment`
// Member `recommendations`
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__ValidateCalibration_Response__init(autonomy_interfaces__srv__ValidateCalibration_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // reprojection_error
  // quality_assessment
  if (!rosidl_runtime_c__String__init(&msg->quality_assessment)) {
    autonomy_interfaces__srv__ValidateCalibration_Response__fini(msg);
    return false;
  }
  // recommendations
  if (!rosidl_runtime_c__String__init(&msg->recommendations)) {
    autonomy_interfaces__srv__ValidateCalibration_Response__fini(msg);
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    autonomy_interfaces__srv__ValidateCalibration_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__ValidateCalibration_Response__fini(autonomy_interfaces__srv__ValidateCalibration_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // reprojection_error
  // quality_assessment
  rosidl_runtime_c__String__fini(&msg->quality_assessment);
  // recommendations
  rosidl_runtime_c__String__fini(&msg->recommendations);
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
autonomy_interfaces__srv__ValidateCalibration_Response__are_equal(const autonomy_interfaces__srv__ValidateCalibration_Response * lhs, const autonomy_interfaces__srv__ValidateCalibration_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // reprojection_error
  if (lhs->reprojection_error != rhs->reprojection_error) {
    return false;
  }
  // quality_assessment
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->quality_assessment), &(rhs->quality_assessment)))
  {
    return false;
  }
  // recommendations
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recommendations), &(rhs->recommendations)))
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
autonomy_interfaces__srv__ValidateCalibration_Response__copy(
  const autonomy_interfaces__srv__ValidateCalibration_Response * input,
  autonomy_interfaces__srv__ValidateCalibration_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // reprojection_error
  output->reprojection_error = input->reprojection_error;
  // quality_assessment
  if (!rosidl_runtime_c__String__copy(
      &(input->quality_assessment), &(output->quality_assessment)))
  {
    return false;
  }
  // recommendations
  if (!rosidl_runtime_c__String__copy(
      &(input->recommendations), &(output->recommendations)))
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

autonomy_interfaces__srv__ValidateCalibration_Response *
autonomy_interfaces__srv__ValidateCalibration_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Response * msg = (autonomy_interfaces__srv__ValidateCalibration_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__ValidateCalibration_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ValidateCalibration_Response));
  bool success = autonomy_interfaces__srv__ValidateCalibration_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ValidateCalibration_Response__destroy(autonomy_interfaces__srv__ValidateCalibration_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ValidateCalibration_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__init(autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ValidateCalibration_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ValidateCalibration_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ValidateCalibration_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ValidateCalibration_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__fini(autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * array)
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
      autonomy_interfaces__srv__ValidateCalibration_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__ValidateCalibration_Response__Sequence *
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * array = (autonomy_interfaces__srv__ValidateCalibration_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ValidateCalibration_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__destroy(autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__are_equal(const autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * lhs, const autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ValidateCalibration_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ValidateCalibration_Response__Sequence__copy(
  const autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * input,
  autonomy_interfaces__srv__ValidateCalibration_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ValidateCalibration_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ValidateCalibration_Response * data =
      (autonomy_interfaces__srv__ValidateCalibration_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ValidateCalibration_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ValidateCalibration_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ValidateCalibration_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
