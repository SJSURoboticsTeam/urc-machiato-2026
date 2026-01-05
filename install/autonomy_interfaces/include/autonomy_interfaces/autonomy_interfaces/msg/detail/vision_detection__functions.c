// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/VisionDetection.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/vision_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `class_name`
// Member `detector_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `size`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__msg__VisionDetection__init(autonomy_interfaces__msg__VisionDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // class_id
  // confidence
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // size
  if (!geometry_msgs__msg__Vector3__init(&msg->size)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__float__Sequence__init(&msg->keypoints, 0)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // detector_type
  if (!rosidl_runtime_c__String__init(&msg->detector_type)) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
    return false;
  }
  // track_id
  // age
  return true;
}

void
autonomy_interfaces__msg__VisionDetection__fini(autonomy_interfaces__msg__VisionDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // class_id
  // confidence
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // size
  geometry_msgs__msg__Vector3__fini(&msg->size);
  // keypoints
  rosidl_runtime_c__float__Sequence__fini(&msg->keypoints);
  // detector_type
  rosidl_runtime_c__String__fini(&msg->detector_type);
  // track_id
  // age
}

bool
autonomy_interfaces__msg__VisionDetection__are_equal(const autonomy_interfaces__msg__VisionDetection * lhs, const autonomy_interfaces__msg__VisionDetection * rhs)
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
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // size
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->size), &(rhs->size)))
  {
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->keypoints), &(rhs->keypoints)))
  {
    return false;
  }
  // detector_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detector_type), &(rhs->detector_type)))
  {
    return false;
  }
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // age
  if (lhs->age != rhs->age) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__VisionDetection__copy(
  const autonomy_interfaces__msg__VisionDetection * input,
  autonomy_interfaces__msg__VisionDetection * output)
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
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // class_id
  output->class_id = input->class_id;
  // confidence
  output->confidence = input->confidence;
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // size
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->size), &(output->size)))
  {
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->keypoints), &(output->keypoints)))
  {
    return false;
  }
  // detector_type
  if (!rosidl_runtime_c__String__copy(
      &(input->detector_type), &(output->detector_type)))
  {
    return false;
  }
  // track_id
  output->track_id = input->track_id;
  // age
  output->age = input->age;
  return true;
}

autonomy_interfaces__msg__VisionDetection *
autonomy_interfaces__msg__VisionDetection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__VisionDetection * msg = (autonomy_interfaces__msg__VisionDetection *)allocator.allocate(sizeof(autonomy_interfaces__msg__VisionDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__VisionDetection));
  bool success = autonomy_interfaces__msg__VisionDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__VisionDetection__destroy(autonomy_interfaces__msg__VisionDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__VisionDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__VisionDetection__Sequence__init(autonomy_interfaces__msg__VisionDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__VisionDetection * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__VisionDetection *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__VisionDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__VisionDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__VisionDetection__fini(&data[i - 1]);
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
autonomy_interfaces__msg__VisionDetection__Sequence__fini(autonomy_interfaces__msg__VisionDetection__Sequence * array)
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
      autonomy_interfaces__msg__VisionDetection__fini(&array->data[i]);
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

autonomy_interfaces__msg__VisionDetection__Sequence *
autonomy_interfaces__msg__VisionDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__VisionDetection__Sequence * array = (autonomy_interfaces__msg__VisionDetection__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__VisionDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__VisionDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__VisionDetection__Sequence__destroy(autonomy_interfaces__msg__VisionDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__VisionDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__VisionDetection__Sequence__are_equal(const autonomy_interfaces__msg__VisionDetection__Sequence * lhs, const autonomy_interfaces__msg__VisionDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__VisionDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__VisionDetection__Sequence__copy(
  const autonomy_interfaces__msg__VisionDetection__Sequence * input,
  autonomy_interfaces__msg__VisionDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__VisionDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__VisionDetection * data =
      (autonomy_interfaces__msg__VisionDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__VisionDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__VisionDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__VisionDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
