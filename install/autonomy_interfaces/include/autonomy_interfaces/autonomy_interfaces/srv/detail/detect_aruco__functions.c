// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/DetectAruco.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `target_tag_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `mission_type`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__DetectAruco_Request__init(autonomy_interfaces__srv__DetectAruco_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->target_tag_ids, 0)) {
    autonomy_interfaces__srv__DetectAruco_Request__fini(msg);
    return false;
  }
  // detection_timeout
  // require_distance_estimate
  // max_detection_distance
  // calculate_alignment
  // target_depth
  // mission_type
  if (!rosidl_runtime_c__String__init(&msg->mission_type)) {
    autonomy_interfaces__srv__DetectAruco_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__DetectAruco_Request__fini(autonomy_interfaces__srv__DetectAruco_Request * msg)
{
  if (!msg) {
    return;
  }
  // target_tag_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->target_tag_ids);
  // detection_timeout
  // require_distance_estimate
  // max_detection_distance
  // calculate_alignment
  // target_depth
  // mission_type
  rosidl_runtime_c__String__fini(&msg->mission_type);
}

bool
autonomy_interfaces__srv__DetectAruco_Request__are_equal(const autonomy_interfaces__srv__DetectAruco_Request * lhs, const autonomy_interfaces__srv__DetectAruco_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->target_tag_ids), &(rhs->target_tag_ids)))
  {
    return false;
  }
  // detection_timeout
  if (lhs->detection_timeout != rhs->detection_timeout) {
    return false;
  }
  // require_distance_estimate
  if (lhs->require_distance_estimate != rhs->require_distance_estimate) {
    return false;
  }
  // max_detection_distance
  if (lhs->max_detection_distance != rhs->max_detection_distance) {
    return false;
  }
  // calculate_alignment
  if (lhs->calculate_alignment != rhs->calculate_alignment) {
    return false;
  }
  // target_depth
  if (lhs->target_depth != rhs->target_depth) {
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_type), &(rhs->mission_type)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Request__copy(
  const autonomy_interfaces__srv__DetectAruco_Request * input,
  autonomy_interfaces__srv__DetectAruco_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->target_tag_ids), &(output->target_tag_ids)))
  {
    return false;
  }
  // detection_timeout
  output->detection_timeout = input->detection_timeout;
  // require_distance_estimate
  output->require_distance_estimate = input->require_distance_estimate;
  // max_detection_distance
  output->max_detection_distance = input->max_detection_distance;
  // calculate_alignment
  output->calculate_alignment = input->calculate_alignment;
  // target_depth
  output->target_depth = input->target_depth;
  // mission_type
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_type), &(output->mission_type)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__DetectAruco_Request *
autonomy_interfaces__srv__DetectAruco_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Request * msg = (autonomy_interfaces__srv__DetectAruco_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectAruco_Request));
  bool success = autonomy_interfaces__srv__DetectAruco_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectAruco_Request__destroy(autonomy_interfaces__srv__DetectAruco_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectAruco_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectAruco_Request__Sequence__init(autonomy_interfaces__srv__DetectAruco_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectAruco_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectAruco_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectAruco_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectAruco_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectAruco_Request__Sequence__fini(autonomy_interfaces__srv__DetectAruco_Request__Sequence * array)
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
      autonomy_interfaces__srv__DetectAruco_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectAruco_Request__Sequence *
autonomy_interfaces__srv__DetectAruco_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Request__Sequence * array = (autonomy_interfaces__srv__DetectAruco_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectAruco_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectAruco_Request__Sequence__destroy(autonomy_interfaces__srv__DetectAruco_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectAruco_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectAruco_Request__Sequence__are_equal(const autonomy_interfaces__srv__DetectAruco_Request__Sequence * lhs, const autonomy_interfaces__srv__DetectAruco_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Request__Sequence__copy(
  const autonomy_interfaces__srv__DetectAruco_Request__Sequence * input,
  autonomy_interfaces__srv__DetectAruco_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectAruco_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectAruco_Request * data =
      (autonomy_interfaces__srv__DetectAruco_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectAruco_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectAruco_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `alignment_warnings`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `detected_tag_ids`
// Member `tag_distances`
// Member `tag_angles`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tag_positions`
// Member `alignment_center`
// Member `arm_target_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `detection_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `alignment_orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
autonomy_interfaces__srv__DetectAruco_Response__init(autonomy_interfaces__srv__DetectAruco_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // detected_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->detected_tag_ids, 0)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // tag_positions
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->tag_positions, 0)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // tag_distances
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tag_distances, 0)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // tag_angles
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tag_angles, 0)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // detection_time
  if (!builtin_interfaces__msg__Time__init(&msg->detection_time)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // alignment_available
  // alignment_center
  if (!geometry_msgs__msg__Point__init(&msg->alignment_center)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // alignment_orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->alignment_orientation)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // arm_target_position
  if (!geometry_msgs__msg__Point__init(&msg->arm_target_position)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  // alignment_quality
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__init(&msg->alignment_warnings, 0)) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__DetectAruco_Response__fini(autonomy_interfaces__srv__DetectAruco_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // detected_tag_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->detected_tag_ids);
  // tag_positions
  geometry_msgs__msg__Point__Sequence__fini(&msg->tag_positions);
  // tag_distances
  rosidl_runtime_c__float__Sequence__fini(&msg->tag_distances);
  // tag_angles
  rosidl_runtime_c__float__Sequence__fini(&msg->tag_angles);
  // detection_time
  builtin_interfaces__msg__Time__fini(&msg->detection_time);
  // alignment_available
  // alignment_center
  geometry_msgs__msg__Point__fini(&msg->alignment_center);
  // alignment_orientation
  geometry_msgs__msg__Quaternion__fini(&msg->alignment_orientation);
  // arm_target_position
  geometry_msgs__msg__Point__fini(&msg->arm_target_position);
  // alignment_quality
  // alignment_warnings
  rosidl_runtime_c__String__Sequence__fini(&msg->alignment_warnings);
}

bool
autonomy_interfaces__srv__DetectAruco_Response__are_equal(const autonomy_interfaces__srv__DetectAruco_Response * lhs, const autonomy_interfaces__srv__DetectAruco_Response * rhs)
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
  // detected_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->detected_tag_ids), &(rhs->detected_tag_ids)))
  {
    return false;
  }
  // tag_positions
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->tag_positions), &(rhs->tag_positions)))
  {
    return false;
  }
  // tag_distances
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->tag_distances), &(rhs->tag_distances)))
  {
    return false;
  }
  // tag_angles
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->tag_angles), &(rhs->tag_angles)))
  {
    return false;
  }
  // detection_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->detection_time), &(rhs->detection_time)))
  {
    return false;
  }
  // alignment_available
  if (lhs->alignment_available != rhs->alignment_available) {
    return false;
  }
  // alignment_center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->alignment_center), &(rhs->alignment_center)))
  {
    return false;
  }
  // alignment_orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->alignment_orientation), &(rhs->alignment_orientation)))
  {
    return false;
  }
  // arm_target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->arm_target_position), &(rhs->arm_target_position)))
  {
    return false;
  }
  // alignment_quality
  if (lhs->alignment_quality != rhs->alignment_quality) {
    return false;
  }
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->alignment_warnings), &(rhs->alignment_warnings)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Response__copy(
  const autonomy_interfaces__srv__DetectAruco_Response * input,
  autonomy_interfaces__srv__DetectAruco_Response * output)
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
  // detected_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->detected_tag_ids), &(output->detected_tag_ids)))
  {
    return false;
  }
  // tag_positions
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->tag_positions), &(output->tag_positions)))
  {
    return false;
  }
  // tag_distances
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->tag_distances), &(output->tag_distances)))
  {
    return false;
  }
  // tag_angles
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->tag_angles), &(output->tag_angles)))
  {
    return false;
  }
  // detection_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->detection_time), &(output->detection_time)))
  {
    return false;
  }
  // alignment_available
  output->alignment_available = input->alignment_available;
  // alignment_center
  if (!geometry_msgs__msg__Point__copy(
      &(input->alignment_center), &(output->alignment_center)))
  {
    return false;
  }
  // alignment_orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->alignment_orientation), &(output->alignment_orientation)))
  {
    return false;
  }
  // arm_target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->arm_target_position), &(output->arm_target_position)))
  {
    return false;
  }
  // alignment_quality
  output->alignment_quality = input->alignment_quality;
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->alignment_warnings), &(output->alignment_warnings)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__DetectAruco_Response *
autonomy_interfaces__srv__DetectAruco_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Response * msg = (autonomy_interfaces__srv__DetectAruco_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectAruco_Response));
  bool success = autonomy_interfaces__srv__DetectAruco_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectAruco_Response__destroy(autonomy_interfaces__srv__DetectAruco_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectAruco_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectAruco_Response__Sequence__init(autonomy_interfaces__srv__DetectAruco_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectAruco_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectAruco_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectAruco_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectAruco_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectAruco_Response__Sequence__fini(autonomy_interfaces__srv__DetectAruco_Response__Sequence * array)
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
      autonomy_interfaces__srv__DetectAruco_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectAruco_Response__Sequence *
autonomy_interfaces__srv__DetectAruco_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Response__Sequence * array = (autonomy_interfaces__srv__DetectAruco_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectAruco_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectAruco_Response__Sequence__destroy(autonomy_interfaces__srv__DetectAruco_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectAruco_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectAruco_Response__Sequence__are_equal(const autonomy_interfaces__srv__DetectAruco_Response__Sequence * lhs, const autonomy_interfaces__srv__DetectAruco_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Response__Sequence__copy(
  const autonomy_interfaces__srv__DetectAruco_Response__Sequence * input,
  autonomy_interfaces__srv__DetectAruco_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectAruco_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectAruco_Response * data =
      (autonomy_interfaces__srv__DetectAruco_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectAruco_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectAruco_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/detect_aruco__functions.h"

bool
autonomy_interfaces__srv__DetectAruco_Event__init(autonomy_interfaces__srv__DetectAruco_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__DetectAruco_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__DetectAruco_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__DetectAruco_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectAruco_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__DetectAruco_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__DetectAruco_Event__fini(autonomy_interfaces__srv__DetectAruco_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__DetectAruco_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__DetectAruco_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__DetectAruco_Event__are_equal(const autonomy_interfaces__srv__DetectAruco_Event * lhs, const autonomy_interfaces__srv__DetectAruco_Event * rhs)
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
  if (!autonomy_interfaces__srv__DetectAruco_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectAruco_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Event__copy(
  const autonomy_interfaces__srv__DetectAruco_Event * input,
  autonomy_interfaces__srv__DetectAruco_Event * output)
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
  if (!autonomy_interfaces__srv__DetectAruco_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectAruco_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__DetectAruco_Event *
autonomy_interfaces__srv__DetectAruco_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Event * msg = (autonomy_interfaces__srv__DetectAruco_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectAruco_Event));
  bool success = autonomy_interfaces__srv__DetectAruco_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectAruco_Event__destroy(autonomy_interfaces__srv__DetectAruco_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectAruco_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectAruco_Event__Sequence__init(autonomy_interfaces__srv__DetectAruco_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectAruco_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectAruco_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectAruco_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectAruco_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectAruco_Event__Sequence__fini(autonomy_interfaces__srv__DetectAruco_Event__Sequence * array)
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
      autonomy_interfaces__srv__DetectAruco_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectAruco_Event__Sequence *
autonomy_interfaces__srv__DetectAruco_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectAruco_Event__Sequence * array = (autonomy_interfaces__srv__DetectAruco_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectAruco_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectAruco_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectAruco_Event__Sequence__destroy(autonomy_interfaces__srv__DetectAruco_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectAruco_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectAruco_Event__Sequence__are_equal(const autonomy_interfaces__srv__DetectAruco_Event__Sequence * lhs, const autonomy_interfaces__srv__DetectAruco_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectAruco_Event__Sequence__copy(
  const autonomy_interfaces__srv__DetectAruco_Event__Sequence * input,
  autonomy_interfaces__srv__DetectAruco_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectAruco_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectAruco_Event * data =
      (autonomy_interfaces__srv__DetectAruco_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectAruco_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectAruco_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectAruco_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
