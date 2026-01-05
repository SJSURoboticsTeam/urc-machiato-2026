// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/DetectMissionAruco.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mission_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `required_tag_ids`
// Member `optional_tag_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__srv__DetectMissionAruco_Request__init(autonomy_interfaces__srv__DetectMissionAruco_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__init(&msg->mission_type)) {
    autonomy_interfaces__srv__DetectMissionAruco_Request__fini(msg);
    return false;
  }
  // required_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->required_tag_ids, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Request__fini(msg);
    return false;
  }
  // optional_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->optional_tag_ids, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Request__fini(msg);
    return false;
  }
  // detection_timeout
  // target_depth
  // max_detection_distance
  // require_all_tags
  // min_alignment_quality
  return true;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Request__fini(autonomy_interfaces__srv__DetectMissionAruco_Request * msg)
{
  if (!msg) {
    return;
  }
  // mission_type
  rosidl_runtime_c__String__fini(&msg->mission_type);
  // required_tag_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->required_tag_ids);
  // optional_tag_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->optional_tag_ids);
  // detection_timeout
  // target_depth
  // max_detection_distance
  // require_all_tags
  // min_alignment_quality
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Request__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Request * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_type), &(rhs->mission_type)))
  {
    return false;
  }
  // required_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->required_tag_ids), &(rhs->required_tag_ids)))
  {
    return false;
  }
  // optional_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->optional_tag_ids), &(rhs->optional_tag_ids)))
  {
    return false;
  }
  // detection_timeout
  if (lhs->detection_timeout != rhs->detection_timeout) {
    return false;
  }
  // target_depth
  if (lhs->target_depth != rhs->target_depth) {
    return false;
  }
  // max_detection_distance
  if (lhs->max_detection_distance != rhs->max_detection_distance) {
    return false;
  }
  // require_all_tags
  if (lhs->require_all_tags != rhs->require_all_tags) {
    return false;
  }
  // min_alignment_quality
  if (lhs->min_alignment_quality != rhs->min_alignment_quality) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Request__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Request * input,
  autonomy_interfaces__srv__DetectMissionAruco_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_type), &(output->mission_type)))
  {
    return false;
  }
  // required_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->required_tag_ids), &(output->required_tag_ids)))
  {
    return false;
  }
  // optional_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->optional_tag_ids), &(output->optional_tag_ids)))
  {
    return false;
  }
  // detection_timeout
  output->detection_timeout = input->detection_timeout;
  // target_depth
  output->target_depth = input->target_depth;
  // max_detection_distance
  output->max_detection_distance = input->max_detection_distance;
  // require_all_tags
  output->require_all_tags = input->require_all_tags;
  // min_alignment_quality
  output->min_alignment_quality = input->min_alignment_quality;
  return true;
}

autonomy_interfaces__srv__DetectMissionAruco_Request *
autonomy_interfaces__srv__DetectMissionAruco_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Request * msg = (autonomy_interfaces__srv__DetectMissionAruco_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Request));
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Request__destroy(autonomy_interfaces__srv__DetectMissionAruco_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectMissionAruco_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__init(autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectMissionAruco_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectMissionAruco_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectMissionAruco_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__fini(autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * array)
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
      autonomy_interfaces__srv__DetectMissionAruco_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * array = (autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__destroy(autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * input,
  autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectMissionAruco_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectMissionAruco_Request * data =
      (autonomy_interfaces__srv__DetectMissionAruco_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectMissionAruco_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectMissionAruco_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `mission_type`
// Member `missing_required_tags`
// Member `detected_optional_tags`
// Member `alignment_warnings`
// Member `mission_recommendations`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `detected_tag_ids`
// Member `tag_distances`
// Member `tag_angles`
// Member `alignment_errors`
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
autonomy_interfaces__srv__DetectMissionAruco_Response__init(autonomy_interfaces__srv__DetectMissionAruco_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__init(&msg->mission_type)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // detected_tag_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->detected_tag_ids, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // tag_positions
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->tag_positions, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // tag_distances
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tag_distances, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // tag_angles
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tag_angles, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // detection_time
  if (!builtin_interfaces__msg__Time__init(&msg->detection_time)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // alignment_available
  // alignment_center
  if (!geometry_msgs__msg__Point__init(&msg->alignment_center)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // alignment_orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->alignment_orientation)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // arm_target_position
  if (!geometry_msgs__msg__Point__init(&msg->arm_target_position)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // alignment_quality
  // alignment_errors
  if (!rosidl_runtime_c__float__Sequence__init(&msg->alignment_errors, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // mission_ready
  // missing_required_tags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->missing_required_tags, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // detected_optional_tags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->detected_optional_tags, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__init(&msg->alignment_warnings, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  // mission_recommendations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->mission_recommendations, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Response__fini(autonomy_interfaces__srv__DetectMissionAruco_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // mission_type
  rosidl_runtime_c__String__fini(&msg->mission_type);
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
  // alignment_errors
  rosidl_runtime_c__float__Sequence__fini(&msg->alignment_errors);
  // mission_ready
  // missing_required_tags
  rosidl_runtime_c__String__Sequence__fini(&msg->missing_required_tags);
  // detected_optional_tags
  rosidl_runtime_c__String__Sequence__fini(&msg->detected_optional_tags);
  // alignment_warnings
  rosidl_runtime_c__String__Sequence__fini(&msg->alignment_warnings);
  // mission_recommendations
  rosidl_runtime_c__String__Sequence__fini(&msg->mission_recommendations);
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Response__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Response * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Response * rhs)
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
  // mission_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_type), &(rhs->mission_type)))
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
  // alignment_errors
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->alignment_errors), &(rhs->alignment_errors)))
  {
    return false;
  }
  // mission_ready
  if (lhs->mission_ready != rhs->mission_ready) {
    return false;
  }
  // missing_required_tags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->missing_required_tags), &(rhs->missing_required_tags)))
  {
    return false;
  }
  // detected_optional_tags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->detected_optional_tags), &(rhs->detected_optional_tags)))
  {
    return false;
  }
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->alignment_warnings), &(rhs->alignment_warnings)))
  {
    return false;
  }
  // mission_recommendations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->mission_recommendations), &(rhs->mission_recommendations)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Response__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Response * input,
  autonomy_interfaces__srv__DetectMissionAruco_Response * output)
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
  // mission_type
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_type), &(output->mission_type)))
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
  // alignment_errors
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->alignment_errors), &(output->alignment_errors)))
  {
    return false;
  }
  // mission_ready
  output->mission_ready = input->mission_ready;
  // missing_required_tags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->missing_required_tags), &(output->missing_required_tags)))
  {
    return false;
  }
  // detected_optional_tags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->detected_optional_tags), &(output->detected_optional_tags)))
  {
    return false;
  }
  // alignment_warnings
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->alignment_warnings), &(output->alignment_warnings)))
  {
    return false;
  }
  // mission_recommendations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->mission_recommendations), &(output->mission_recommendations)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__DetectMissionAruco_Response *
autonomy_interfaces__srv__DetectMissionAruco_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Response * msg = (autonomy_interfaces__srv__DetectMissionAruco_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Response));
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Response__destroy(autonomy_interfaces__srv__DetectMissionAruco_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__init(autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectMissionAruco_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectMissionAruco_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectMissionAruco_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__fini(autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * array)
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
      autonomy_interfaces__srv__DetectMissionAruco_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * array = (autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__destroy(autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * input,
  autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectMissionAruco_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectMissionAruco_Response * data =
      (autonomy_interfaces__srv__DetectMissionAruco_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectMissionAruco_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectMissionAruco_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/detect_mission_aruco__functions.h"

bool
autonomy_interfaces__srv__DetectMissionAruco_Event__init(autonomy_interfaces__srv__DetectMissionAruco_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__DetectMissionAruco_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__DetectMissionAruco_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Event__fini(autonomy_interfaces__srv__DetectMissionAruco_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Event__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Event * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Event * rhs)
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
  if (!autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Event__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Event * input,
  autonomy_interfaces__srv__DetectMissionAruco_Event * output)
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
  if (!autonomy_interfaces__srv__DetectMissionAruco_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__DetectMissionAruco_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__DetectMissionAruco_Event *
autonomy_interfaces__srv__DetectMissionAruco_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Event * msg = (autonomy_interfaces__srv__DetectMissionAruco_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Event));
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Event__destroy(autonomy_interfaces__srv__DetectMissionAruco_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__DetectMissionAruco_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__init(autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__DetectMissionAruco_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__DetectMissionAruco_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__DetectMissionAruco_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__DetectMissionAruco_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__fini(autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * array)
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
      autonomy_interfaces__srv__DetectMissionAruco_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence *
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * array = (autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__destroy(autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__are_equal(const autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * lhs, const autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence__copy(
  const autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * input,
  autonomy_interfaces__srv__DetectMissionAruco_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__DetectMissionAruco_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__DetectMissionAruco_Event * data =
      (autonomy_interfaces__srv__DetectMissionAruco_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__DetectMissionAruco_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__DetectMissionAruco_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__DetectMissionAruco_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
