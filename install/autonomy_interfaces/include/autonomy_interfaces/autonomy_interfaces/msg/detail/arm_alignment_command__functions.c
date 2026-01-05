// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:msg/ArmAlignmentCommand.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/msg/detail/arm_alignment_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mission_type`
// Member `alignment_id`
// Member `safety_zones`
// Member `required_aruco_tags`
#include "rosidl_runtime_c/string_functions.h"
// Member `target_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `target_orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
autonomy_interfaces__msg__ArmAlignmentCommand__init(autonomy_interfaces__msg__ArmAlignmentCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // mission_type
  if (!rosidl_runtime_c__String__init(&msg->mission_type)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // alignment_id
  if (!rosidl_runtime_c__String__init(&msg->alignment_id)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__init(&msg->target_position)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // target_orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->target_orientation)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // approach_distance
  // final_distance
  // alignment_quality
  // max_position_error
  // max_orientation_error
  // alignment_timeout
  // max_approach_speed
  // max_rotation_speed
  // enable_collision_avoidance
  // safety_zones
  if (!rosidl_runtime_c__String__Sequence__init(&msg->safety_zones, 0)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // require_position_feedback
  // require_force_feedback
  // feedback_rate
  // required_aruco_tags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->required_aruco_tags, 0)) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
    return false;
  }
  // tag_visibility_timeout
  // allow_realignment
  return true;
}

void
autonomy_interfaces__msg__ArmAlignmentCommand__fini(autonomy_interfaces__msg__ArmAlignmentCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mission_type
  rosidl_runtime_c__String__fini(&msg->mission_type);
  // alignment_id
  rosidl_runtime_c__String__fini(&msg->alignment_id);
  // target_position
  geometry_msgs__msg__Point__fini(&msg->target_position);
  // target_orientation
  geometry_msgs__msg__Quaternion__fini(&msg->target_orientation);
  // approach_distance
  // final_distance
  // alignment_quality
  // max_position_error
  // max_orientation_error
  // alignment_timeout
  // max_approach_speed
  // max_rotation_speed
  // enable_collision_avoidance
  // safety_zones
  rosidl_runtime_c__String__Sequence__fini(&msg->safety_zones);
  // require_position_feedback
  // require_force_feedback
  // feedback_rate
  // required_aruco_tags
  rosidl_runtime_c__String__Sequence__fini(&msg->required_aruco_tags);
  // tag_visibility_timeout
  // allow_realignment
}

bool
autonomy_interfaces__msg__ArmAlignmentCommand__are_equal(const autonomy_interfaces__msg__ArmAlignmentCommand * lhs, const autonomy_interfaces__msg__ArmAlignmentCommand * rhs)
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
  // mission_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_type), &(rhs->mission_type)))
  {
    return false;
  }
  // alignment_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->alignment_id), &(rhs->alignment_id)))
  {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_position), &(rhs->target_position)))
  {
    return false;
  }
  // target_orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->target_orientation), &(rhs->target_orientation)))
  {
    return false;
  }
  // approach_distance
  if (lhs->approach_distance != rhs->approach_distance) {
    return false;
  }
  // final_distance
  if (lhs->final_distance != rhs->final_distance) {
    return false;
  }
  // alignment_quality
  if (lhs->alignment_quality != rhs->alignment_quality) {
    return false;
  }
  // max_position_error
  if (lhs->max_position_error != rhs->max_position_error) {
    return false;
  }
  // max_orientation_error
  if (lhs->max_orientation_error != rhs->max_orientation_error) {
    return false;
  }
  // alignment_timeout
  if (lhs->alignment_timeout != rhs->alignment_timeout) {
    return false;
  }
  // max_approach_speed
  if (lhs->max_approach_speed != rhs->max_approach_speed) {
    return false;
  }
  // max_rotation_speed
  if (lhs->max_rotation_speed != rhs->max_rotation_speed) {
    return false;
  }
  // enable_collision_avoidance
  if (lhs->enable_collision_avoidance != rhs->enable_collision_avoidance) {
    return false;
  }
  // safety_zones
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->safety_zones), &(rhs->safety_zones)))
  {
    return false;
  }
  // require_position_feedback
  if (lhs->require_position_feedback != rhs->require_position_feedback) {
    return false;
  }
  // require_force_feedback
  if (lhs->require_force_feedback != rhs->require_force_feedback) {
    return false;
  }
  // feedback_rate
  if (lhs->feedback_rate != rhs->feedback_rate) {
    return false;
  }
  // required_aruco_tags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->required_aruco_tags), &(rhs->required_aruco_tags)))
  {
    return false;
  }
  // tag_visibility_timeout
  if (lhs->tag_visibility_timeout != rhs->tag_visibility_timeout) {
    return false;
  }
  // allow_realignment
  if (lhs->allow_realignment != rhs->allow_realignment) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__msg__ArmAlignmentCommand__copy(
  const autonomy_interfaces__msg__ArmAlignmentCommand * input,
  autonomy_interfaces__msg__ArmAlignmentCommand * output)
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
  // mission_type
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_type), &(output->mission_type)))
  {
    return false;
  }
  // alignment_id
  if (!rosidl_runtime_c__String__copy(
      &(input->alignment_id), &(output->alignment_id)))
  {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_position), &(output->target_position)))
  {
    return false;
  }
  // target_orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->target_orientation), &(output->target_orientation)))
  {
    return false;
  }
  // approach_distance
  output->approach_distance = input->approach_distance;
  // final_distance
  output->final_distance = input->final_distance;
  // alignment_quality
  output->alignment_quality = input->alignment_quality;
  // max_position_error
  output->max_position_error = input->max_position_error;
  // max_orientation_error
  output->max_orientation_error = input->max_orientation_error;
  // alignment_timeout
  output->alignment_timeout = input->alignment_timeout;
  // max_approach_speed
  output->max_approach_speed = input->max_approach_speed;
  // max_rotation_speed
  output->max_rotation_speed = input->max_rotation_speed;
  // enable_collision_avoidance
  output->enable_collision_avoidance = input->enable_collision_avoidance;
  // safety_zones
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->safety_zones), &(output->safety_zones)))
  {
    return false;
  }
  // require_position_feedback
  output->require_position_feedback = input->require_position_feedback;
  // require_force_feedback
  output->require_force_feedback = input->require_force_feedback;
  // feedback_rate
  output->feedback_rate = input->feedback_rate;
  // required_aruco_tags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->required_aruco_tags), &(output->required_aruco_tags)))
  {
    return false;
  }
  // tag_visibility_timeout
  output->tag_visibility_timeout = input->tag_visibility_timeout;
  // allow_realignment
  output->allow_realignment = input->allow_realignment;
  return true;
}

autonomy_interfaces__msg__ArmAlignmentCommand *
autonomy_interfaces__msg__ArmAlignmentCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ArmAlignmentCommand * msg = (autonomy_interfaces__msg__ArmAlignmentCommand *)allocator.allocate(sizeof(autonomy_interfaces__msg__ArmAlignmentCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__msg__ArmAlignmentCommand));
  bool success = autonomy_interfaces__msg__ArmAlignmentCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__msg__ArmAlignmentCommand__destroy(autonomy_interfaces__msg__ArmAlignmentCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__msg__ArmAlignmentCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__init(autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ArmAlignmentCommand * data = NULL;

  if (size) {
    data = (autonomy_interfaces__msg__ArmAlignmentCommand *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__msg__ArmAlignmentCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__msg__ArmAlignmentCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__msg__ArmAlignmentCommand__fini(&data[i - 1]);
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
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__fini(autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * array)
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
      autonomy_interfaces__msg__ArmAlignmentCommand__fini(&array->data[i]);
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

autonomy_interfaces__msg__ArmAlignmentCommand__Sequence *
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * array = (autonomy_interfaces__msg__ArmAlignmentCommand__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__msg__ArmAlignmentCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__destroy(autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__are_equal(const autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * lhs, const autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__msg__ArmAlignmentCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__msg__ArmAlignmentCommand__Sequence__copy(
  const autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * input,
  autonomy_interfaces__msg__ArmAlignmentCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__msg__ArmAlignmentCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__msg__ArmAlignmentCommand * data =
      (autonomy_interfaces__msg__ArmAlignmentCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__msg__ArmAlignmentCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__msg__ArmAlignmentCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__msg__ArmAlignmentCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
