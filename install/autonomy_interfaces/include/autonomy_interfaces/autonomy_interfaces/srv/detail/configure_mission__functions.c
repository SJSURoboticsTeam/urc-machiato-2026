// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/ConfigureMission.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/configure_mission__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mission_name`
// Member `objectives`
// Member `waypoint_names`
// Member `typing_text`
// Member `terrain_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `waypoints`
// Member `typing_location`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `precision_required`
// Member `waypoint_tolerances`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
autonomy_interfaces__srv__ConfigureMission_Request__init(autonomy_interfaces__srv__ConfigureMission_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mission_name
  if (!rosidl_runtime_c__String__init(&msg->mission_name)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // objectives
  if (!rosidl_runtime_c__String__Sequence__init(&msg->objectives, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->waypoints, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // waypoint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->waypoint_names, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // precision_required
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->precision_required, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // waypoint_tolerances
  if (!rosidl_runtime_c__float__Sequence__init(&msg->waypoint_tolerances, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // time_limit
  // waypoint_timeout
  // max_linear_velocity
  // max_angular_velocity
  // waypoint_approach_tolerance
  // typing_text
  if (!rosidl_runtime_c__String__init(&msg->typing_text)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // typing_location
  if (!geometry_msgs__msg__PoseStamped__init(&msg->typing_location)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // terrain_type
  if (!rosidl_runtime_c__String__init(&msg->terrain_type)) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
    return false;
  }
  // max_incline
  return true;
}

void
autonomy_interfaces__srv__ConfigureMission_Request__fini(autonomy_interfaces__srv__ConfigureMission_Request * msg)
{
  if (!msg) {
    return;
  }
  // mission_name
  rosidl_runtime_c__String__fini(&msg->mission_name);
  // objectives
  rosidl_runtime_c__String__Sequence__fini(&msg->objectives);
  // waypoints
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->waypoints);
  // waypoint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->waypoint_names);
  // precision_required
  rosidl_runtime_c__boolean__Sequence__fini(&msg->precision_required);
  // waypoint_tolerances
  rosidl_runtime_c__float__Sequence__fini(&msg->waypoint_tolerances);
  // time_limit
  // waypoint_timeout
  // max_linear_velocity
  // max_angular_velocity
  // waypoint_approach_tolerance
  // typing_text
  rosidl_runtime_c__String__fini(&msg->typing_text);
  // typing_location
  geometry_msgs__msg__PoseStamped__fini(&msg->typing_location);
  // terrain_type
  rosidl_runtime_c__String__fini(&msg->terrain_type);
  // max_incline
}

bool
autonomy_interfaces__srv__ConfigureMission_Request__are_equal(const autonomy_interfaces__srv__ConfigureMission_Request * lhs, const autonomy_interfaces__srv__ConfigureMission_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mission_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_name), &(rhs->mission_name)))
  {
    return false;
  }
  // objectives
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->objectives), &(rhs->objectives)))
  {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->waypoints), &(rhs->waypoints)))
  {
    return false;
  }
  // waypoint_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->waypoint_names), &(rhs->waypoint_names)))
  {
    return false;
  }
  // precision_required
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->precision_required), &(rhs->precision_required)))
  {
    return false;
  }
  // waypoint_tolerances
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->waypoint_tolerances), &(rhs->waypoint_tolerances)))
  {
    return false;
  }
  // time_limit
  if (lhs->time_limit != rhs->time_limit) {
    return false;
  }
  // waypoint_timeout
  if (lhs->waypoint_timeout != rhs->waypoint_timeout) {
    return false;
  }
  // max_linear_velocity
  if (lhs->max_linear_velocity != rhs->max_linear_velocity) {
    return false;
  }
  // max_angular_velocity
  if (lhs->max_angular_velocity != rhs->max_angular_velocity) {
    return false;
  }
  // waypoint_approach_tolerance
  if (lhs->waypoint_approach_tolerance != rhs->waypoint_approach_tolerance) {
    return false;
  }
  // typing_text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->typing_text), &(rhs->typing_text)))
  {
    return false;
  }
  // typing_location
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->typing_location), &(rhs->typing_location)))
  {
    return false;
  }
  // terrain_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->terrain_type), &(rhs->terrain_type)))
  {
    return false;
  }
  // max_incline
  if (lhs->max_incline != rhs->max_incline) {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__ConfigureMission_Request__copy(
  const autonomy_interfaces__srv__ConfigureMission_Request * input,
  autonomy_interfaces__srv__ConfigureMission_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mission_name
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_name), &(output->mission_name)))
  {
    return false;
  }
  // objectives
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->objectives), &(output->objectives)))
  {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->waypoints), &(output->waypoints)))
  {
    return false;
  }
  // waypoint_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->waypoint_names), &(output->waypoint_names)))
  {
    return false;
  }
  // precision_required
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->precision_required), &(output->precision_required)))
  {
    return false;
  }
  // waypoint_tolerances
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->waypoint_tolerances), &(output->waypoint_tolerances)))
  {
    return false;
  }
  // time_limit
  output->time_limit = input->time_limit;
  // waypoint_timeout
  output->waypoint_timeout = input->waypoint_timeout;
  // max_linear_velocity
  output->max_linear_velocity = input->max_linear_velocity;
  // max_angular_velocity
  output->max_angular_velocity = input->max_angular_velocity;
  // waypoint_approach_tolerance
  output->waypoint_approach_tolerance = input->waypoint_approach_tolerance;
  // typing_text
  if (!rosidl_runtime_c__String__copy(
      &(input->typing_text), &(output->typing_text)))
  {
    return false;
  }
  // typing_location
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->typing_location), &(output->typing_location)))
  {
    return false;
  }
  // terrain_type
  if (!rosidl_runtime_c__String__copy(
      &(input->terrain_type), &(output->terrain_type)))
  {
    return false;
  }
  // max_incline
  output->max_incline = input->max_incline;
  return true;
}

autonomy_interfaces__srv__ConfigureMission_Request *
autonomy_interfaces__srv__ConfigureMission_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Request * msg = (autonomy_interfaces__srv__ConfigureMission_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__ConfigureMission_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ConfigureMission_Request));
  bool success = autonomy_interfaces__srv__ConfigureMission_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ConfigureMission_Request__destroy(autonomy_interfaces__srv__ConfigureMission_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ConfigureMission_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__init(autonomy_interfaces__srv__ConfigureMission_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ConfigureMission_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ConfigureMission_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ConfigureMission_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ConfigureMission_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__fini(autonomy_interfaces__srv__ConfigureMission_Request__Sequence * array)
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
      autonomy_interfaces__srv__ConfigureMission_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__ConfigureMission_Request__Sequence *
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence * array = (autonomy_interfaces__srv__ConfigureMission_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ConfigureMission_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ConfigureMission_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__destroy(autonomy_interfaces__srv__ConfigureMission_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ConfigureMission_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__are_equal(const autonomy_interfaces__srv__ConfigureMission_Request__Sequence * lhs, const autonomy_interfaces__srv__ConfigureMission_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ConfigureMission_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ConfigureMission_Request__Sequence__copy(
  const autonomy_interfaces__srv__ConfigureMission_Request__Sequence * input,
  autonomy_interfaces__srv__ConfigureMission_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ConfigureMission_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ConfigureMission_Request * data =
      (autonomy_interfaces__srv__ConfigureMission_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ConfigureMission_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ConfigureMission_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ConfigureMission_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `mission_id`
// Member `configured_objectives`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__ConfigureMission_Response__init(autonomy_interfaces__srv__ConfigureMission_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__ConfigureMission_Response__fini(msg);
    return false;
  }
  // mission_id
  if (!rosidl_runtime_c__String__init(&msg->mission_id)) {
    autonomy_interfaces__srv__ConfigureMission_Response__fini(msg);
    return false;
  }
  // estimated_duration
  // total_waypoints
  // configured_objectives
  if (!rosidl_runtime_c__String__Sequence__init(&msg->configured_objectives, 0)) {
    autonomy_interfaces__srv__ConfigureMission_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__ConfigureMission_Response__fini(autonomy_interfaces__srv__ConfigureMission_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // mission_id
  rosidl_runtime_c__String__fini(&msg->mission_id);
  // estimated_duration
  // total_waypoints
  // configured_objectives
  rosidl_runtime_c__String__Sequence__fini(&msg->configured_objectives);
}

bool
autonomy_interfaces__srv__ConfigureMission_Response__are_equal(const autonomy_interfaces__srv__ConfigureMission_Response * lhs, const autonomy_interfaces__srv__ConfigureMission_Response * rhs)
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
  // mission_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_id), &(rhs->mission_id)))
  {
    return false;
  }
  // estimated_duration
  if (lhs->estimated_duration != rhs->estimated_duration) {
    return false;
  }
  // total_waypoints
  if (lhs->total_waypoints != rhs->total_waypoints) {
    return false;
  }
  // configured_objectives
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->configured_objectives), &(rhs->configured_objectives)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__ConfigureMission_Response__copy(
  const autonomy_interfaces__srv__ConfigureMission_Response * input,
  autonomy_interfaces__srv__ConfigureMission_Response * output)
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
  // mission_id
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_id), &(output->mission_id)))
  {
    return false;
  }
  // estimated_duration
  output->estimated_duration = input->estimated_duration;
  // total_waypoints
  output->total_waypoints = input->total_waypoints;
  // configured_objectives
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->configured_objectives), &(output->configured_objectives)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__ConfigureMission_Response *
autonomy_interfaces__srv__ConfigureMission_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Response * msg = (autonomy_interfaces__srv__ConfigureMission_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__ConfigureMission_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__ConfigureMission_Response));
  bool success = autonomy_interfaces__srv__ConfigureMission_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__ConfigureMission_Response__destroy(autonomy_interfaces__srv__ConfigureMission_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__ConfigureMission_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__init(autonomy_interfaces__srv__ConfigureMission_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__ConfigureMission_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__ConfigureMission_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__ConfigureMission_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__ConfigureMission_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__fini(autonomy_interfaces__srv__ConfigureMission_Response__Sequence * array)
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
      autonomy_interfaces__srv__ConfigureMission_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__ConfigureMission_Response__Sequence *
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence * array = (autonomy_interfaces__srv__ConfigureMission_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__ConfigureMission_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__ConfigureMission_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__destroy(autonomy_interfaces__srv__ConfigureMission_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__ConfigureMission_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__are_equal(const autonomy_interfaces__srv__ConfigureMission_Response__Sequence * lhs, const autonomy_interfaces__srv__ConfigureMission_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__ConfigureMission_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__ConfigureMission_Response__Sequence__copy(
  const autonomy_interfaces__srv__ConfigureMission_Response__Sequence * input,
  autonomy_interfaces__srv__ConfigureMission_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__ConfigureMission_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__ConfigureMission_Response * data =
      (autonomy_interfaces__srv__ConfigureMission_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__ConfigureMission_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__ConfigureMission_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__ConfigureMission_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
