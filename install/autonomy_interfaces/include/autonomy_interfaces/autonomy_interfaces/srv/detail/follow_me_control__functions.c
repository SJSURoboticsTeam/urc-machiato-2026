// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonomy_interfaces:srv/FollowMeControl.idl
// generated code does not contain a copyright notice
#include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `operator_id`
#include "rosidl_runtime_c/string_functions.h"

bool
autonomy_interfaces__srv__FollowMeControl_Request__init(autonomy_interfaces__srv__FollowMeControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target_tag_id
  // safety_distance
  // max_speed
  // enable_following
  // operator_id
  if (!rosidl_runtime_c__String__init(&msg->operator_id)) {
    autonomy_interfaces__srv__FollowMeControl_Request__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__FollowMeControl_Request__fini(autonomy_interfaces__srv__FollowMeControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // target_tag_id
  // safety_distance
  // max_speed
  // enable_following
  // operator_id
  rosidl_runtime_c__String__fini(&msg->operator_id);
}

bool
autonomy_interfaces__srv__FollowMeControl_Request__are_equal(const autonomy_interfaces__srv__FollowMeControl_Request * lhs, const autonomy_interfaces__srv__FollowMeControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_tag_id
  if (lhs->target_tag_id != rhs->target_tag_id) {
    return false;
  }
  // safety_distance
  if (lhs->safety_distance != rhs->safety_distance) {
    return false;
  }
  // max_speed
  if (lhs->max_speed != rhs->max_speed) {
    return false;
  }
  // enable_following
  if (lhs->enable_following != rhs->enable_following) {
    return false;
  }
  // operator_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operator_id), &(rhs->operator_id)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Request__copy(
  const autonomy_interfaces__srv__FollowMeControl_Request * input,
  autonomy_interfaces__srv__FollowMeControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target_tag_id
  output->target_tag_id = input->target_tag_id;
  // safety_distance
  output->safety_distance = input->safety_distance;
  // max_speed
  output->max_speed = input->max_speed;
  // enable_following
  output->enable_following = input->enable_following;
  // operator_id
  if (!rosidl_runtime_c__String__copy(
      &(input->operator_id), &(output->operator_id)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__FollowMeControl_Request *
autonomy_interfaces__srv__FollowMeControl_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Request * msg = (autonomy_interfaces__srv__FollowMeControl_Request *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__FollowMeControl_Request));
  bool success = autonomy_interfaces__srv__FollowMeControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__FollowMeControl_Request__destroy(autonomy_interfaces__srv__FollowMeControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__FollowMeControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__init(autonomy_interfaces__srv__FollowMeControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Request * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__FollowMeControl_Request *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__FollowMeControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__FollowMeControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__FollowMeControl_Request__fini(&data[i - 1]);
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
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__fini(autonomy_interfaces__srv__FollowMeControl_Request__Sequence * array)
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
      autonomy_interfaces__srv__FollowMeControl_Request__fini(&array->data[i]);
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

autonomy_interfaces__srv__FollowMeControl_Request__Sequence *
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence * array = (autonomy_interfaces__srv__FollowMeControl_Request__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__FollowMeControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__destroy(autonomy_interfaces__srv__FollowMeControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__FollowMeControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__are_equal(const autonomy_interfaces__srv__FollowMeControl_Request__Sequence * lhs, const autonomy_interfaces__srv__FollowMeControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Request__Sequence__copy(
  const autonomy_interfaces__srv__FollowMeControl_Request__Sequence * input,
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__FollowMeControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__FollowMeControl_Request * data =
      (autonomy_interfaces__srv__FollowMeControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__FollowMeControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__FollowMeControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `target_position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
autonomy_interfaces__srv__FollowMeControl_Response__init(autonomy_interfaces__srv__FollowMeControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    autonomy_interfaces__srv__FollowMeControl_Response__fini(msg);
    return false;
  }
  // is_following
  // current_target_tag
  // current_distance
  // target_position
  if (!geometry_msgs__msg__Point__init(&msg->target_position)) {
    autonomy_interfaces__srv__FollowMeControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__FollowMeControl_Response__fini(autonomy_interfaces__srv__FollowMeControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // is_following
  // current_target_tag
  // current_distance
  // target_position
  geometry_msgs__msg__Point__fini(&msg->target_position);
}

bool
autonomy_interfaces__srv__FollowMeControl_Response__are_equal(const autonomy_interfaces__srv__FollowMeControl_Response * lhs, const autonomy_interfaces__srv__FollowMeControl_Response * rhs)
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
  // is_following
  if (lhs->is_following != rhs->is_following) {
    return false;
  }
  // current_target_tag
  if (lhs->current_target_tag != rhs->current_target_tag) {
    return false;
  }
  // current_distance
  if (lhs->current_distance != rhs->current_distance) {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_position), &(rhs->target_position)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Response__copy(
  const autonomy_interfaces__srv__FollowMeControl_Response * input,
  autonomy_interfaces__srv__FollowMeControl_Response * output)
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
  // is_following
  output->is_following = input->is_following;
  // current_target_tag
  output->current_target_tag = input->current_target_tag;
  // current_distance
  output->current_distance = input->current_distance;
  // target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_position), &(output->target_position)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__FollowMeControl_Response *
autonomy_interfaces__srv__FollowMeControl_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Response * msg = (autonomy_interfaces__srv__FollowMeControl_Response *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__FollowMeControl_Response));
  bool success = autonomy_interfaces__srv__FollowMeControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__FollowMeControl_Response__destroy(autonomy_interfaces__srv__FollowMeControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__FollowMeControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__init(autonomy_interfaces__srv__FollowMeControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Response * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__FollowMeControl_Response *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__FollowMeControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__FollowMeControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__FollowMeControl_Response__fini(&data[i - 1]);
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
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__fini(autonomy_interfaces__srv__FollowMeControl_Response__Sequence * array)
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
      autonomy_interfaces__srv__FollowMeControl_Response__fini(&array->data[i]);
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

autonomy_interfaces__srv__FollowMeControl_Response__Sequence *
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence * array = (autonomy_interfaces__srv__FollowMeControl_Response__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__FollowMeControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__destroy(autonomy_interfaces__srv__FollowMeControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__FollowMeControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__are_equal(const autonomy_interfaces__srv__FollowMeControl_Response__Sequence * lhs, const autonomy_interfaces__srv__FollowMeControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Response__Sequence__copy(
  const autonomy_interfaces__srv__FollowMeControl_Response__Sequence * input,
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__FollowMeControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__FollowMeControl_Response * data =
      (autonomy_interfaces__srv__FollowMeControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__FollowMeControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__FollowMeControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Response__copy(
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
// #include "autonomy_interfaces/srv/detail/follow_me_control__functions.h"

bool
autonomy_interfaces__srv__FollowMeControl_Event__init(autonomy_interfaces__srv__FollowMeControl_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    autonomy_interfaces__srv__FollowMeControl_Event__fini(msg);
    return false;
  }
  // request
  if (!autonomy_interfaces__srv__FollowMeControl_Request__Sequence__init(&msg->request, 0)) {
    autonomy_interfaces__srv__FollowMeControl_Event__fini(msg);
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__FollowMeControl_Response__Sequence__init(&msg->response, 0)) {
    autonomy_interfaces__srv__FollowMeControl_Event__fini(msg);
    return false;
  }
  return true;
}

void
autonomy_interfaces__srv__FollowMeControl_Event__fini(autonomy_interfaces__srv__FollowMeControl_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  autonomy_interfaces__srv__FollowMeControl_Request__Sequence__fini(&msg->request);
  // response
  autonomy_interfaces__srv__FollowMeControl_Response__Sequence__fini(&msg->response);
}

bool
autonomy_interfaces__srv__FollowMeControl_Event__are_equal(const autonomy_interfaces__srv__FollowMeControl_Event * lhs, const autonomy_interfaces__srv__FollowMeControl_Event * rhs)
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
  if (!autonomy_interfaces__srv__FollowMeControl_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__FollowMeControl_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Event__copy(
  const autonomy_interfaces__srv__FollowMeControl_Event * input,
  autonomy_interfaces__srv__FollowMeControl_Event * output)
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
  if (!autonomy_interfaces__srv__FollowMeControl_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!autonomy_interfaces__srv__FollowMeControl_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

autonomy_interfaces__srv__FollowMeControl_Event *
autonomy_interfaces__srv__FollowMeControl_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Event * msg = (autonomy_interfaces__srv__FollowMeControl_Event *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonomy_interfaces__srv__FollowMeControl_Event));
  bool success = autonomy_interfaces__srv__FollowMeControl_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonomy_interfaces__srv__FollowMeControl_Event__destroy(autonomy_interfaces__srv__FollowMeControl_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonomy_interfaces__srv__FollowMeControl_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__init(autonomy_interfaces__srv__FollowMeControl_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Event * data = NULL;

  if (size) {
    data = (autonomy_interfaces__srv__FollowMeControl_Event *)allocator.zero_allocate(size, sizeof(autonomy_interfaces__srv__FollowMeControl_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonomy_interfaces__srv__FollowMeControl_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonomy_interfaces__srv__FollowMeControl_Event__fini(&data[i - 1]);
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
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__fini(autonomy_interfaces__srv__FollowMeControl_Event__Sequence * array)
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
      autonomy_interfaces__srv__FollowMeControl_Event__fini(&array->data[i]);
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

autonomy_interfaces__srv__FollowMeControl_Event__Sequence *
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonomy_interfaces__srv__FollowMeControl_Event__Sequence * array = (autonomy_interfaces__srv__FollowMeControl_Event__Sequence *)allocator.allocate(sizeof(autonomy_interfaces__srv__FollowMeControl_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonomy_interfaces__srv__FollowMeControl_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__destroy(autonomy_interfaces__srv__FollowMeControl_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonomy_interfaces__srv__FollowMeControl_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__are_equal(const autonomy_interfaces__srv__FollowMeControl_Event__Sequence * lhs, const autonomy_interfaces__srv__FollowMeControl_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonomy_interfaces__srv__FollowMeControl_Event__Sequence__copy(
  const autonomy_interfaces__srv__FollowMeControl_Event__Sequence * input,
  autonomy_interfaces__srv__FollowMeControl_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonomy_interfaces__srv__FollowMeControl_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonomy_interfaces__srv__FollowMeControl_Event * data =
      (autonomy_interfaces__srv__FollowMeControl_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonomy_interfaces__srv__FollowMeControl_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonomy_interfaces__srv__FollowMeControl_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonomy_interfaces__srv__FollowMeControl_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
