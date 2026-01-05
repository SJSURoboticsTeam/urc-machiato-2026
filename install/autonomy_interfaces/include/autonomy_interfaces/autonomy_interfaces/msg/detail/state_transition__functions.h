// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/state_transition.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__FUNCTIONS_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "autonomy_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "autonomy_interfaces/msg/detail/state_transition__struct.h"

/// Initialize msg/StateTransition message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * autonomy_interfaces__msg__StateTransition
 * )) before or use
 * autonomy_interfaces__msg__StateTransition__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__init(autonomy_interfaces__msg__StateTransition * msg);

/// Finalize msg/StateTransition message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__StateTransition__fini(autonomy_interfaces__msg__StateTransition * msg);

/// Create msg/StateTransition message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * autonomy_interfaces__msg__StateTransition__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__StateTransition *
autonomy_interfaces__msg__StateTransition__create(void);

/// Destroy msg/StateTransition message.
/**
 * It calls
 * autonomy_interfaces__msg__StateTransition__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__StateTransition__destroy(autonomy_interfaces__msg__StateTransition * msg);

/// Check for msg/StateTransition message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__are_equal(const autonomy_interfaces__msg__StateTransition * lhs, const autonomy_interfaces__msg__StateTransition * rhs);

/// Copy a msg/StateTransition message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__copy(
  const autonomy_interfaces__msg__StateTransition * input,
  autonomy_interfaces__msg__StateTransition * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_type_hash_t *
autonomy_interfaces__msg__StateTransition__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
autonomy_interfaces__msg__StateTransition__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_runtime_c__type_description__TypeSource *
autonomy_interfaces__msg__StateTransition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
autonomy_interfaces__msg__StateTransition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/StateTransition messages.
/**
 * It allocates the memory for the number of elements and calls
 * autonomy_interfaces__msg__StateTransition__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__Sequence__init(autonomy_interfaces__msg__StateTransition__Sequence * array, size_t size);

/// Finalize array of msg/StateTransition messages.
/**
 * It calls
 * autonomy_interfaces__msg__StateTransition__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__StateTransition__Sequence__fini(autonomy_interfaces__msg__StateTransition__Sequence * array);

/// Create array of msg/StateTransition messages.
/**
 * It allocates the memory for the array and calls
 * autonomy_interfaces__msg__StateTransition__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__StateTransition__Sequence *
autonomy_interfaces__msg__StateTransition__Sequence__create(size_t size);

/// Destroy array of msg/StateTransition messages.
/**
 * It calls
 * autonomy_interfaces__msg__StateTransition__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__StateTransition__Sequence__destroy(autonomy_interfaces__msg__StateTransition__Sequence * array);

/// Check for msg/StateTransition message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__Sequence__are_equal(const autonomy_interfaces__msg__StateTransition__Sequence * lhs, const autonomy_interfaces__msg__StateTransition__Sequence * rhs);

/// Copy an array of msg/StateTransition messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__StateTransition__Sequence__copy(
  const autonomy_interfaces__msg__StateTransition__Sequence * input,
  autonomy_interfaces__msg__StateTransition__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__FUNCTIONS_H_
