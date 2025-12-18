// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from autonomy_interfaces:msg/TypingGoal.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__FUNCTIONS_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "autonomy_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "autonomy_interfaces/msg/detail/typing_goal__struct.h"

/// Initialize msg/TypingGoal message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * autonomy_interfaces__msg__TypingGoal
 * )) before or use
 * autonomy_interfaces__msg__TypingGoal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__TypingGoal__init(autonomy_interfaces__msg__TypingGoal * msg);

/// Finalize msg/TypingGoal message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__TypingGoal__fini(autonomy_interfaces__msg__TypingGoal * msg);

/// Create msg/TypingGoal message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * autonomy_interfaces__msg__TypingGoal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__TypingGoal *
autonomy_interfaces__msg__TypingGoal__create();

/// Destroy msg/TypingGoal message.
/**
 * It calls
 * autonomy_interfaces__msg__TypingGoal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__TypingGoal__destroy(autonomy_interfaces__msg__TypingGoal * msg);

/// Check for msg/TypingGoal message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__TypingGoal__are_equal(const autonomy_interfaces__msg__TypingGoal * lhs, const autonomy_interfaces__msg__TypingGoal * rhs);

/// Copy a msg/TypingGoal message.
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
autonomy_interfaces__msg__TypingGoal__copy(
  const autonomy_interfaces__msg__TypingGoal * input,
  autonomy_interfaces__msg__TypingGoal * output);

/// Initialize array of msg/TypingGoal messages.
/**
 * It allocates the memory for the number of elements and calls
 * autonomy_interfaces__msg__TypingGoal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__TypingGoal__Sequence__init(autonomy_interfaces__msg__TypingGoal__Sequence * array, size_t size);

/// Finalize array of msg/TypingGoal messages.
/**
 * It calls
 * autonomy_interfaces__msg__TypingGoal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__TypingGoal__Sequence__fini(autonomy_interfaces__msg__TypingGoal__Sequence * array);

/// Create array of msg/TypingGoal messages.
/**
 * It allocates the memory for the array and calls
 * autonomy_interfaces__msg__TypingGoal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__TypingGoal__Sequence *
autonomy_interfaces__msg__TypingGoal__Sequence__create(size_t size);

/// Destroy array of msg/TypingGoal messages.
/**
 * It calls
 * autonomy_interfaces__msg__TypingGoal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__TypingGoal__Sequence__destroy(autonomy_interfaces__msg__TypingGoal__Sequence * array);

/// Check for msg/TypingGoal message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__TypingGoal__Sequence__are_equal(const autonomy_interfaces__msg__TypingGoal__Sequence * lhs, const autonomy_interfaces__msg__TypingGoal__Sequence * rhs);

/// Copy an array of msg/TypingGoal messages.
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
autonomy_interfaces__msg__TypingGoal__Sequence__copy(
  const autonomy_interfaces__msg__TypingGoal__Sequence * input,
  autonomy_interfaces__msg__TypingGoal__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__TYPING_GOAL__FUNCTIONS_H_
