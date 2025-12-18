// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from autonomy_interfaces:msg/CameraCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__FUNCTIONS_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "autonomy_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "autonomy_interfaces/msg/detail/camera_command__struct.h"

/// Initialize msg/CameraCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * autonomy_interfaces__msg__CameraCommand
 * )) before or use
 * autonomy_interfaces__msg__CameraCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__CameraCommand__init(autonomy_interfaces__msg__CameraCommand * msg);

/// Finalize msg/CameraCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__CameraCommand__fini(autonomy_interfaces__msg__CameraCommand * msg);

/// Create msg/CameraCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * autonomy_interfaces__msg__CameraCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__CameraCommand *
autonomy_interfaces__msg__CameraCommand__create();

/// Destroy msg/CameraCommand message.
/**
 * It calls
 * autonomy_interfaces__msg__CameraCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__CameraCommand__destroy(autonomy_interfaces__msg__CameraCommand * msg);

/// Check for msg/CameraCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__CameraCommand__are_equal(const autonomy_interfaces__msg__CameraCommand * lhs, const autonomy_interfaces__msg__CameraCommand * rhs);

/// Copy a msg/CameraCommand message.
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
autonomy_interfaces__msg__CameraCommand__copy(
  const autonomy_interfaces__msg__CameraCommand * input,
  autonomy_interfaces__msg__CameraCommand * output);

/// Initialize array of msg/CameraCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * autonomy_interfaces__msg__CameraCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__CameraCommand__Sequence__init(autonomy_interfaces__msg__CameraCommand__Sequence * array, size_t size);

/// Finalize array of msg/CameraCommand messages.
/**
 * It calls
 * autonomy_interfaces__msg__CameraCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__CameraCommand__Sequence__fini(autonomy_interfaces__msg__CameraCommand__Sequence * array);

/// Create array of msg/CameraCommand messages.
/**
 * It allocates the memory for the array and calls
 * autonomy_interfaces__msg__CameraCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
autonomy_interfaces__msg__CameraCommand__Sequence *
autonomy_interfaces__msg__CameraCommand__Sequence__create(size_t size);

/// Destroy array of msg/CameraCommand messages.
/**
 * It calls
 * autonomy_interfaces__msg__CameraCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
void
autonomy_interfaces__msg__CameraCommand__Sequence__destroy(autonomy_interfaces__msg__CameraCommand__Sequence * array);

/// Check for msg/CameraCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonomy_interfaces
bool
autonomy_interfaces__msg__CameraCommand__Sequence__are_equal(const autonomy_interfaces__msg__CameraCommand__Sequence * lhs, const autonomy_interfaces__msg__CameraCommand__Sequence * rhs);

/// Copy an array of msg/CameraCommand messages.
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
autonomy_interfaces__msg__CameraCommand__Sequence__copy(
  const autonomy_interfaces__msg__CameraCommand__Sequence * input,
  autonomy_interfaces__msg__CameraCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CAMERA_COMMAND__FUNCTIONS_H_
