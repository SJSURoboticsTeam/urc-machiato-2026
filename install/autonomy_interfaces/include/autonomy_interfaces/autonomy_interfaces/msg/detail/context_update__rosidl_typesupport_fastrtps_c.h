// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice
#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "autonomy_interfaces/msg/detail/context_update__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
bool cdr_serialize_autonomy_interfaces__msg__ContextUpdate(
  const autonomy_interfaces__msg__ContextUpdate * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
bool cdr_deserialize_autonomy_interfaces__msg__ContextUpdate(
  eprosima::fastcdr::Cdr &,
  autonomy_interfaces__msg__ContextUpdate * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
size_t get_serialized_size_autonomy_interfaces__msg__ContextUpdate(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
size_t max_serialized_size_autonomy_interfaces__msg__ContextUpdate(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
bool cdr_serialize_key_autonomy_interfaces__msg__ContextUpdate(
  const autonomy_interfaces__msg__ContextUpdate * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
size_t get_serialized_size_key_autonomy_interfaces__msg__ContextUpdate(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
size_t max_serialized_size_key_autonomy_interfaces__msg__ContextUpdate(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, autonomy_interfaces, msg, ContextUpdate)();

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
