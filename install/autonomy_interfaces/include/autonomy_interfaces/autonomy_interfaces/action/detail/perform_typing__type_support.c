// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonomy_interfaces:action/PerformTyping.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
#include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonomy_interfaces/action/detail/perform_typing__functions.h"
#include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `typing_goal`
#include "autonomy_interfaces/msg/typing_goal.h"
// Member `typing_goal`
#include "autonomy_interfaces/msg/detail/typing_goal__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_Goal__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_member_array[1] = {
  {
    "typing_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Goal, typing_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_Goal",  // message name
  1,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_Goal),
  autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Goal)() {
  autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, msg, TypingGoal)();
  if (!autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_Goal__rosidl_typesupport_introspection_c__PerformTyping_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `current_character`
#include "rosidl_runtime_c/string_functions.h"
// Member `current_hand_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `current_hand_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_Result__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_member_array[4] = {
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Result, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_character",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Result, current_character),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "characters_completed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Result, characters_completed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_hand_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Result, current_hand_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_Result",  // message name
  4,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_Result),
  autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Result)() {
  autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_Result__rosidl_typesupport_introspection_c__PerformTyping_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `message`
// Member `text_typed`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_Feedback__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "text_typed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, text_typed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "characters_attempted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, characters_attempted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "characters_successful",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, characters_successful),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accuracy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, accuracy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_Feedback, total_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_Feedback",  // message name
  7,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_Feedback),
  autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Feedback)() {
  if (!autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_Feedback__rosidl_typesupport_introspection_c__PerformTyping_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "autonomy_interfaces/action/perform_typing.h"
// Member `goal`
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_SendGoal_Request),
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Request)() {
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Goal)();
  if (!autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_SendGoal_Request__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_SendGoal_Response),
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Response)() {
  autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_SendGoal_Response__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_members = {
  "autonomy_interfaces__action",  // service namespace
  "PerformTyping_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_type_support_handle = {
  0,
  &autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal)() {
  if (!autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_SendGoal_Response)()->data;
  }

  return &autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_GetResult_Request__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_GetResult_Request),
  autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Request)() {
  autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_GetResult_Request__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "autonomy_interfaces/action/perform_typing.h"
// Member `result`
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_GetResult_Response__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_GetResult_Response),
  autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Response)() {
  autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Result)();
  if (!autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_GetResult_Response__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_members = {
  "autonomy_interfaces__action",  // service namespace
  "PerformTyping_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_type_support_handle = {
  0,
  &autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult)() {
  if (!autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_GetResult_Response)()->data;
  }

  return &autonomy_interfaces__action__detail__perform_typing__rosidl_typesupport_introspection_c__PerformTyping_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autonomy_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__functions.h"
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "autonomy_interfaces/action/perform_typing.h"
// Member `feedback`
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__init(message_memory);
}

void autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_fini_function(void * message_memory)
{
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonomy_interfaces__action__PerformTyping_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_members = {
  "autonomy_interfaces__action",  // message namespace
  "PerformTyping_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(autonomy_interfaces__action__PerformTyping_FeedbackMessage),
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_member_array,  // message members
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_type_support_handle = {
  0,
  &autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonomy_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_FeedbackMessage)() {
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonomy_interfaces, action, PerformTyping_Feedback)();
  if (!autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonomy_interfaces__action__PerformTyping_FeedbackMessage__rosidl_typesupport_introspection_c__PerformTyping_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
