// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:action/PerformTyping.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_H_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'typing_goal'
#include "autonomy_interfaces/msg/detail/typing_goal__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_Goal
{
  /// Goal: What to type and where
  autonomy_interfaces__msg__TypingGoal typing_goal;
} autonomy_interfaces__action__PerformTyping_Goal;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_Goal.
typedef struct autonomy_interfaces__action__PerformTyping_Goal__Sequence
{
  autonomy_interfaces__action__PerformTyping_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_character'
#include "rosidl_runtime_c/string.h"
// Member 'current_hand_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_Result
{
  /// 0.0 to 1.0 completion
  float progress;
  /// Character currently being typed
  rosidl_runtime_c__String current_character;
  /// Number of characters successfully typed
  int32_t characters_completed;
  /// Current hand/tool position
  geometry_msgs__msg__PoseStamped current_hand_pose;
} autonomy_interfaces__action__PerformTyping_Result;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_Result.
typedef struct autonomy_interfaces__action__PerformTyping_Result__Sequence
{
  autonomy_interfaces__action__PerformTyping_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'text_typed'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_Feedback
{
  bool success;
  rosidl_runtime_c__String message;
  /// What was actually typed
  rosidl_runtime_c__String text_typed;
  int32_t characters_attempted;
  int32_t characters_successful;
  /// characters_successful / characters_attempted
  float accuracy;
  /// Seconds taken
  float total_time;
} autonomy_interfaces__action__PerformTyping_Feedback;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_Feedback.
typedef struct autonomy_interfaces__action__PerformTyping_Feedback__Sequence
{
  autonomy_interfaces__action__PerformTyping_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "autonomy_interfaces/action/detail/perform_typing__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__PerformTyping_Goal goal;
} autonomy_interfaces__action__PerformTyping_SendGoal_Request;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_SendGoal_Request.
typedef struct autonomy_interfaces__action__PerformTyping_SendGoal_Request__Sequence
{
  autonomy_interfaces__action__PerformTyping_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} autonomy_interfaces__action__PerformTyping_SendGoal_Response;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_SendGoal_Response.
typedef struct autonomy_interfaces__action__PerformTyping_SendGoal_Response__Sequence
{
  autonomy_interfaces__action__PerformTyping_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} autonomy_interfaces__action__PerformTyping_GetResult_Request;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_GetResult_Request.
typedef struct autonomy_interfaces__action__PerformTyping_GetResult_Request__Sequence
{
  autonomy_interfaces__action__PerformTyping_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_GetResult_Response
{
  int8_t status;
  autonomy_interfaces__action__PerformTyping_Result result;
} autonomy_interfaces__action__PerformTyping_GetResult_Response;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_GetResult_Response.
typedef struct autonomy_interfaces__action__PerformTyping_GetResult_Response__Sequence
{
  autonomy_interfaces__action__PerformTyping_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "autonomy_interfaces/action/detail/perform_typing__struct.h"

/// Struct defined in action/PerformTyping in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__PerformTyping_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__PerformTyping_Feedback feedback;
} autonomy_interfaces__action__PerformTyping_FeedbackMessage;

// Struct for a sequence of autonomy_interfaces__action__PerformTyping_FeedbackMessage.
typedef struct autonomy_interfaces__action__PerformTyping_FeedbackMessage__Sequence
{
  autonomy_interfaces__action__PerformTyping_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__PerformTyping_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__STRUCT_H_
