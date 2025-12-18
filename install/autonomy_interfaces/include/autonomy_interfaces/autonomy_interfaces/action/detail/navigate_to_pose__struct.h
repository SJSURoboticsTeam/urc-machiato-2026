// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:action/NavigateToPose.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__STRUCT_H_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_Goal
{
  /// Goal: Where to go
  geometry_msgs__msg__PoseStamped target_pose;
  /// Acceptable distance error (meters)
  float tolerance;
  /// Maximum time allowed (seconds, 0 = no limit)
  float timeout;
} autonomy_interfaces__action__NavigateToPose_Goal;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_Goal.
typedef struct autonomy_interfaces__action__NavigateToPose_Goal__Sequence
{
  autonomy_interfaces__action__NavigateToPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'navigation_state'
#include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_Result
{
  /// Current distance remaining (meters)
  float distance_to_goal;
  /// Seconds until arrival
  float estimated_time_remaining;
  /// Current navigation state
  rosidl_runtime_c__String navigation_state;
  /// Current robot pose
  geometry_msgs__msg__PoseStamped current_pose;
} autonomy_interfaces__action__NavigateToPose_Result;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_Result.
typedef struct autonomy_interfaces__action__NavigateToPose_Result__Sequence
{
  autonomy_interfaces__action__NavigateToPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_Feedback
{
  bool success;
  rosidl_runtime_c__String message;
  /// Where robot ended up
  geometry_msgs__msg__PoseStamped final_pose;
  /// Meters traveled during navigation
  float total_distance_traveled;
  /// Seconds taken to complete
  float total_time;
} autonomy_interfaces__action__NavigateToPose_Feedback;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_Feedback.
typedef struct autonomy_interfaces__action__NavigateToPose_Feedback__Sequence
{
  autonomy_interfaces__action__NavigateToPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "autonomy_interfaces/action/detail/navigate_to_pose__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__NavigateToPose_Goal goal;
} autonomy_interfaces__action__NavigateToPose_SendGoal_Request;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_SendGoal_Request.
typedef struct autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence
{
  autonomy_interfaces__action__NavigateToPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} autonomy_interfaces__action__NavigateToPose_SendGoal_Response;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_SendGoal_Response.
typedef struct autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence
{
  autonomy_interfaces__action__NavigateToPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} autonomy_interfaces__action__NavigateToPose_GetResult_Request;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_GetResult_Request.
typedef struct autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence
{
  autonomy_interfaces__action__NavigateToPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "autonomy_interfaces/action/detail/navigate_to_pose__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_GetResult_Response
{
  int8_t status;
  autonomy_interfaces__action__NavigateToPose_Result result;
} autonomy_interfaces__action__NavigateToPose_GetResult_Response;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_GetResult_Response.
typedef struct autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence
{
  autonomy_interfaces__action__NavigateToPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "autonomy_interfaces/action/detail/navigate_to_pose__struct.h"

/// Struct defined in action/NavigateToPose in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__NavigateToPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__NavigateToPose_Feedback feedback;
} autonomy_interfaces__action__NavigateToPose_FeedbackMessage;

// Struct for a sequence of autonomy_interfaces__action__NavigateToPose_FeedbackMessage.
typedef struct autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence
{
  autonomy_interfaces__action__NavigateToPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__NavigateToPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__STRUCT_H_
