// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:action/ExecuteMission.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/action/execute_mission.h"


#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__STRUCT_H_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mission_type'
// Member 'mission_id'
// Member 'waypoints'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_Goal
{
  /// Goal: Mission parameters
  /// Type of mission (e.g., "sample_collection", "scouting")
  rosidl_runtime_c__String mission_type;
  /// Unique mission identifier
  rosidl_runtime_c__String mission_id;
  /// List of waypoints to visit (optional)
  rosidl_runtime_c__String__Sequence waypoints;
  /// Maximum mission time (seconds, 0 = no limit)
  float timeout;
} autonomy_interfaces__action__ExecuteMission_Goal;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_Goal.
typedef struct autonomy_interfaces__action__ExecuteMission_Goal__Sequence
{
  autonomy_interfaces__action__ExecuteMission_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'current_phase'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_Result
{
  /// Current mission phase
  rosidl_runtime_c__String current_phase;
  /// Overall progress (0.0 to 1.0)
  float progress;
  /// Human-readable status
  rosidl_runtime_c__String status_message;
  /// Number of waypoints completed
  int32_t waypoints_completed;
  /// Seconds until completion
  float estimated_time_remaining;
} autonomy_interfaces__action__ExecuteMission_Result;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_Result.
typedef struct autonomy_interfaces__action__ExecuteMission_Result__Sequence
{
  autonomy_interfaces__action__ExecuteMission_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'completion_status'
// Member 'completed_tasks'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_Feedback
{
  bool success;
  /// Detailed completion message
  rosidl_runtime_c__String completion_status;
  /// List of tasks that were completed
  rosidl_runtime_c__String__Sequence completed_tasks;
  /// Total mission time in seconds
  float total_time;
  /// Total waypoints visited
  int32_t waypoints_visited;
} autonomy_interfaces__action__ExecuteMission_Feedback;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_Feedback.
typedef struct autonomy_interfaces__action__ExecuteMission_Feedback__Sequence
{
  autonomy_interfaces__action__ExecuteMission_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "autonomy_interfaces/action/detail/execute_mission__struct.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__ExecuteMission_Goal goal;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Request;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_SendGoal_Request.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Response;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_SendGoal_Response.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__action__ExecuteMission_SendGoal_Request__Sequence request;
  autonomy_interfaces__action__ExecuteMission_SendGoal_Response__Sequence response;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Event;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_SendGoal_Event.
typedef struct autonomy_interfaces__action__ExecuteMission_SendGoal_Event__Sequence
{
  autonomy_interfaces__action__ExecuteMission_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} autonomy_interfaces__action__ExecuteMission_GetResult_Request;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_GetResult_Request.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Response
{
  int8_t status;
  autonomy_interfaces__action__ExecuteMission_Result result;
} autonomy_interfaces__action__ExecuteMission_GetResult_Response;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_GetResult_Response.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__action__ExecuteMission_GetResult_Request__Sequence request;
  autonomy_interfaces__action__ExecuteMission_GetResult_Response__Sequence response;
} autonomy_interfaces__action__ExecuteMission_GetResult_Event;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_GetResult_Event.
typedef struct autonomy_interfaces__action__ExecuteMission_GetResult_Event__Sequence
{
  autonomy_interfaces__action__ExecuteMission_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "autonomy_interfaces/action/detail/execute_mission__struct.h"

/// Struct defined in action/ExecuteMission in the package autonomy_interfaces.
typedef struct autonomy_interfaces__action__ExecuteMission_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  autonomy_interfaces__action__ExecuteMission_Feedback feedback;
} autonomy_interfaces__action__ExecuteMission_FeedbackMessage;

// Struct for a sequence of autonomy_interfaces__action__ExecuteMission_FeedbackMessage.
typedef struct autonomy_interfaces__action__ExecuteMission_FeedbackMessage__Sequence
{
  autonomy_interfaces__action__ExecuteMission_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__action__ExecuteMission_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__STRUCT_H_
