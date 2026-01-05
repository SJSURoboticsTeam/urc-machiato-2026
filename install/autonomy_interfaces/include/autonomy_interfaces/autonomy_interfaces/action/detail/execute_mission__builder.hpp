// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:action/ExecuteMission.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/action/execute_mission.hpp"


#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__BUILDER_HPP_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/action/detail/execute_mission__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_Goal_timeout
{
public:
  explicit Init_ExecuteMission_Goal_timeout(::autonomy_interfaces::action::ExecuteMission_Goal & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_Goal timeout(::autonomy_interfaces::action::ExecuteMission_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Goal msg_;
};

class Init_ExecuteMission_Goal_waypoints
{
public:
  explicit Init_ExecuteMission_Goal_waypoints(::autonomy_interfaces::action::ExecuteMission_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Goal_timeout waypoints(::autonomy_interfaces::action::ExecuteMission_Goal::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return Init_ExecuteMission_Goal_timeout(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Goal msg_;
};

class Init_ExecuteMission_Goal_mission_id
{
public:
  explicit Init_ExecuteMission_Goal_mission_id(::autonomy_interfaces::action::ExecuteMission_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Goal_waypoints mission_id(::autonomy_interfaces::action::ExecuteMission_Goal::_mission_id_type arg)
  {
    msg_.mission_id = std::move(arg);
    return Init_ExecuteMission_Goal_waypoints(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Goal msg_;
};

class Init_ExecuteMission_Goal_mission_type
{
public:
  Init_ExecuteMission_Goal_mission_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_Goal_mission_id mission_type(::autonomy_interfaces::action::ExecuteMission_Goal::_mission_type_type arg)
  {
    msg_.mission_type = std::move(arg);
    return Init_ExecuteMission_Goal_mission_id(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_Goal>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_Goal_mission_type();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_Result_estimated_time_remaining
{
public:
  explicit Init_ExecuteMission_Result_estimated_time_remaining(::autonomy_interfaces::action::ExecuteMission_Result & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_Result estimated_time_remaining(::autonomy_interfaces::action::ExecuteMission_Result::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Result msg_;
};

class Init_ExecuteMission_Result_waypoints_completed
{
public:
  explicit Init_ExecuteMission_Result_waypoints_completed(::autonomy_interfaces::action::ExecuteMission_Result & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Result_estimated_time_remaining waypoints_completed(::autonomy_interfaces::action::ExecuteMission_Result::_waypoints_completed_type arg)
  {
    msg_.waypoints_completed = std::move(arg);
    return Init_ExecuteMission_Result_estimated_time_remaining(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Result msg_;
};

class Init_ExecuteMission_Result_status_message
{
public:
  explicit Init_ExecuteMission_Result_status_message(::autonomy_interfaces::action::ExecuteMission_Result & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Result_waypoints_completed status_message(::autonomy_interfaces::action::ExecuteMission_Result::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_ExecuteMission_Result_waypoints_completed(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Result msg_;
};

class Init_ExecuteMission_Result_progress
{
public:
  explicit Init_ExecuteMission_Result_progress(::autonomy_interfaces::action::ExecuteMission_Result & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Result_status_message progress(::autonomy_interfaces::action::ExecuteMission_Result::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_ExecuteMission_Result_status_message(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Result msg_;
};

class Init_ExecuteMission_Result_current_phase
{
public:
  Init_ExecuteMission_Result_current_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_Result_progress current_phase(::autonomy_interfaces::action::ExecuteMission_Result::_current_phase_type arg)
  {
    msg_.current_phase = std::move(arg);
    return Init_ExecuteMission_Result_progress(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_Result>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_Result_current_phase();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_Feedback_waypoints_visited
{
public:
  explicit Init_ExecuteMission_Feedback_waypoints_visited(::autonomy_interfaces::action::ExecuteMission_Feedback & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_Feedback waypoints_visited(::autonomy_interfaces::action::ExecuteMission_Feedback::_waypoints_visited_type arg)
  {
    msg_.waypoints_visited = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Feedback msg_;
};

class Init_ExecuteMission_Feedback_total_time
{
public:
  explicit Init_ExecuteMission_Feedback_total_time(::autonomy_interfaces::action::ExecuteMission_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Feedback_waypoints_visited total_time(::autonomy_interfaces::action::ExecuteMission_Feedback::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return Init_ExecuteMission_Feedback_waypoints_visited(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Feedback msg_;
};

class Init_ExecuteMission_Feedback_completed_tasks
{
public:
  explicit Init_ExecuteMission_Feedback_completed_tasks(::autonomy_interfaces::action::ExecuteMission_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Feedback_total_time completed_tasks(::autonomy_interfaces::action::ExecuteMission_Feedback::_completed_tasks_type arg)
  {
    msg_.completed_tasks = std::move(arg);
    return Init_ExecuteMission_Feedback_total_time(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Feedback msg_;
};

class Init_ExecuteMission_Feedback_completion_status
{
public:
  explicit Init_ExecuteMission_Feedback_completion_status(::autonomy_interfaces::action::ExecuteMission_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_Feedback_completed_tasks completion_status(::autonomy_interfaces::action::ExecuteMission_Feedback::_completion_status_type arg)
  {
    msg_.completion_status = std::move(arg);
    return Init_ExecuteMission_Feedback_completed_tasks(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Feedback msg_;
};

class Init_ExecuteMission_Feedback_success
{
public:
  Init_ExecuteMission_Feedback_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_Feedback_completion_status success(::autonomy_interfaces::action::ExecuteMission_Feedback::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecuteMission_Feedback_completion_status(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_Feedback>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_Feedback_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_SendGoal_Request_goal
{
public:
  explicit Init_ExecuteMission_SendGoal_Request_goal(::autonomy_interfaces::action::ExecuteMission_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Request goal(::autonomy_interfaces::action::ExecuteMission_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Request msg_;
};

class Init_ExecuteMission_SendGoal_Request_goal_id
{
public:
  Init_ExecuteMission_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_SendGoal_Request_goal goal_id(::autonomy_interfaces::action::ExecuteMission_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteMission_SendGoal_Request_goal(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_SendGoal_Request>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_SendGoal_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_SendGoal_Response_stamp
{
public:
  explicit Init_ExecuteMission_SendGoal_Response_stamp(::autonomy_interfaces::action::ExecuteMission_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Response stamp(::autonomy_interfaces::action::ExecuteMission_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Response msg_;
};

class Init_ExecuteMission_SendGoal_Response_accepted
{
public:
  Init_ExecuteMission_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_SendGoal_Response_stamp accepted(::autonomy_interfaces::action::ExecuteMission_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExecuteMission_SendGoal_Response_stamp(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_SendGoal_Response>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_SendGoal_Response_accepted();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_SendGoal_Event_response
{
public:
  explicit Init_ExecuteMission_SendGoal_Event_response(::autonomy_interfaces::action::ExecuteMission_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Event response(::autonomy_interfaces::action::ExecuteMission_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Event msg_;
};

class Init_ExecuteMission_SendGoal_Event_request
{
public:
  explicit Init_ExecuteMission_SendGoal_Event_request(::autonomy_interfaces::action::ExecuteMission_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_SendGoal_Event_response request(::autonomy_interfaces::action::ExecuteMission_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExecuteMission_SendGoal_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Event msg_;
};

class Init_ExecuteMission_SendGoal_Event_info
{
public:
  Init_ExecuteMission_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_SendGoal_Event_request info(::autonomy_interfaces::action::ExecuteMission_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExecuteMission_SendGoal_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_SendGoal_Event>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_SendGoal_Event_info();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_GetResult_Request_goal_id
{
public:
  Init_ExecuteMission_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Request goal_id(::autonomy_interfaces::action::ExecuteMission_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_GetResult_Request>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_GetResult_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_GetResult_Response_result
{
public:
  explicit Init_ExecuteMission_GetResult_Response_result(::autonomy_interfaces::action::ExecuteMission_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Response result(::autonomy_interfaces::action::ExecuteMission_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Response msg_;
};

class Init_ExecuteMission_GetResult_Response_status
{
public:
  Init_ExecuteMission_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_GetResult_Response_result status(::autonomy_interfaces::action::ExecuteMission_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExecuteMission_GetResult_Response_result(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_GetResult_Response>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_GetResult_Response_status();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_GetResult_Event_response
{
public:
  explicit Init_ExecuteMission_GetResult_Event_response(::autonomy_interfaces::action::ExecuteMission_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Event response(::autonomy_interfaces::action::ExecuteMission_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Event msg_;
};

class Init_ExecuteMission_GetResult_Event_request
{
public:
  explicit Init_ExecuteMission_GetResult_Event_request(::autonomy_interfaces::action::ExecuteMission_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_ExecuteMission_GetResult_Event_response request(::autonomy_interfaces::action::ExecuteMission_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExecuteMission_GetResult_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Event msg_;
};

class Init_ExecuteMission_GetResult_Event_info
{
public:
  Init_ExecuteMission_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_GetResult_Event_request info(::autonomy_interfaces::action::ExecuteMission_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExecuteMission_GetResult_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_GetResult_Event>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_GetResult_Event_info();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMission_FeedbackMessage_feedback
{
public:
  explicit Init_ExecuteMission_FeedbackMessage_feedback(::autonomy_interfaces::action::ExecuteMission_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::ExecuteMission_FeedbackMessage feedback(::autonomy_interfaces::action::ExecuteMission_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_FeedbackMessage msg_;
};

class Init_ExecuteMission_FeedbackMessage_goal_id
{
public:
  Init_ExecuteMission_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMission_FeedbackMessage_feedback goal_id(::autonomy_interfaces::action::ExecuteMission_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteMission_FeedbackMessage_feedback(msg_);
  }

private:
  ::autonomy_interfaces::action::ExecuteMission_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::ExecuteMission_FeedbackMessage>()
{
  return autonomy_interfaces::action::builder::Init_ExecuteMission_FeedbackMessage_goal_id();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__EXECUTE_MISSION__BUILDER_HPP_
