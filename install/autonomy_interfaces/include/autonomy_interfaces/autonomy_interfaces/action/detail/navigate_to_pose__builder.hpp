// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:action/NavigateToPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/action/navigate_to_pose.hpp"


#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/action/detail/navigate_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_Goal_timeout
{
public:
  explicit Init_NavigateToPose_Goal_timeout(::autonomy_interfaces::action::NavigateToPose_Goal & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_Goal timeout(::autonomy_interfaces::action::NavigateToPose_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Goal msg_;
};

class Init_NavigateToPose_Goal_tolerance
{
public:
  explicit Init_NavigateToPose_Goal_tolerance(::autonomy_interfaces::action::NavigateToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Goal_timeout tolerance(::autonomy_interfaces::action::NavigateToPose_Goal::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return Init_NavigateToPose_Goal_timeout(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Goal msg_;
};

class Init_NavigateToPose_Goal_target_pose
{
public:
  Init_NavigateToPose_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_Goal_tolerance target_pose(::autonomy_interfaces::action::NavigateToPose_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_NavigateToPose_Goal_tolerance(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_Goal>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_Goal_target_pose();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_Result_current_pose
{
public:
  explicit Init_NavigateToPose_Result_current_pose(::autonomy_interfaces::action::NavigateToPose_Result & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_Result current_pose(::autonomy_interfaces::action::NavigateToPose_Result::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Result msg_;
};

class Init_NavigateToPose_Result_navigation_state
{
public:
  explicit Init_NavigateToPose_Result_navigation_state(::autonomy_interfaces::action::NavigateToPose_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Result_current_pose navigation_state(::autonomy_interfaces::action::NavigateToPose_Result::_navigation_state_type arg)
  {
    msg_.navigation_state = std::move(arg);
    return Init_NavigateToPose_Result_current_pose(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Result msg_;
};

class Init_NavigateToPose_Result_estimated_time_remaining
{
public:
  explicit Init_NavigateToPose_Result_estimated_time_remaining(::autonomy_interfaces::action::NavigateToPose_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Result_navigation_state estimated_time_remaining(::autonomy_interfaces::action::NavigateToPose_Result::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return Init_NavigateToPose_Result_navigation_state(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Result msg_;
};

class Init_NavigateToPose_Result_distance_to_goal
{
public:
  Init_NavigateToPose_Result_distance_to_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_Result_estimated_time_remaining distance_to_goal(::autonomy_interfaces::action::NavigateToPose_Result::_distance_to_goal_type arg)
  {
    msg_.distance_to_goal = std::move(arg);
    return Init_NavigateToPose_Result_estimated_time_remaining(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_Result>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_Result_distance_to_goal();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_Feedback_total_time
{
public:
  explicit Init_NavigateToPose_Feedback_total_time(::autonomy_interfaces::action::NavigateToPose_Feedback & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_Feedback total_time(::autonomy_interfaces::action::NavigateToPose_Feedback::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Feedback msg_;
};

class Init_NavigateToPose_Feedback_total_distance_traveled
{
public:
  explicit Init_NavigateToPose_Feedback_total_distance_traveled(::autonomy_interfaces::action::NavigateToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Feedback_total_time total_distance_traveled(::autonomy_interfaces::action::NavigateToPose_Feedback::_total_distance_traveled_type arg)
  {
    msg_.total_distance_traveled = std::move(arg);
    return Init_NavigateToPose_Feedback_total_time(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Feedback msg_;
};

class Init_NavigateToPose_Feedback_final_pose
{
public:
  explicit Init_NavigateToPose_Feedback_final_pose(::autonomy_interfaces::action::NavigateToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Feedback_total_distance_traveled final_pose(::autonomy_interfaces::action::NavigateToPose_Feedback::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_NavigateToPose_Feedback_total_distance_traveled(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Feedback msg_;
};

class Init_NavigateToPose_Feedback_message
{
public:
  explicit Init_NavigateToPose_Feedback_message(::autonomy_interfaces::action::NavigateToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_Feedback_final_pose message(::autonomy_interfaces::action::NavigateToPose_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_NavigateToPose_Feedback_final_pose(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Feedback msg_;
};

class Init_NavigateToPose_Feedback_success
{
public:
  Init_NavigateToPose_Feedback_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_Feedback_message success(::autonomy_interfaces::action::NavigateToPose_Feedback::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NavigateToPose_Feedback_message(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_Feedback>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_Feedback_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_SendGoal_Request_goal
{
public:
  explicit Init_NavigateToPose_SendGoal_Request_goal(::autonomy_interfaces::action::NavigateToPose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Request goal(::autonomy_interfaces::action::NavigateToPose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Request msg_;
};

class Init_NavigateToPose_SendGoal_Request_goal_id
{
public:
  Init_NavigateToPose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_SendGoal_Request_goal goal_id(::autonomy_interfaces::action::NavigateToPose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToPose_SendGoal_Request_goal(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_SendGoal_Request>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_SendGoal_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_SendGoal_Response_stamp
{
public:
  explicit Init_NavigateToPose_SendGoal_Response_stamp(::autonomy_interfaces::action::NavigateToPose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Response stamp(::autonomy_interfaces::action::NavigateToPose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Response msg_;
};

class Init_NavigateToPose_SendGoal_Response_accepted
{
public:
  Init_NavigateToPose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_SendGoal_Response_stamp accepted(::autonomy_interfaces::action::NavigateToPose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavigateToPose_SendGoal_Response_stamp(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_SendGoal_Response>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_SendGoal_Response_accepted();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_SendGoal_Event_response
{
public:
  explicit Init_NavigateToPose_SendGoal_Event_response(::autonomy_interfaces::action::NavigateToPose_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Event response(::autonomy_interfaces::action::NavigateToPose_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Event msg_;
};

class Init_NavigateToPose_SendGoal_Event_request
{
public:
  explicit Init_NavigateToPose_SendGoal_Event_request(::autonomy_interfaces::action::NavigateToPose_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_SendGoal_Event_response request(::autonomy_interfaces::action::NavigateToPose_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_NavigateToPose_SendGoal_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Event msg_;
};

class Init_NavigateToPose_SendGoal_Event_info
{
public:
  Init_NavigateToPose_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_SendGoal_Event_request info(::autonomy_interfaces::action::NavigateToPose_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_NavigateToPose_SendGoal_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_SendGoal_Event>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_SendGoal_Event_info();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_GetResult_Request_goal_id
{
public:
  Init_NavigateToPose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Request goal_id(::autonomy_interfaces::action::NavigateToPose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_GetResult_Request>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_GetResult_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_GetResult_Response_result
{
public:
  explicit Init_NavigateToPose_GetResult_Response_result(::autonomy_interfaces::action::NavigateToPose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Response result(::autonomy_interfaces::action::NavigateToPose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Response msg_;
};

class Init_NavigateToPose_GetResult_Response_status
{
public:
  Init_NavigateToPose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_GetResult_Response_result status(::autonomy_interfaces::action::NavigateToPose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateToPose_GetResult_Response_result(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_GetResult_Response>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_GetResult_Response_status();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_GetResult_Event_response
{
public:
  explicit Init_NavigateToPose_GetResult_Event_response(::autonomy_interfaces::action::NavigateToPose_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Event response(::autonomy_interfaces::action::NavigateToPose_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Event msg_;
};

class Init_NavigateToPose_GetResult_Event_request
{
public:
  explicit Init_NavigateToPose_GetResult_Event_request(::autonomy_interfaces::action::NavigateToPose_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_NavigateToPose_GetResult_Event_response request(::autonomy_interfaces::action::NavigateToPose_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_NavigateToPose_GetResult_Event_response(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Event msg_;
};

class Init_NavigateToPose_GetResult_Event_info
{
public:
  Init_NavigateToPose_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_GetResult_Event_request info(::autonomy_interfaces::action::NavigateToPose_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_NavigateToPose_GetResult_Event_request(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_GetResult_Event>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_GetResult_Event_info();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToPose_FeedbackMessage_feedback
{
public:
  explicit Init_NavigateToPose_FeedbackMessage_feedback(::autonomy_interfaces::action::NavigateToPose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::NavigateToPose_FeedbackMessage feedback(::autonomy_interfaces::action::NavigateToPose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_FeedbackMessage msg_;
};

class Init_NavigateToPose_FeedbackMessage_goal_id
{
public:
  Init_NavigateToPose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToPose_FeedbackMessage_feedback goal_id(::autonomy_interfaces::action::NavigateToPose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToPose_FeedbackMessage_feedback(msg_);
  }

private:
  ::autonomy_interfaces::action::NavigateToPose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::NavigateToPose_FeedbackMessage>()
{
  return autonomy_interfaces::action::builder::Init_NavigateToPose_FeedbackMessage_goal_id();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_POSE__BUILDER_HPP_
