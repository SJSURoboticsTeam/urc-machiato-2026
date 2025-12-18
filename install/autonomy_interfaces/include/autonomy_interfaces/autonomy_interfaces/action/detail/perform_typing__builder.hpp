// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:action/PerformTyping.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__BUILDER_HPP_
#define AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/action/detail/perform_typing__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_Goal_typing_goal
{
public:
  Init_PerformTyping_Goal_typing_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::action::PerformTyping_Goal typing_goal(::autonomy_interfaces::action::PerformTyping_Goal::_typing_goal_type arg)
  {
    msg_.typing_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_Goal>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_Goal_typing_goal();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_Result_current_hand_pose
{
public:
  explicit Init_PerformTyping_Result_current_hand_pose(::autonomy_interfaces::action::PerformTyping_Result & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_Result current_hand_pose(::autonomy_interfaces::action::PerformTyping_Result::_current_hand_pose_type arg)
  {
    msg_.current_hand_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Result msg_;
};

class Init_PerformTyping_Result_characters_completed
{
public:
  explicit Init_PerformTyping_Result_characters_completed(::autonomy_interfaces::action::PerformTyping_Result & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Result_current_hand_pose characters_completed(::autonomy_interfaces::action::PerformTyping_Result::_characters_completed_type arg)
  {
    msg_.characters_completed = std::move(arg);
    return Init_PerformTyping_Result_current_hand_pose(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Result msg_;
};

class Init_PerformTyping_Result_current_character
{
public:
  explicit Init_PerformTyping_Result_current_character(::autonomy_interfaces::action::PerformTyping_Result & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Result_characters_completed current_character(::autonomy_interfaces::action::PerformTyping_Result::_current_character_type arg)
  {
    msg_.current_character = std::move(arg);
    return Init_PerformTyping_Result_characters_completed(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Result msg_;
};

class Init_PerformTyping_Result_progress
{
public:
  Init_PerformTyping_Result_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_Result_current_character progress(::autonomy_interfaces::action::PerformTyping_Result::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_PerformTyping_Result_current_character(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_Result>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_Result_progress();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_Feedback_total_time
{
public:
  explicit Init_PerformTyping_Feedback_total_time(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_Feedback total_time(::autonomy_interfaces::action::PerformTyping_Feedback::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_accuracy
{
public:
  explicit Init_PerformTyping_Feedback_accuracy(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Feedback_total_time accuracy(::autonomy_interfaces::action::PerformTyping_Feedback::_accuracy_type arg)
  {
    msg_.accuracy = std::move(arg);
    return Init_PerformTyping_Feedback_total_time(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_characters_successful
{
public:
  explicit Init_PerformTyping_Feedback_characters_successful(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Feedback_accuracy characters_successful(::autonomy_interfaces::action::PerformTyping_Feedback::_characters_successful_type arg)
  {
    msg_.characters_successful = std::move(arg);
    return Init_PerformTyping_Feedback_accuracy(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_characters_attempted
{
public:
  explicit Init_PerformTyping_Feedback_characters_attempted(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Feedback_characters_successful characters_attempted(::autonomy_interfaces::action::PerformTyping_Feedback::_characters_attempted_type arg)
  {
    msg_.characters_attempted = std::move(arg);
    return Init_PerformTyping_Feedback_characters_successful(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_text_typed
{
public:
  explicit Init_PerformTyping_Feedback_text_typed(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Feedback_characters_attempted text_typed(::autonomy_interfaces::action::PerformTyping_Feedback::_text_typed_type arg)
  {
    msg_.text_typed = std::move(arg);
    return Init_PerformTyping_Feedback_characters_attempted(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_message
{
public:
  explicit Init_PerformTyping_Feedback_message(::autonomy_interfaces::action::PerformTyping_Feedback & msg)
  : msg_(msg)
  {}
  Init_PerformTyping_Feedback_text_typed message(::autonomy_interfaces::action::PerformTyping_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_PerformTyping_Feedback_text_typed(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

class Init_PerformTyping_Feedback_success
{
public:
  Init_PerformTyping_Feedback_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_Feedback_message success(::autonomy_interfaces::action::PerformTyping_Feedback::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PerformTyping_Feedback_message(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_Feedback>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_Feedback_success();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_SendGoal_Request_goal
{
public:
  explicit Init_PerformTyping_SendGoal_Request_goal(::autonomy_interfaces::action::PerformTyping_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Request goal(::autonomy_interfaces::action::PerformTyping_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Request msg_;
};

class Init_PerformTyping_SendGoal_Request_goal_id
{
public:
  Init_PerformTyping_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_SendGoal_Request_goal goal_id(::autonomy_interfaces::action::PerformTyping_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PerformTyping_SendGoal_Request_goal(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_SendGoal_Request>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_SendGoal_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_SendGoal_Response_stamp
{
public:
  explicit Init_PerformTyping_SendGoal_Response_stamp(::autonomy_interfaces::action::PerformTyping_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Response stamp(::autonomy_interfaces::action::PerformTyping_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Response msg_;
};

class Init_PerformTyping_SendGoal_Response_accepted
{
public:
  Init_PerformTyping_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_SendGoal_Response_stamp accepted(::autonomy_interfaces::action::PerformTyping_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PerformTyping_SendGoal_Response_stamp(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_SendGoal_Response>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_SendGoal_Response_accepted();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_GetResult_Request_goal_id
{
public:
  Init_PerformTyping_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::action::PerformTyping_GetResult_Request goal_id(::autonomy_interfaces::action::PerformTyping_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_GetResult_Request>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_GetResult_Request_goal_id();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_GetResult_Response_result
{
public:
  explicit Init_PerformTyping_GetResult_Response_result(::autonomy_interfaces::action::PerformTyping_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_GetResult_Response result(::autonomy_interfaces::action::PerformTyping_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_GetResult_Response msg_;
};

class Init_PerformTyping_GetResult_Response_status
{
public:
  Init_PerformTyping_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_GetResult_Response_result status(::autonomy_interfaces::action::PerformTyping_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PerformTyping_GetResult_Response_result(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_GetResult_Response>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_GetResult_Response_status();
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace action
{

namespace builder
{

class Init_PerformTyping_FeedbackMessage_feedback
{
public:
  explicit Init_PerformTyping_FeedbackMessage_feedback(::autonomy_interfaces::action::PerformTyping_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::action::PerformTyping_FeedbackMessage feedback(::autonomy_interfaces::action::PerformTyping_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_FeedbackMessage msg_;
};

class Init_PerformTyping_FeedbackMessage_goal_id
{
public:
  Init_PerformTyping_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformTyping_FeedbackMessage_feedback goal_id(::autonomy_interfaces::action::PerformTyping_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PerformTyping_FeedbackMessage_feedback(msg_);
  }

private:
  ::autonomy_interfaces::action::PerformTyping_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::action::PerformTyping_FeedbackMessage>()
{
  return autonomy_interfaces::action::builder::Init_PerformTyping_FeedbackMessage_goal_id();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__ACTION__DETAIL__PERFORM_TYPING__BUILDER_HPP_
