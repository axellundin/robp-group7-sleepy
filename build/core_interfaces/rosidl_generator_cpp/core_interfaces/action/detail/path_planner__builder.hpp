// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core_interfaces:action/PathPlanner.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/action/path_planner.hpp"


#ifndef CORE_INTERFACES__ACTION__DETAIL__PATH_PLANNER__BUILDER_HPP_
#define CORE_INTERFACES__ACTION__DETAIL__PATH_PLANNER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core_interfaces/action/detail/path_planner__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_Goal_goal_pose
{
public:
  Init_PathPlanner_Goal_goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::PathPlanner_Goal goal_pose(::core_interfaces::action::PathPlanner_Goal::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_Goal>()
{
  return core_interfaces::action::builder::Init_PathPlanner_Goal_goal_pose();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_Result_success
{
public:
  Init_PathPlanner_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::PathPlanner_Result success(::core_interfaces::action::PathPlanner_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_Result>()
{
  return core_interfaces::action::builder::Init_PathPlanner_Result_success();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_Feedback_progress
{
public:
  Init_PathPlanner_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::PathPlanner_Feedback progress(::core_interfaces::action::PathPlanner_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_Feedback>()
{
  return core_interfaces::action::builder::Init_PathPlanner_Feedback_progress();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_SendGoal_Request_goal
{
public:
  explicit Init_PathPlanner_SendGoal_Request_goal(::core_interfaces::action::PathPlanner_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_SendGoal_Request goal(::core_interfaces::action::PathPlanner_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Request msg_;
};

class Init_PathPlanner_SendGoal_Request_goal_id
{
public:
  Init_PathPlanner_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_SendGoal_Request_goal goal_id(::core_interfaces::action::PathPlanner_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PathPlanner_SendGoal_Request_goal(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_SendGoal_Request>()
{
  return core_interfaces::action::builder::Init_PathPlanner_SendGoal_Request_goal_id();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_SendGoal_Response_stamp
{
public:
  explicit Init_PathPlanner_SendGoal_Response_stamp(::core_interfaces::action::PathPlanner_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_SendGoal_Response stamp(::core_interfaces::action::PathPlanner_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Response msg_;
};

class Init_PathPlanner_SendGoal_Response_accepted
{
public:
  Init_PathPlanner_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_SendGoal_Response_stamp accepted(::core_interfaces::action::PathPlanner_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PathPlanner_SendGoal_Response_stamp(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_SendGoal_Response>()
{
  return core_interfaces::action::builder::Init_PathPlanner_SendGoal_Response_accepted();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_SendGoal_Event_response
{
public:
  explicit Init_PathPlanner_SendGoal_Event_response(::core_interfaces::action::PathPlanner_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_SendGoal_Event response(::core_interfaces::action::PathPlanner_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Event msg_;
};

class Init_PathPlanner_SendGoal_Event_request
{
public:
  explicit Init_PathPlanner_SendGoal_Event_request(::core_interfaces::action::PathPlanner_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_PathPlanner_SendGoal_Event_response request(::core_interfaces::action::PathPlanner_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PathPlanner_SendGoal_Event_response(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Event msg_;
};

class Init_PathPlanner_SendGoal_Event_info
{
public:
  Init_PathPlanner_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_SendGoal_Event_request info(::core_interfaces::action::PathPlanner_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PathPlanner_SendGoal_Event_request(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_SendGoal_Event>()
{
  return core_interfaces::action::builder::Init_PathPlanner_SendGoal_Event_info();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_GetResult_Request_goal_id
{
public:
  Init_PathPlanner_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::PathPlanner_GetResult_Request goal_id(::core_interfaces::action::PathPlanner_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_GetResult_Request>()
{
  return core_interfaces::action::builder::Init_PathPlanner_GetResult_Request_goal_id();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_GetResult_Response_result
{
public:
  explicit Init_PathPlanner_GetResult_Response_result(::core_interfaces::action::PathPlanner_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_GetResult_Response result(::core_interfaces::action::PathPlanner_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Response msg_;
};

class Init_PathPlanner_GetResult_Response_status
{
public:
  Init_PathPlanner_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_GetResult_Response_result status(::core_interfaces::action::PathPlanner_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PathPlanner_GetResult_Response_result(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_GetResult_Response>()
{
  return core_interfaces::action::builder::Init_PathPlanner_GetResult_Response_status();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_GetResult_Event_response
{
public:
  explicit Init_PathPlanner_GetResult_Event_response(::core_interfaces::action::PathPlanner_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_GetResult_Event response(::core_interfaces::action::PathPlanner_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Event msg_;
};

class Init_PathPlanner_GetResult_Event_request
{
public:
  explicit Init_PathPlanner_GetResult_Event_request(::core_interfaces::action::PathPlanner_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_PathPlanner_GetResult_Event_response request(::core_interfaces::action::PathPlanner_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PathPlanner_GetResult_Event_response(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Event msg_;
};

class Init_PathPlanner_GetResult_Event_info
{
public:
  Init_PathPlanner_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_GetResult_Event_request info(::core_interfaces::action::PathPlanner_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PathPlanner_GetResult_Event_request(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_GetResult_Event>()
{
  return core_interfaces::action::builder::Init_PathPlanner_GetResult_Event_info();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_PathPlanner_FeedbackMessage_feedback
{
public:
  explicit Init_PathPlanner_FeedbackMessage_feedback(::core_interfaces::action::PathPlanner_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::PathPlanner_FeedbackMessage feedback(::core_interfaces::action::PathPlanner_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_FeedbackMessage msg_;
};

class Init_PathPlanner_FeedbackMessage_goal_id
{
public:
  Init_PathPlanner_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlanner_FeedbackMessage_feedback goal_id(::core_interfaces::action::PathPlanner_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PathPlanner_FeedbackMessage_feedback(msg_);
  }

private:
  ::core_interfaces::action::PathPlanner_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::PathPlanner_FeedbackMessage>()
{
  return core_interfaces::action::builder::Init_PathPlanner_FeedbackMessage_goal_id();
}

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__ACTION__DETAIL__PATH_PLANNER__BUILDER_HPP_
