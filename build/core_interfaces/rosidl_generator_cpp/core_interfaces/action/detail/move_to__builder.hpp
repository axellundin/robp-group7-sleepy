// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core_interfaces:action/MoveTo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/action/move_to.hpp"


#ifndef CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__BUILDER_HPP_
#define CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core_interfaces/action/detail/move_to__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_Goal_stop_at_goal
{
public:
  explicit Init_MoveTo_Goal_stop_at_goal(::core_interfaces::action::MoveTo_Goal & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_Goal stop_at_goal(::core_interfaces::action::MoveTo_Goal::_stop_at_goal_type arg)
  {
    msg_.stop_at_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_Goal msg_;
};

class Init_MoveTo_Goal_enforce_orientation
{
public:
  explicit Init_MoveTo_Goal_enforce_orientation(::core_interfaces::action::MoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveTo_Goal_stop_at_goal enforce_orientation(::core_interfaces::action::MoveTo_Goal::_enforce_orientation_type arg)
  {
    msg_.enforce_orientation = std::move(arg);
    return Init_MoveTo_Goal_stop_at_goal(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_Goal msg_;
};

class Init_MoveTo_Goal_way_point
{
public:
  Init_MoveTo_Goal_way_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_Goal_enforce_orientation way_point(::core_interfaces::action::MoveTo_Goal::_way_point_type arg)
  {
    msg_.way_point = std::move(arg);
    return Init_MoveTo_Goal_enforce_orientation(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_Goal>()
{
  return core_interfaces::action::builder::Init_MoveTo_Goal_way_point();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_Result_success
{
public:
  Init_MoveTo_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::MoveTo_Result success(::core_interfaces::action::MoveTo_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_Result>()
{
  return core_interfaces::action::builder::Init_MoveTo_Result_success();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_Feedback_distance_to_goal
{
public:
  Init_MoveTo_Feedback_distance_to_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::MoveTo_Feedback distance_to_goal(::core_interfaces::action::MoveTo_Feedback::_distance_to_goal_type arg)
  {
    msg_.distance_to_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_Feedback>()
{
  return core_interfaces::action::builder::Init_MoveTo_Feedback_distance_to_goal();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_SendGoal_Request_goal
{
public:
  explicit Init_MoveTo_SendGoal_Request_goal(::core_interfaces::action::MoveTo_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_SendGoal_Request goal(::core_interfaces::action::MoveTo_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Request msg_;
};

class Init_MoveTo_SendGoal_Request_goal_id
{
public:
  Init_MoveTo_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_SendGoal_Request_goal goal_id(::core_interfaces::action::MoveTo_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveTo_SendGoal_Request_goal(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_SendGoal_Request>()
{
  return core_interfaces::action::builder::Init_MoveTo_SendGoal_Request_goal_id();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_SendGoal_Response_stamp
{
public:
  explicit Init_MoveTo_SendGoal_Response_stamp(::core_interfaces::action::MoveTo_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_SendGoal_Response stamp(::core_interfaces::action::MoveTo_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Response msg_;
};

class Init_MoveTo_SendGoal_Response_accepted
{
public:
  Init_MoveTo_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_SendGoal_Response_stamp accepted(::core_interfaces::action::MoveTo_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveTo_SendGoal_Response_stamp(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_SendGoal_Response>()
{
  return core_interfaces::action::builder::Init_MoveTo_SendGoal_Response_accepted();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_SendGoal_Event_response
{
public:
  explicit Init_MoveTo_SendGoal_Event_response(::core_interfaces::action::MoveTo_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_SendGoal_Event response(::core_interfaces::action::MoveTo_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Event msg_;
};

class Init_MoveTo_SendGoal_Event_request
{
public:
  explicit Init_MoveTo_SendGoal_Event_request(::core_interfaces::action::MoveTo_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_MoveTo_SendGoal_Event_response request(::core_interfaces::action::MoveTo_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveTo_SendGoal_Event_response(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Event msg_;
};

class Init_MoveTo_SendGoal_Event_info
{
public:
  Init_MoveTo_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_SendGoal_Event_request info(::core_interfaces::action::MoveTo_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveTo_SendGoal_Event_request(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_SendGoal_Event>()
{
  return core_interfaces::action::builder::Init_MoveTo_SendGoal_Event_info();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_GetResult_Request_goal_id
{
public:
  Init_MoveTo_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::action::MoveTo_GetResult_Request goal_id(::core_interfaces::action::MoveTo_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_GetResult_Request>()
{
  return core_interfaces::action::builder::Init_MoveTo_GetResult_Request_goal_id();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_GetResult_Response_result
{
public:
  explicit Init_MoveTo_GetResult_Response_result(::core_interfaces::action::MoveTo_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_GetResult_Response result(::core_interfaces::action::MoveTo_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Response msg_;
};

class Init_MoveTo_GetResult_Response_status
{
public:
  Init_MoveTo_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_GetResult_Response_result status(::core_interfaces::action::MoveTo_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveTo_GetResult_Response_result(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_GetResult_Response>()
{
  return core_interfaces::action::builder::Init_MoveTo_GetResult_Response_status();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_GetResult_Event_response
{
public:
  explicit Init_MoveTo_GetResult_Event_response(::core_interfaces::action::MoveTo_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_GetResult_Event response(::core_interfaces::action::MoveTo_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Event msg_;
};

class Init_MoveTo_GetResult_Event_request
{
public:
  explicit Init_MoveTo_GetResult_Event_request(::core_interfaces::action::MoveTo_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_MoveTo_GetResult_Event_response request(::core_interfaces::action::MoveTo_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveTo_GetResult_Event_response(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Event msg_;
};

class Init_MoveTo_GetResult_Event_info
{
public:
  Init_MoveTo_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_GetResult_Event_request info(::core_interfaces::action::MoveTo_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveTo_GetResult_Event_request(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_GetResult_Event>()
{
  return core_interfaces::action::builder::Init_MoveTo_GetResult_Event_info();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveTo_FeedbackMessage_feedback
{
public:
  explicit Init_MoveTo_FeedbackMessage_feedback(::core_interfaces::action::MoveTo_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::core_interfaces::action::MoveTo_FeedbackMessage feedback(::core_interfaces::action::MoveTo_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_FeedbackMessage msg_;
};

class Init_MoveTo_FeedbackMessage_goal_id
{
public:
  Init_MoveTo_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTo_FeedbackMessage_feedback goal_id(::core_interfaces::action::MoveTo_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveTo_FeedbackMessage_feedback(msg_);
  }

private:
  ::core_interfaces::action::MoveTo_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::action::MoveTo_FeedbackMessage>()
{
  return core_interfaces::action::builder::Init_MoveTo_FeedbackMessage_goal_id();
}

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__BUILDER_HPP_
