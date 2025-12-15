// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from explorer_interfaces:action/Wander.idl
// generated code does not contain a copyright notice

#ifndef EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__BUILDER_HPP_
#define EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "explorer_interfaces/action/detail/wander__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_Goal_map_completed_thres
{
public:
  explicit Init_Wander_Goal_map_completed_thres(::explorer_interfaces::action::Wander_Goal & msg)
  : msg_(msg)
  {}
  ::explorer_interfaces::action::Wander_Goal map_completed_thres(::explorer_interfaces::action::Wander_Goal::_map_completed_thres_type arg)
  {
    msg_.map_completed_thres = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_Goal msg_;
};

class Init_Wander_Goal_strategy
{
public:
  Init_Wander_Goal_strategy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wander_Goal_map_completed_thres strategy(::explorer_interfaces::action::Wander_Goal::_strategy_type arg)
  {
    msg_.strategy = std::move(arg);
    return Init_Wander_Goal_map_completed_thres(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_Goal>()
{
  return explorer_interfaces::action::builder::Init_Wander_Goal_strategy();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_Result>()
{
  return ::explorer_interfaces::action::Wander_Result(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_Feedback_progress
{
public:
  Init_Wander_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::explorer_interfaces::action::Wander_Feedback progress(::explorer_interfaces::action::Wander_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_Feedback>()
{
  return explorer_interfaces::action::builder::Init_Wander_Feedback_progress();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_SendGoal_Request_goal
{
public:
  explicit Init_Wander_SendGoal_Request_goal(::explorer_interfaces::action::Wander_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::explorer_interfaces::action::Wander_SendGoal_Request goal(::explorer_interfaces::action::Wander_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_SendGoal_Request msg_;
};

class Init_Wander_SendGoal_Request_goal_id
{
public:
  Init_Wander_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wander_SendGoal_Request_goal goal_id(::explorer_interfaces::action::Wander_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Wander_SendGoal_Request_goal(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_SendGoal_Request>()
{
  return explorer_interfaces::action::builder::Init_Wander_SendGoal_Request_goal_id();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_SendGoal_Response_stamp
{
public:
  explicit Init_Wander_SendGoal_Response_stamp(::explorer_interfaces::action::Wander_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::explorer_interfaces::action::Wander_SendGoal_Response stamp(::explorer_interfaces::action::Wander_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_SendGoal_Response msg_;
};

class Init_Wander_SendGoal_Response_accepted
{
public:
  Init_Wander_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wander_SendGoal_Response_stamp accepted(::explorer_interfaces::action::Wander_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Wander_SendGoal_Response_stamp(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_SendGoal_Response>()
{
  return explorer_interfaces::action::builder::Init_Wander_SendGoal_Response_accepted();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_GetResult_Request_goal_id
{
public:
  Init_Wander_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::explorer_interfaces::action::Wander_GetResult_Request goal_id(::explorer_interfaces::action::Wander_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_GetResult_Request>()
{
  return explorer_interfaces::action::builder::Init_Wander_GetResult_Request_goal_id();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_GetResult_Response_result
{
public:
  explicit Init_Wander_GetResult_Response_result(::explorer_interfaces::action::Wander_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::explorer_interfaces::action::Wander_GetResult_Response result(::explorer_interfaces::action::Wander_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_GetResult_Response msg_;
};

class Init_Wander_GetResult_Response_status
{
public:
  Init_Wander_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wander_GetResult_Response_result status(::explorer_interfaces::action::Wander_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Wander_GetResult_Response_result(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_GetResult_Response>()
{
  return explorer_interfaces::action::builder::Init_Wander_GetResult_Response_status();
}

}  // namespace explorer_interfaces


namespace explorer_interfaces
{

namespace action
{

namespace builder
{

class Init_Wander_FeedbackMessage_feedback
{
public:
  explicit Init_Wander_FeedbackMessage_feedback(::explorer_interfaces::action::Wander_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::explorer_interfaces::action::Wander_FeedbackMessage feedback(::explorer_interfaces::action::Wander_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_FeedbackMessage msg_;
};

class Init_Wander_FeedbackMessage_goal_id
{
public:
  Init_Wander_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wander_FeedbackMessage_feedback goal_id(::explorer_interfaces::action::Wander_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Wander_FeedbackMessage_feedback(msg_);
  }

private:
  ::explorer_interfaces::action::Wander_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::explorer_interfaces::action::Wander_FeedbackMessage>()
{
  return explorer_interfaces::action::builder::Init_Wander_FeedbackMessage_goal_id();
}

}  // namespace explorer_interfaces

#endif  // EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__BUILDER_HPP_
