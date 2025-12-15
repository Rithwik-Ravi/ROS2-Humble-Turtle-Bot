// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from explorer_interfaces:action/Wander.idl
// generated code does not contain a copyright notice

#ifndef EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__TRAITS_HPP_
#define EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "explorer_interfaces/action/detail/wander__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: strategy
  {
    out << "strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.strategy, out);
    out << ", ";
  }

  // member: map_completed_thres
  {
    out << "map_completed_thres: ";
    rosidl_generator_traits::value_to_yaml(msg.map_completed_thres, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: strategy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.strategy, out);
    out << "\n";
  }

  // member: map_completed_thres
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_completed_thres: ";
    rosidl_generator_traits::value_to_yaml(msg.map_completed_thres, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_Goal & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_Goal>()
{
  return "explorer_interfaces::action::Wander_Goal";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_Goal>()
{
  return "explorer_interfaces/action/Wander_Goal";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<explorer_interfaces::action::Wander_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_Result & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_Result & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_Result>()
{
  return "explorer_interfaces::action::Wander_Result";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_Result>()
{
  return "explorer_interfaces/action/Wander_Result";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<explorer_interfaces::action::Wander_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: progress
  {
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_Feedback & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_Feedback>()
{
  return "explorer_interfaces::action::Wander_Feedback";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_Feedback>()
{
  return "explorer_interfaces/action/Wander_Feedback";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<explorer_interfaces::action::Wander_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "explorer_interfaces/action/detail/wander__traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_SendGoal_Request & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_SendGoal_Request>()
{
  return "explorer_interfaces::action::Wander_SendGoal_Request";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_SendGoal_Request>()
{
  return "explorer_interfaces/action/Wander_SendGoal_Request";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<explorer_interfaces::action::Wander_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<explorer_interfaces::action::Wander_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<explorer_interfaces::action::Wander_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_SendGoal_Response & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_SendGoal_Response>()
{
  return "explorer_interfaces::action::Wander_SendGoal_Response";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_SendGoal_Response>()
{
  return "explorer_interfaces/action/Wander_SendGoal_Response";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<explorer_interfaces::action::Wander_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<explorer_interfaces::action::Wander_SendGoal>()
{
  return "explorer_interfaces::action::Wander_SendGoal";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_SendGoal>()
{
  return "explorer_interfaces/action/Wander_SendGoal";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<explorer_interfaces::action::Wander_SendGoal_Request>::value &&
    has_fixed_size<explorer_interfaces::action::Wander_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<explorer_interfaces::action::Wander_SendGoal_Request>::value &&
    has_bounded_size<explorer_interfaces::action::Wander_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<explorer_interfaces::action::Wander_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<explorer_interfaces::action::Wander_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<explorer_interfaces::action::Wander_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_GetResult_Request & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_GetResult_Request>()
{
  return "explorer_interfaces::action::Wander_GetResult_Request";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_GetResult_Request>()
{
  return "explorer_interfaces/action/Wander_GetResult_Request";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<explorer_interfaces::action::Wander_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "explorer_interfaces/action/detail/wander__traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_GetResult_Response & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_GetResult_Response>()
{
  return "explorer_interfaces::action::Wander_GetResult_Response";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_GetResult_Response>()
{
  return "explorer_interfaces/action/Wander_GetResult_Response";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<explorer_interfaces::action::Wander_Result>::value> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<explorer_interfaces::action::Wander_Result>::value> {};

template<>
struct is_message<explorer_interfaces::action::Wander_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<explorer_interfaces::action::Wander_GetResult>()
{
  return "explorer_interfaces::action::Wander_GetResult";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_GetResult>()
{
  return "explorer_interfaces/action/Wander_GetResult";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<explorer_interfaces::action::Wander_GetResult_Request>::value &&
    has_fixed_size<explorer_interfaces::action::Wander_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<explorer_interfaces::action::Wander_GetResult_Request>::value &&
    has_bounded_size<explorer_interfaces::action::Wander_GetResult_Response>::value
  >
{
};

template<>
struct is_service<explorer_interfaces::action::Wander_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<explorer_interfaces::action::Wander_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<explorer_interfaces::action::Wander_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "explorer_interfaces/action/detail/wander__traits.hpp"

namespace explorer_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Wander_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wander_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wander_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace explorer_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use explorer_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explorer_interfaces::action::Wander_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  explorer_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explorer_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const explorer_interfaces::action::Wander_FeedbackMessage & msg)
{
  return explorer_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<explorer_interfaces::action::Wander_FeedbackMessage>()
{
  return "explorer_interfaces::action::Wander_FeedbackMessage";
}

template<>
inline const char * name<explorer_interfaces::action::Wander_FeedbackMessage>()
{
  return "explorer_interfaces/action/Wander_FeedbackMessage";
}

template<>
struct has_fixed_size<explorer_interfaces::action::Wander_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<explorer_interfaces::action::Wander_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<explorer_interfaces::action::Wander_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<explorer_interfaces::action::Wander_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<explorer_interfaces::action::Wander_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<explorer_interfaces::action::Wander>
  : std::true_type
{
};

template<>
struct is_action_goal<explorer_interfaces::action::Wander_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<explorer_interfaces::action::Wander_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<explorer_interfaces::action::Wander_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__TRAITS_HPP_
