// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from explorer_interfaces:action/Wander.idl
// generated code does not contain a copyright notice

#ifndef EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__STRUCT_H_
#define EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_Goal
{
  /// Strategy to use to explore the map:
  ///    0=Random control
  ///    1=Wall hugging
  ///    ...
  uint8_t strategy;
  /// Threshold to consider the map completed
  float map_completed_thres;
} explorer_interfaces__action__Wander_Goal;

// Struct for a sequence of explorer_interfaces__action__Wander_Goal.
typedef struct explorer_interfaces__action__Wander_Goal__Sequence
{
  explorer_interfaces__action__Wander_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_Result
{
  uint8_t structure_needs_at_least_one_member;
} explorer_interfaces__action__Wander_Result;

// Struct for a sequence of explorer_interfaces__action__Wander_Result.
typedef struct explorer_interfaces__action__Wander_Result__Sequence
{
  explorer_interfaces__action__Wander_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_Feedback
{
  float progress;
} explorer_interfaces__action__Wander_Feedback;

// Struct for a sequence of explorer_interfaces__action__Wander_Feedback.
typedef struct explorer_interfaces__action__Wander_Feedback__Sequence
{
  explorer_interfaces__action__Wander_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "explorer_interfaces/action/detail/wander__struct.h"

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  explorer_interfaces__action__Wander_Goal goal;
} explorer_interfaces__action__Wander_SendGoal_Request;

// Struct for a sequence of explorer_interfaces__action__Wander_SendGoal_Request.
typedef struct explorer_interfaces__action__Wander_SendGoal_Request__Sequence
{
  explorer_interfaces__action__Wander_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} explorer_interfaces__action__Wander_SendGoal_Response;

// Struct for a sequence of explorer_interfaces__action__Wander_SendGoal_Response.
typedef struct explorer_interfaces__action__Wander_SendGoal_Response__Sequence
{
  explorer_interfaces__action__Wander_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} explorer_interfaces__action__Wander_GetResult_Request;

// Struct for a sequence of explorer_interfaces__action__Wander_GetResult_Request.
typedef struct explorer_interfaces__action__Wander_GetResult_Request__Sequence
{
  explorer_interfaces__action__Wander_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_GetResult_Response
{
  int8_t status;
  explorer_interfaces__action__Wander_Result result;
} explorer_interfaces__action__Wander_GetResult_Response;

// Struct for a sequence of explorer_interfaces__action__Wander_GetResult_Response.
typedef struct explorer_interfaces__action__Wander_GetResult_Response__Sequence
{
  explorer_interfaces__action__Wander_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"

/// Struct defined in action/Wander in the package explorer_interfaces.
typedef struct explorer_interfaces__action__Wander_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  explorer_interfaces__action__Wander_Feedback feedback;
} explorer_interfaces__action__Wander_FeedbackMessage;

// Struct for a sequence of explorer_interfaces__action__Wander_FeedbackMessage.
typedef struct explorer_interfaces__action__Wander_FeedbackMessage__Sequence
{
  explorer_interfaces__action__Wander_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explorer_interfaces__action__Wander_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXPLORER_INTERFACES__ACTION__DETAIL__WANDER__STRUCT_H_
