// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from explorer_interfaces:action/Wander.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
#include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "explorer_interfaces/action/detail/wander__functions.h"
#include "explorer_interfaces/action/detail/wander__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_Goal__init(message_memory);
}

void explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_member_array[2] = {
  {
    "strategy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_Goal, strategy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_completed_thres",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_Goal, map_completed_thres),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_Goal",  // message name
  2,  // number of fields
  sizeof(explorer_interfaces__action__Wander_Goal),
  explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_member_array,  // message members
  explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Goal)() {
  if (!explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_Goal__rosidl_typesupport_introspection_c__Wander_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_Result__init(message_memory);
}

void explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_Result, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_Result",  // message name
  1,  // number of fields
  sizeof(explorer_interfaces__action__Wander_Result),
  explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_member_array,  // message members
  explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Result)() {
  if (!explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_Result__rosidl_typesupport_introspection_c__Wander_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_Feedback__init(message_memory);
}

void explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_member_array[1] = {
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_Feedback, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_Feedback",  // message name
  1,  // number of fields
  sizeof(explorer_interfaces__action__Wander_Feedback),
  explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_member_array,  // message members
  explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Feedback)() {
  if (!explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_Feedback__rosidl_typesupport_introspection_c__Wander_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "explorer_interfaces/action/wander.h"
// Member `goal`
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_SendGoal_Request__init(message_memory);
}

void explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(explorer_interfaces__action__Wander_SendGoal_Request),
  explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_member_array,  // message members
  explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Request)() {
  explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Goal)();
  if (!explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_SendGoal_Request__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_SendGoal_Response__init(message_memory);
}

void explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(explorer_interfaces__action__Wander_SendGoal_Response),
  explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_member_array,  // message members
  explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Response)() {
  explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_SendGoal_Response__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_members = {
  "explorer_interfaces__action",  // service namespace
  "Wander_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_type_support_handle = {
  0,
  &explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal)() {
  if (!explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Response)()->data;
  }

  return &explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_GetResult_Request__init(message_memory);
}

void explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(explorer_interfaces__action__Wander_GetResult_Request),
  explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_member_array,  // message members
  explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Request)() {
  explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_GetResult_Request__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "explorer_interfaces/action/wander.h"
// Member `result`
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_GetResult_Response__init(message_memory);
}

void explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(explorer_interfaces__action__Wander_GetResult_Response),
  explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_member_array,  // message members
  explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Response)() {
  explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Result)();
  if (!explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_GetResult_Response__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_members = {
  "explorer_interfaces__action",  // service namespace
  "Wander_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_type_support_handle = {
  0,
  &explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult)() {
  if (!explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Response)()->data;
  }

  return &explorer_interfaces__action__detail__wander__rosidl_typesupport_introspection_c__Wander_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"
// already included above
// #include "explorer_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__functions.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "explorer_interfaces/action/wander.h"
// Member `feedback`
// already included above
// #include "explorer_interfaces/action/detail/wander__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  explorer_interfaces__action__Wander_FeedbackMessage__init(message_memory);
}

void explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_fini_function(void * message_memory)
{
  explorer_interfaces__action__Wander_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(explorer_interfaces__action__Wander_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_members = {
  "explorer_interfaces__action",  // message namespace
  "Wander_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(explorer_interfaces__action__Wander_FeedbackMessage),
  explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_member_array,  // message members
  explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_type_support_handle = {
  0,
  &explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_explorer_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_FeedbackMessage)() {
  explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Feedback)();
  if (!explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &explorer_interfaces__action__Wander_FeedbackMessage__rosidl_typesupport_introspection_c__Wander_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
