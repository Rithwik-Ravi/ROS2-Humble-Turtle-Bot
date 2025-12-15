// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from explorer_interfaces:action/Discover.idl
// generated code does not contain a copyright notice

#ifndef EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__FUNCTIONS_H_
#define EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "explorer_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "explorer_interfaces/action/detail/discover__struct.h"

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_Goal
 * )) before or use
 * explorer_interfaces__action__Discover_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__init(explorer_interfaces__action__Discover_Goal * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Goal__fini(explorer_interfaces__action__Discover_Goal * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Goal *
explorer_interfaces__action__Discover_Goal__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Goal__destroy(explorer_interfaces__action__Discover_Goal * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__are_equal(const explorer_interfaces__action__Discover_Goal * lhs, const explorer_interfaces__action__Discover_Goal * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__copy(
  const explorer_interfaces__action__Discover_Goal * input,
  explorer_interfaces__action__Discover_Goal * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__Sequence__init(explorer_interfaces__action__Discover_Goal__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Goal__Sequence__fini(explorer_interfaces__action__Discover_Goal__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Goal__Sequence *
explorer_interfaces__action__Discover_Goal__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Goal__Sequence__destroy(explorer_interfaces__action__Discover_Goal__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__Sequence__are_equal(const explorer_interfaces__action__Discover_Goal__Sequence * lhs, const explorer_interfaces__action__Discover_Goal__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Goal__Sequence__copy(
  const explorer_interfaces__action__Discover_Goal__Sequence * input,
  explorer_interfaces__action__Discover_Goal__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_Result
 * )) before or use
 * explorer_interfaces__action__Discover_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__init(explorer_interfaces__action__Discover_Result * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Result__fini(explorer_interfaces__action__Discover_Result * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Result *
explorer_interfaces__action__Discover_Result__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Result__destroy(explorer_interfaces__action__Discover_Result * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__are_equal(const explorer_interfaces__action__Discover_Result * lhs, const explorer_interfaces__action__Discover_Result * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__copy(
  const explorer_interfaces__action__Discover_Result * input,
  explorer_interfaces__action__Discover_Result * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__Sequence__init(explorer_interfaces__action__Discover_Result__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Result__Sequence__fini(explorer_interfaces__action__Discover_Result__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Result__Sequence *
explorer_interfaces__action__Discover_Result__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Result__Sequence__destroy(explorer_interfaces__action__Discover_Result__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__Sequence__are_equal(const explorer_interfaces__action__Discover_Result__Sequence * lhs, const explorer_interfaces__action__Discover_Result__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Result__Sequence__copy(
  const explorer_interfaces__action__Discover_Result__Sequence * input,
  explorer_interfaces__action__Discover_Result__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_Feedback
 * )) before or use
 * explorer_interfaces__action__Discover_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__init(explorer_interfaces__action__Discover_Feedback * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Feedback__fini(explorer_interfaces__action__Discover_Feedback * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Feedback *
explorer_interfaces__action__Discover_Feedback__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Feedback__destroy(explorer_interfaces__action__Discover_Feedback * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__are_equal(const explorer_interfaces__action__Discover_Feedback * lhs, const explorer_interfaces__action__Discover_Feedback * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__copy(
  const explorer_interfaces__action__Discover_Feedback * input,
  explorer_interfaces__action__Discover_Feedback * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__Sequence__init(explorer_interfaces__action__Discover_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Feedback__Sequence__fini(explorer_interfaces__action__Discover_Feedback__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_Feedback__Sequence *
explorer_interfaces__action__Discover_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_Feedback__Sequence__destroy(explorer_interfaces__action__Discover_Feedback__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__Sequence__are_equal(const explorer_interfaces__action__Discover_Feedback__Sequence * lhs, const explorer_interfaces__action__Discover_Feedback__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_Feedback__Sequence__copy(
  const explorer_interfaces__action__Discover_Feedback__Sequence * input,
  explorer_interfaces__action__Discover_Feedback__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_SendGoal_Request
 * )) before or use
 * explorer_interfaces__action__Discover_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__init(explorer_interfaces__action__Discover_SendGoal_Request * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Request__fini(explorer_interfaces__action__Discover_SendGoal_Request * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_SendGoal_Request *
explorer_interfaces__action__Discover_SendGoal_Request__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Request__destroy(explorer_interfaces__action__Discover_SendGoal_Request * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__are_equal(const explorer_interfaces__action__Discover_SendGoal_Request * lhs, const explorer_interfaces__action__Discover_SendGoal_Request * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__copy(
  const explorer_interfaces__action__Discover_SendGoal_Request * input,
  explorer_interfaces__action__Discover_SendGoal_Request * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__init(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__fini(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_SendGoal_Request__Sequence *
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__destroy(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__are_equal(const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * lhs, const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__copy(
  const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * input,
  explorer_interfaces__action__Discover_SendGoal_Request__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_SendGoal_Response
 * )) before or use
 * explorer_interfaces__action__Discover_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__init(explorer_interfaces__action__Discover_SendGoal_Response * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Response__fini(explorer_interfaces__action__Discover_SendGoal_Response * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_SendGoal_Response *
explorer_interfaces__action__Discover_SendGoal_Response__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Response__destroy(explorer_interfaces__action__Discover_SendGoal_Response * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__are_equal(const explorer_interfaces__action__Discover_SendGoal_Response * lhs, const explorer_interfaces__action__Discover_SendGoal_Response * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__copy(
  const explorer_interfaces__action__Discover_SendGoal_Response * input,
  explorer_interfaces__action__Discover_SendGoal_Response * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__init(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__fini(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_SendGoal_Response__Sequence *
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__destroy(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__are_equal(const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * lhs, const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__copy(
  const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * input,
  explorer_interfaces__action__Discover_SendGoal_Response__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_GetResult_Request
 * )) before or use
 * explorer_interfaces__action__Discover_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__init(explorer_interfaces__action__Discover_GetResult_Request * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Request__fini(explorer_interfaces__action__Discover_GetResult_Request * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_GetResult_Request *
explorer_interfaces__action__Discover_GetResult_Request__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Request__destroy(explorer_interfaces__action__Discover_GetResult_Request * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__are_equal(const explorer_interfaces__action__Discover_GetResult_Request * lhs, const explorer_interfaces__action__Discover_GetResult_Request * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__copy(
  const explorer_interfaces__action__Discover_GetResult_Request * input,
  explorer_interfaces__action__Discover_GetResult_Request * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__init(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Request__Sequence__fini(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_GetResult_Request__Sequence *
explorer_interfaces__action__Discover_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Request__Sequence__destroy(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__are_equal(const explorer_interfaces__action__Discover_GetResult_Request__Sequence * lhs, const explorer_interfaces__action__Discover_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__copy(
  const explorer_interfaces__action__Discover_GetResult_Request__Sequence * input,
  explorer_interfaces__action__Discover_GetResult_Request__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_GetResult_Response
 * )) before or use
 * explorer_interfaces__action__Discover_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__init(explorer_interfaces__action__Discover_GetResult_Response * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Response__fini(explorer_interfaces__action__Discover_GetResult_Response * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_GetResult_Response *
explorer_interfaces__action__Discover_GetResult_Response__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Response__destroy(explorer_interfaces__action__Discover_GetResult_Response * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__are_equal(const explorer_interfaces__action__Discover_GetResult_Response * lhs, const explorer_interfaces__action__Discover_GetResult_Response * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__copy(
  const explorer_interfaces__action__Discover_GetResult_Response * input,
  explorer_interfaces__action__Discover_GetResult_Response * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__init(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Response__Sequence__fini(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_GetResult_Response__Sequence *
explorer_interfaces__action__Discover_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_GetResult_Response__Sequence__destroy(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__are_equal(const explorer_interfaces__action__Discover_GetResult_Response__Sequence * lhs, const explorer_interfaces__action__Discover_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__copy(
  const explorer_interfaces__action__Discover_GetResult_Response__Sequence * input,
  explorer_interfaces__action__Discover_GetResult_Response__Sequence * output);

/// Initialize action/Discover message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explorer_interfaces__action__Discover_FeedbackMessage
 * )) before or use
 * explorer_interfaces__action__Discover_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__init(explorer_interfaces__action__Discover_FeedbackMessage * msg);

/// Finalize action/Discover message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_FeedbackMessage__fini(explorer_interfaces__action__Discover_FeedbackMessage * msg);

/// Create action/Discover message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explorer_interfaces__action__Discover_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_FeedbackMessage *
explorer_interfaces__action__Discover_FeedbackMessage__create();

/// Destroy action/Discover message.
/**
 * It calls
 * explorer_interfaces__action__Discover_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_FeedbackMessage__destroy(explorer_interfaces__action__Discover_FeedbackMessage * msg);

/// Check for action/Discover message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__are_equal(const explorer_interfaces__action__Discover_FeedbackMessage * lhs, const explorer_interfaces__action__Discover_FeedbackMessage * rhs);

/// Copy a action/Discover message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__copy(
  const explorer_interfaces__action__Discover_FeedbackMessage * input,
  explorer_interfaces__action__Discover_FeedbackMessage * output);

/// Initialize array of action/Discover messages.
/**
 * It allocates the memory for the number of elements and calls
 * explorer_interfaces__action__Discover_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__init(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__fini(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array);

/// Create array of action/Discover messages.
/**
 * It allocates the memory for the array and calls
 * explorer_interfaces__action__Discover_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
explorer_interfaces__action__Discover_FeedbackMessage__Sequence *
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Discover messages.
/**
 * It calls
 * explorer_interfaces__action__Discover_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
void
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__destroy(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array);

/// Check for action/Discover message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__are_equal(const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * lhs, const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Discover messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_explorer_interfaces
bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__copy(
  const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * input,
  explorer_interfaces__action__Discover_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__FUNCTIONS_H_
