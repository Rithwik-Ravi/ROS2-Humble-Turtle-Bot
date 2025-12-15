// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from explorer_interfaces:action/Discover.idl
// generated code does not contain a copyright notice
#include "explorer_interfaces/action/detail/discover__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
explorer_interfaces__action__Discover_Goal__init(explorer_interfaces__action__Discover_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // strategy
  // map_completed_thres
  return true;
}

void
explorer_interfaces__action__Discover_Goal__fini(explorer_interfaces__action__Discover_Goal * msg)
{
  if (!msg) {
    return;
  }
  // strategy
  // map_completed_thres
}

bool
explorer_interfaces__action__Discover_Goal__are_equal(const explorer_interfaces__action__Discover_Goal * lhs, const explorer_interfaces__action__Discover_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // strategy
  if (lhs->strategy != rhs->strategy) {
    return false;
  }
  // map_completed_thres
  if (lhs->map_completed_thres != rhs->map_completed_thres) {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Goal__copy(
  const explorer_interfaces__action__Discover_Goal * input,
  explorer_interfaces__action__Discover_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // strategy
  output->strategy = input->strategy;
  // map_completed_thres
  output->map_completed_thres = input->map_completed_thres;
  return true;
}

explorer_interfaces__action__Discover_Goal *
explorer_interfaces__action__Discover_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Goal * msg = (explorer_interfaces__action__Discover_Goal *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_Goal));
  bool success = explorer_interfaces__action__Discover_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_Goal__destroy(explorer_interfaces__action__Discover_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_Goal__Sequence__init(explorer_interfaces__action__Discover_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Goal * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_Goal *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_Goal__Sequence__fini(explorer_interfaces__action__Discover_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_Goal__Sequence *
explorer_interfaces__action__Discover_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Goal__Sequence * array = (explorer_interfaces__action__Discover_Goal__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_Goal__Sequence__destroy(explorer_interfaces__action__Discover_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_Goal__Sequence__are_equal(const explorer_interfaces__action__Discover_Goal__Sequence * lhs, const explorer_interfaces__action__Discover_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Goal__Sequence__copy(
  const explorer_interfaces__action__Discover_Goal__Sequence * input,
  explorer_interfaces__action__Discover_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_Goal * data =
      (explorer_interfaces__action__Discover_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
explorer_interfaces__action__Discover_Result__init(explorer_interfaces__action__Discover_Result * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
explorer_interfaces__action__Discover_Result__fini(explorer_interfaces__action__Discover_Result * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
explorer_interfaces__action__Discover_Result__are_equal(const explorer_interfaces__action__Discover_Result * lhs, const explorer_interfaces__action__Discover_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Result__copy(
  const explorer_interfaces__action__Discover_Result * input,
  explorer_interfaces__action__Discover_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

explorer_interfaces__action__Discover_Result *
explorer_interfaces__action__Discover_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Result * msg = (explorer_interfaces__action__Discover_Result *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_Result));
  bool success = explorer_interfaces__action__Discover_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_Result__destroy(explorer_interfaces__action__Discover_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_Result__Sequence__init(explorer_interfaces__action__Discover_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Result * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_Result *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_Result__Sequence__fini(explorer_interfaces__action__Discover_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_Result__Sequence *
explorer_interfaces__action__Discover_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Result__Sequence * array = (explorer_interfaces__action__Discover_Result__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_Result__Sequence__destroy(explorer_interfaces__action__Discover_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_Result__Sequence__are_equal(const explorer_interfaces__action__Discover_Result__Sequence * lhs, const explorer_interfaces__action__Discover_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Result__Sequence__copy(
  const explorer_interfaces__action__Discover_Result__Sequence * input,
  explorer_interfaces__action__Discover_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_Result * data =
      (explorer_interfaces__action__Discover_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
explorer_interfaces__action__Discover_Feedback__init(explorer_interfaces__action__Discover_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // progress
  return true;
}

void
explorer_interfaces__action__Discover_Feedback__fini(explorer_interfaces__action__Discover_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // progress
}

bool
explorer_interfaces__action__Discover_Feedback__are_equal(const explorer_interfaces__action__Discover_Feedback * lhs, const explorer_interfaces__action__Discover_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // progress
  if (lhs->progress != rhs->progress) {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Feedback__copy(
  const explorer_interfaces__action__Discover_Feedback * input,
  explorer_interfaces__action__Discover_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // progress
  output->progress = input->progress;
  return true;
}

explorer_interfaces__action__Discover_Feedback *
explorer_interfaces__action__Discover_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Feedback * msg = (explorer_interfaces__action__Discover_Feedback *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_Feedback));
  bool success = explorer_interfaces__action__Discover_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_Feedback__destroy(explorer_interfaces__action__Discover_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_Feedback__Sequence__init(explorer_interfaces__action__Discover_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Feedback * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_Feedback *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_Feedback__Sequence__fini(explorer_interfaces__action__Discover_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_Feedback__Sequence *
explorer_interfaces__action__Discover_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_Feedback__Sequence * array = (explorer_interfaces__action__Discover_Feedback__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_Feedback__Sequence__destroy(explorer_interfaces__action__Discover_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_Feedback__Sequence__are_equal(const explorer_interfaces__action__Discover_Feedback__Sequence * lhs, const explorer_interfaces__action__Discover_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_Feedback__Sequence__copy(
  const explorer_interfaces__action__Discover_Feedback__Sequence * input,
  explorer_interfaces__action__Discover_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_Feedback * data =
      (explorer_interfaces__action__Discover_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "explorer_interfaces/action/detail/discover__functions.h"

bool
explorer_interfaces__action__Discover_SendGoal_Request__init(explorer_interfaces__action__Discover_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    explorer_interfaces__action__Discover_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!explorer_interfaces__action__Discover_Goal__init(&msg->goal)) {
    explorer_interfaces__action__Discover_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
explorer_interfaces__action__Discover_SendGoal_Request__fini(explorer_interfaces__action__Discover_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  explorer_interfaces__action__Discover_Goal__fini(&msg->goal);
}

bool
explorer_interfaces__action__Discover_SendGoal_Request__are_equal(const explorer_interfaces__action__Discover_SendGoal_Request * lhs, const explorer_interfaces__action__Discover_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!explorer_interfaces__action__Discover_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_SendGoal_Request__copy(
  const explorer_interfaces__action__Discover_SendGoal_Request * input,
  explorer_interfaces__action__Discover_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!explorer_interfaces__action__Discover_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

explorer_interfaces__action__Discover_SendGoal_Request *
explorer_interfaces__action__Discover_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Request * msg = (explorer_interfaces__action__Discover_SendGoal_Request *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_SendGoal_Request));
  bool success = explorer_interfaces__action__Discover_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_SendGoal_Request__destroy(explorer_interfaces__action__Discover_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__init(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Request * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_SendGoal_Request *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__fini(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_SendGoal_Request__Sequence *
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array = (explorer_interfaces__action__Discover_SendGoal_Request__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__destroy(explorer_interfaces__action__Discover_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__are_equal(const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * lhs, const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_SendGoal_Request__Sequence__copy(
  const explorer_interfaces__action__Discover_SendGoal_Request__Sequence * input,
  explorer_interfaces__action__Discover_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_SendGoal_Request * data =
      (explorer_interfaces__action__Discover_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
explorer_interfaces__action__Discover_SendGoal_Response__init(explorer_interfaces__action__Discover_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    explorer_interfaces__action__Discover_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
explorer_interfaces__action__Discover_SendGoal_Response__fini(explorer_interfaces__action__Discover_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
explorer_interfaces__action__Discover_SendGoal_Response__are_equal(const explorer_interfaces__action__Discover_SendGoal_Response * lhs, const explorer_interfaces__action__Discover_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_SendGoal_Response__copy(
  const explorer_interfaces__action__Discover_SendGoal_Response * input,
  explorer_interfaces__action__Discover_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

explorer_interfaces__action__Discover_SendGoal_Response *
explorer_interfaces__action__Discover_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Response * msg = (explorer_interfaces__action__Discover_SendGoal_Response *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_SendGoal_Response));
  bool success = explorer_interfaces__action__Discover_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_SendGoal_Response__destroy(explorer_interfaces__action__Discover_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__init(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Response * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_SendGoal_Response *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__fini(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_SendGoal_Response__Sequence *
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array = (explorer_interfaces__action__Discover_SendGoal_Response__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__destroy(explorer_interfaces__action__Discover_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__are_equal(const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * lhs, const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_SendGoal_Response__Sequence__copy(
  const explorer_interfaces__action__Discover_SendGoal_Response__Sequence * input,
  explorer_interfaces__action__Discover_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_SendGoal_Response * data =
      (explorer_interfaces__action__Discover_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
explorer_interfaces__action__Discover_GetResult_Request__init(explorer_interfaces__action__Discover_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    explorer_interfaces__action__Discover_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
explorer_interfaces__action__Discover_GetResult_Request__fini(explorer_interfaces__action__Discover_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
explorer_interfaces__action__Discover_GetResult_Request__are_equal(const explorer_interfaces__action__Discover_GetResult_Request * lhs, const explorer_interfaces__action__Discover_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_GetResult_Request__copy(
  const explorer_interfaces__action__Discover_GetResult_Request * input,
  explorer_interfaces__action__Discover_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

explorer_interfaces__action__Discover_GetResult_Request *
explorer_interfaces__action__Discover_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Request * msg = (explorer_interfaces__action__Discover_GetResult_Request *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_GetResult_Request));
  bool success = explorer_interfaces__action__Discover_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_GetResult_Request__destroy(explorer_interfaces__action__Discover_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__init(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Request * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_GetResult_Request *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_GetResult_Request__Sequence__fini(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_GetResult_Request__Sequence *
explorer_interfaces__action__Discover_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Request__Sequence * array = (explorer_interfaces__action__Discover_GetResult_Request__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_GetResult_Request__Sequence__destroy(explorer_interfaces__action__Discover_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__are_equal(const explorer_interfaces__action__Discover_GetResult_Request__Sequence * lhs, const explorer_interfaces__action__Discover_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_GetResult_Request__Sequence__copy(
  const explorer_interfaces__action__Discover_GetResult_Request__Sequence * input,
  explorer_interfaces__action__Discover_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_GetResult_Request * data =
      (explorer_interfaces__action__Discover_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "explorer_interfaces/action/detail/discover__functions.h"

bool
explorer_interfaces__action__Discover_GetResult_Response__init(explorer_interfaces__action__Discover_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!explorer_interfaces__action__Discover_Result__init(&msg->result)) {
    explorer_interfaces__action__Discover_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
explorer_interfaces__action__Discover_GetResult_Response__fini(explorer_interfaces__action__Discover_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  explorer_interfaces__action__Discover_Result__fini(&msg->result);
}

bool
explorer_interfaces__action__Discover_GetResult_Response__are_equal(const explorer_interfaces__action__Discover_GetResult_Response * lhs, const explorer_interfaces__action__Discover_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!explorer_interfaces__action__Discover_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_GetResult_Response__copy(
  const explorer_interfaces__action__Discover_GetResult_Response * input,
  explorer_interfaces__action__Discover_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!explorer_interfaces__action__Discover_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

explorer_interfaces__action__Discover_GetResult_Response *
explorer_interfaces__action__Discover_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Response * msg = (explorer_interfaces__action__Discover_GetResult_Response *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_GetResult_Response));
  bool success = explorer_interfaces__action__Discover_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_GetResult_Response__destroy(explorer_interfaces__action__Discover_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__init(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Response * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_GetResult_Response *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_GetResult_Response__Sequence__fini(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_GetResult_Response__Sequence *
explorer_interfaces__action__Discover_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_GetResult_Response__Sequence * array = (explorer_interfaces__action__Discover_GetResult_Response__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_GetResult_Response__Sequence__destroy(explorer_interfaces__action__Discover_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__are_equal(const explorer_interfaces__action__Discover_GetResult_Response__Sequence * lhs, const explorer_interfaces__action__Discover_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_GetResult_Response__Sequence__copy(
  const explorer_interfaces__action__Discover_GetResult_Response__Sequence * input,
  explorer_interfaces__action__Discover_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_GetResult_Response * data =
      (explorer_interfaces__action__Discover_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "explorer_interfaces/action/detail/discover__functions.h"

bool
explorer_interfaces__action__Discover_FeedbackMessage__init(explorer_interfaces__action__Discover_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    explorer_interfaces__action__Discover_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!explorer_interfaces__action__Discover_Feedback__init(&msg->feedback)) {
    explorer_interfaces__action__Discover_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
explorer_interfaces__action__Discover_FeedbackMessage__fini(explorer_interfaces__action__Discover_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  explorer_interfaces__action__Discover_Feedback__fini(&msg->feedback);
}

bool
explorer_interfaces__action__Discover_FeedbackMessage__are_equal(const explorer_interfaces__action__Discover_FeedbackMessage * lhs, const explorer_interfaces__action__Discover_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!explorer_interfaces__action__Discover_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
explorer_interfaces__action__Discover_FeedbackMessage__copy(
  const explorer_interfaces__action__Discover_FeedbackMessage * input,
  explorer_interfaces__action__Discover_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!explorer_interfaces__action__Discover_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

explorer_interfaces__action__Discover_FeedbackMessage *
explorer_interfaces__action__Discover_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_FeedbackMessage * msg = (explorer_interfaces__action__Discover_FeedbackMessage *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explorer_interfaces__action__Discover_FeedbackMessage));
  bool success = explorer_interfaces__action__Discover_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explorer_interfaces__action__Discover_FeedbackMessage__destroy(explorer_interfaces__action__Discover_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explorer_interfaces__action__Discover_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__init(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_FeedbackMessage * data = NULL;

  if (size) {
    data = (explorer_interfaces__action__Discover_FeedbackMessage *)allocator.zero_allocate(size, sizeof(explorer_interfaces__action__Discover_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explorer_interfaces__action__Discover_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explorer_interfaces__action__Discover_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__fini(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explorer_interfaces__action__Discover_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explorer_interfaces__action__Discover_FeedbackMessage__Sequence *
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array = (explorer_interfaces__action__Discover_FeedbackMessage__Sequence *)allocator.allocate(sizeof(explorer_interfaces__action__Discover_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explorer_interfaces__action__Discover_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__destroy(explorer_interfaces__action__Discover_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explorer_interfaces__action__Discover_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__are_equal(const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * lhs, const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explorer_interfaces__action__Discover_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explorer_interfaces__action__Discover_FeedbackMessage__Sequence__copy(
  const explorer_interfaces__action__Discover_FeedbackMessage__Sequence * input,
  explorer_interfaces__action__Discover_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explorer_interfaces__action__Discover_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explorer_interfaces__action__Discover_FeedbackMessage * data =
      (explorer_interfaces__action__Discover_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explorer_interfaces__action__Discover_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explorer_interfaces__action__Discover_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explorer_interfaces__action__Discover_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
