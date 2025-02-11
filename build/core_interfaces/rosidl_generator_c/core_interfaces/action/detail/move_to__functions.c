// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from core_interfaces:action/MoveTo.idl
// generated code does not contain a copyright notice
#include "core_interfaces/action/detail/move_to__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `way_point`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
core_interfaces__action__MoveTo_Goal__init(core_interfaces__action__MoveTo_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // way_point
  if (!geometry_msgs__msg__PoseStamped__init(&msg->way_point)) {
    core_interfaces__action__MoveTo_Goal__fini(msg);
    return false;
  }
  // enforce_orientation
  // stop_at_goal
  return true;
}

void
core_interfaces__action__MoveTo_Goal__fini(core_interfaces__action__MoveTo_Goal * msg)
{
  if (!msg) {
    return;
  }
  // way_point
  geometry_msgs__msg__PoseStamped__fini(&msg->way_point);
  // enforce_orientation
  // stop_at_goal
}

bool
core_interfaces__action__MoveTo_Goal__are_equal(const core_interfaces__action__MoveTo_Goal * lhs, const core_interfaces__action__MoveTo_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // way_point
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->way_point), &(rhs->way_point)))
  {
    return false;
  }
  // enforce_orientation
  if (lhs->enforce_orientation != rhs->enforce_orientation) {
    return false;
  }
  // stop_at_goal
  if (lhs->stop_at_goal != rhs->stop_at_goal) {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Goal__copy(
  const core_interfaces__action__MoveTo_Goal * input,
  core_interfaces__action__MoveTo_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // way_point
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->way_point), &(output->way_point)))
  {
    return false;
  }
  // enforce_orientation
  output->enforce_orientation = input->enforce_orientation;
  // stop_at_goal
  output->stop_at_goal = input->stop_at_goal;
  return true;
}

core_interfaces__action__MoveTo_Goal *
core_interfaces__action__MoveTo_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Goal * msg = (core_interfaces__action__MoveTo_Goal *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_Goal));
  bool success = core_interfaces__action__MoveTo_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_Goal__destroy(core_interfaces__action__MoveTo_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_Goal__Sequence__init(core_interfaces__action__MoveTo_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Goal * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_Goal *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_Goal__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_Goal__Sequence__fini(core_interfaces__action__MoveTo_Goal__Sequence * array)
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
      core_interfaces__action__MoveTo_Goal__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_Goal__Sequence *
core_interfaces__action__MoveTo_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Goal__Sequence * array = (core_interfaces__action__MoveTo_Goal__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_Goal__Sequence__destroy(core_interfaces__action__MoveTo_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_Goal__Sequence__are_equal(const core_interfaces__action__MoveTo_Goal__Sequence * lhs, const core_interfaces__action__MoveTo_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Goal__Sequence__copy(
  const core_interfaces__action__MoveTo_Goal__Sequence * input,
  core_interfaces__action__MoveTo_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_Goal * data =
      (core_interfaces__action__MoveTo_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
core_interfaces__action__MoveTo_Result__init(core_interfaces__action__MoveTo_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
core_interfaces__action__MoveTo_Result__fini(core_interfaces__action__MoveTo_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
core_interfaces__action__MoveTo_Result__are_equal(const core_interfaces__action__MoveTo_Result * lhs, const core_interfaces__action__MoveTo_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Result__copy(
  const core_interfaces__action__MoveTo_Result * input,
  core_interfaces__action__MoveTo_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

core_interfaces__action__MoveTo_Result *
core_interfaces__action__MoveTo_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Result * msg = (core_interfaces__action__MoveTo_Result *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_Result));
  bool success = core_interfaces__action__MoveTo_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_Result__destroy(core_interfaces__action__MoveTo_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_Result__Sequence__init(core_interfaces__action__MoveTo_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Result * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_Result *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_Result__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_Result__Sequence__fini(core_interfaces__action__MoveTo_Result__Sequence * array)
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
      core_interfaces__action__MoveTo_Result__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_Result__Sequence *
core_interfaces__action__MoveTo_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Result__Sequence * array = (core_interfaces__action__MoveTo_Result__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_Result__Sequence__destroy(core_interfaces__action__MoveTo_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_Result__Sequence__are_equal(const core_interfaces__action__MoveTo_Result__Sequence * lhs, const core_interfaces__action__MoveTo_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Result__Sequence__copy(
  const core_interfaces__action__MoveTo_Result__Sequence * input,
  core_interfaces__action__MoveTo_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_Result * data =
      (core_interfaces__action__MoveTo_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
core_interfaces__action__MoveTo_Feedback__init(core_interfaces__action__MoveTo_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // distance_to_goal
  return true;
}

void
core_interfaces__action__MoveTo_Feedback__fini(core_interfaces__action__MoveTo_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // distance_to_goal
}

bool
core_interfaces__action__MoveTo_Feedback__are_equal(const core_interfaces__action__MoveTo_Feedback * lhs, const core_interfaces__action__MoveTo_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance_to_goal
  if (lhs->distance_to_goal != rhs->distance_to_goal) {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Feedback__copy(
  const core_interfaces__action__MoveTo_Feedback * input,
  core_interfaces__action__MoveTo_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // distance_to_goal
  output->distance_to_goal = input->distance_to_goal;
  return true;
}

core_interfaces__action__MoveTo_Feedback *
core_interfaces__action__MoveTo_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Feedback * msg = (core_interfaces__action__MoveTo_Feedback *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_Feedback));
  bool success = core_interfaces__action__MoveTo_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_Feedback__destroy(core_interfaces__action__MoveTo_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_Feedback__Sequence__init(core_interfaces__action__MoveTo_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Feedback * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_Feedback *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_Feedback__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_Feedback__Sequence__fini(core_interfaces__action__MoveTo_Feedback__Sequence * array)
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
      core_interfaces__action__MoveTo_Feedback__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_Feedback__Sequence *
core_interfaces__action__MoveTo_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_Feedback__Sequence * array = (core_interfaces__action__MoveTo_Feedback__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_Feedback__Sequence__destroy(core_interfaces__action__MoveTo_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_Feedback__Sequence__are_equal(const core_interfaces__action__MoveTo_Feedback__Sequence * lhs, const core_interfaces__action__MoveTo_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_Feedback__Sequence__copy(
  const core_interfaces__action__MoveTo_Feedback__Sequence * input,
  core_interfaces__action__MoveTo_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_Feedback * data =
      (core_interfaces__action__MoveTo_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_Feedback__copy(
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
// #include "core_interfaces/action/detail/move_to__functions.h"

bool
core_interfaces__action__MoveTo_SendGoal_Request__init(core_interfaces__action__MoveTo_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    core_interfaces__action__MoveTo_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!core_interfaces__action__MoveTo_Goal__init(&msg->goal)) {
    core_interfaces__action__MoveTo_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_SendGoal_Request__fini(core_interfaces__action__MoveTo_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  core_interfaces__action__MoveTo_Goal__fini(&msg->goal);
}

bool
core_interfaces__action__MoveTo_SendGoal_Request__are_equal(const core_interfaces__action__MoveTo_SendGoal_Request * lhs, const core_interfaces__action__MoveTo_SendGoal_Request * rhs)
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
  if (!core_interfaces__action__MoveTo_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_SendGoal_Request__copy(
  const core_interfaces__action__MoveTo_SendGoal_Request * input,
  core_interfaces__action__MoveTo_SendGoal_Request * output)
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
  if (!core_interfaces__action__MoveTo_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

core_interfaces__action__MoveTo_SendGoal_Request *
core_interfaces__action__MoveTo_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Request * msg = (core_interfaces__action__MoveTo_SendGoal_Request *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_SendGoal_Request));
  bool success = core_interfaces__action__MoveTo_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_SendGoal_Request__destroy(core_interfaces__action__MoveTo_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__init(core_interfaces__action__MoveTo_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Request * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_SendGoal_Request *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_SendGoal_Request__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__fini(core_interfaces__action__MoveTo_SendGoal_Request__Sequence * array)
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
      core_interfaces__action__MoveTo_SendGoal_Request__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_SendGoal_Request__Sequence *
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Request__Sequence * array = (core_interfaces__action__MoveTo_SendGoal_Request__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__destroy(core_interfaces__action__MoveTo_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__are_equal(const core_interfaces__action__MoveTo_SendGoal_Request__Sequence * lhs, const core_interfaces__action__MoveTo_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_SendGoal_Request__Sequence__copy(
  const core_interfaces__action__MoveTo_SendGoal_Request__Sequence * input,
  core_interfaces__action__MoveTo_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_SendGoal_Request * data =
      (core_interfaces__action__MoveTo_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Request__copy(
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
core_interfaces__action__MoveTo_SendGoal_Response__init(core_interfaces__action__MoveTo_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    core_interfaces__action__MoveTo_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_SendGoal_Response__fini(core_interfaces__action__MoveTo_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
core_interfaces__action__MoveTo_SendGoal_Response__are_equal(const core_interfaces__action__MoveTo_SendGoal_Response * lhs, const core_interfaces__action__MoveTo_SendGoal_Response * rhs)
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
core_interfaces__action__MoveTo_SendGoal_Response__copy(
  const core_interfaces__action__MoveTo_SendGoal_Response * input,
  core_interfaces__action__MoveTo_SendGoal_Response * output)
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

core_interfaces__action__MoveTo_SendGoal_Response *
core_interfaces__action__MoveTo_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Response * msg = (core_interfaces__action__MoveTo_SendGoal_Response *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_SendGoal_Response));
  bool success = core_interfaces__action__MoveTo_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_SendGoal_Response__destroy(core_interfaces__action__MoveTo_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__init(core_interfaces__action__MoveTo_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Response * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_SendGoal_Response *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_SendGoal_Response__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__fini(core_interfaces__action__MoveTo_SendGoal_Response__Sequence * array)
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
      core_interfaces__action__MoveTo_SendGoal_Response__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_SendGoal_Response__Sequence *
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Response__Sequence * array = (core_interfaces__action__MoveTo_SendGoal_Response__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__destroy(core_interfaces__action__MoveTo_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__are_equal(const core_interfaces__action__MoveTo_SendGoal_Response__Sequence * lhs, const core_interfaces__action__MoveTo_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_SendGoal_Response__Sequence__copy(
  const core_interfaces__action__MoveTo_SendGoal_Response__Sequence * input,
  core_interfaces__action__MoveTo_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_SendGoal_Response * data =
      (core_interfaces__action__MoveTo_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "core_interfaces/action/detail/move_to__functions.h"

bool
core_interfaces__action__MoveTo_SendGoal_Event__init(core_interfaces__action__MoveTo_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    core_interfaces__action__MoveTo_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    core_interfaces__action__MoveTo_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    core_interfaces__action__MoveTo_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_SendGoal_Event__fini(core_interfaces__action__MoveTo_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  core_interfaces__action__MoveTo_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  core_interfaces__action__MoveTo_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
core_interfaces__action__MoveTo_SendGoal_Event__are_equal(const core_interfaces__action__MoveTo_SendGoal_Event * lhs, const core_interfaces__action__MoveTo_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_SendGoal_Event__copy(
  const core_interfaces__action__MoveTo_SendGoal_Event * input,
  core_interfaces__action__MoveTo_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

core_interfaces__action__MoveTo_SendGoal_Event *
core_interfaces__action__MoveTo_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Event * msg = (core_interfaces__action__MoveTo_SendGoal_Event *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_SendGoal_Event));
  bool success = core_interfaces__action__MoveTo_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_SendGoal_Event__destroy(core_interfaces__action__MoveTo_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__init(core_interfaces__action__MoveTo_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Event * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_SendGoal_Event *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_SendGoal_Event__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__fini(core_interfaces__action__MoveTo_SendGoal_Event__Sequence * array)
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
      core_interfaces__action__MoveTo_SendGoal_Event__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_SendGoal_Event__Sequence *
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_SendGoal_Event__Sequence * array = (core_interfaces__action__MoveTo_SendGoal_Event__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__destroy(core_interfaces__action__MoveTo_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__are_equal(const core_interfaces__action__MoveTo_SendGoal_Event__Sequence * lhs, const core_interfaces__action__MoveTo_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_SendGoal_Event__Sequence__copy(
  const core_interfaces__action__MoveTo_SendGoal_Event__Sequence * input,
  core_interfaces__action__MoveTo_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_SendGoal_Event * data =
      (core_interfaces__action__MoveTo_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_SendGoal_Event__copy(
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
core_interfaces__action__MoveTo_GetResult_Request__init(core_interfaces__action__MoveTo_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    core_interfaces__action__MoveTo_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_GetResult_Request__fini(core_interfaces__action__MoveTo_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
core_interfaces__action__MoveTo_GetResult_Request__are_equal(const core_interfaces__action__MoveTo_GetResult_Request * lhs, const core_interfaces__action__MoveTo_GetResult_Request * rhs)
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
core_interfaces__action__MoveTo_GetResult_Request__copy(
  const core_interfaces__action__MoveTo_GetResult_Request * input,
  core_interfaces__action__MoveTo_GetResult_Request * output)
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

core_interfaces__action__MoveTo_GetResult_Request *
core_interfaces__action__MoveTo_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Request * msg = (core_interfaces__action__MoveTo_GetResult_Request *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_GetResult_Request));
  bool success = core_interfaces__action__MoveTo_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_GetResult_Request__destroy(core_interfaces__action__MoveTo_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_GetResult_Request__Sequence__init(core_interfaces__action__MoveTo_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Request * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_GetResult_Request *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_GetResult_Request__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_GetResult_Request__Sequence__fini(core_interfaces__action__MoveTo_GetResult_Request__Sequence * array)
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
      core_interfaces__action__MoveTo_GetResult_Request__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_GetResult_Request__Sequence *
core_interfaces__action__MoveTo_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Request__Sequence * array = (core_interfaces__action__MoveTo_GetResult_Request__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_GetResult_Request__Sequence__destroy(core_interfaces__action__MoveTo_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_GetResult_Request__Sequence__are_equal(const core_interfaces__action__MoveTo_GetResult_Request__Sequence * lhs, const core_interfaces__action__MoveTo_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_GetResult_Request__Sequence__copy(
  const core_interfaces__action__MoveTo_GetResult_Request__Sequence * input,
  core_interfaces__action__MoveTo_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_GetResult_Request * data =
      (core_interfaces__action__MoveTo_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Request__copy(
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
// #include "core_interfaces/action/detail/move_to__functions.h"

bool
core_interfaces__action__MoveTo_GetResult_Response__init(core_interfaces__action__MoveTo_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!core_interfaces__action__MoveTo_Result__init(&msg->result)) {
    core_interfaces__action__MoveTo_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_GetResult_Response__fini(core_interfaces__action__MoveTo_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  core_interfaces__action__MoveTo_Result__fini(&msg->result);
}

bool
core_interfaces__action__MoveTo_GetResult_Response__are_equal(const core_interfaces__action__MoveTo_GetResult_Response * lhs, const core_interfaces__action__MoveTo_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!core_interfaces__action__MoveTo_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_GetResult_Response__copy(
  const core_interfaces__action__MoveTo_GetResult_Response * input,
  core_interfaces__action__MoveTo_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!core_interfaces__action__MoveTo_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

core_interfaces__action__MoveTo_GetResult_Response *
core_interfaces__action__MoveTo_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Response * msg = (core_interfaces__action__MoveTo_GetResult_Response *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_GetResult_Response));
  bool success = core_interfaces__action__MoveTo_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_GetResult_Response__destroy(core_interfaces__action__MoveTo_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_GetResult_Response__Sequence__init(core_interfaces__action__MoveTo_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Response * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_GetResult_Response *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_GetResult_Response__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_GetResult_Response__Sequence__fini(core_interfaces__action__MoveTo_GetResult_Response__Sequence * array)
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
      core_interfaces__action__MoveTo_GetResult_Response__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_GetResult_Response__Sequence *
core_interfaces__action__MoveTo_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Response__Sequence * array = (core_interfaces__action__MoveTo_GetResult_Response__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_GetResult_Response__Sequence__destroy(core_interfaces__action__MoveTo_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_GetResult_Response__Sequence__are_equal(const core_interfaces__action__MoveTo_GetResult_Response__Sequence * lhs, const core_interfaces__action__MoveTo_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_GetResult_Response__Sequence__copy(
  const core_interfaces__action__MoveTo_GetResult_Response__Sequence * input,
  core_interfaces__action__MoveTo_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_GetResult_Response * data =
      (core_interfaces__action__MoveTo_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "core_interfaces/action/detail/move_to__functions.h"

bool
core_interfaces__action__MoveTo_GetResult_Event__init(core_interfaces__action__MoveTo_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    core_interfaces__action__MoveTo_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_GetResult_Request__Sequence__init(&msg->request, 0)) {
    core_interfaces__action__MoveTo_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_GetResult_Response__Sequence__init(&msg->response, 0)) {
    core_interfaces__action__MoveTo_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_GetResult_Event__fini(core_interfaces__action__MoveTo_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  core_interfaces__action__MoveTo_GetResult_Request__Sequence__fini(&msg->request);
  // response
  core_interfaces__action__MoveTo_GetResult_Response__Sequence__fini(&msg->response);
}

bool
core_interfaces__action__MoveTo_GetResult_Event__are_equal(const core_interfaces__action__MoveTo_GetResult_Event * lhs, const core_interfaces__action__MoveTo_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_GetResult_Event__copy(
  const core_interfaces__action__MoveTo_GetResult_Event * input,
  core_interfaces__action__MoveTo_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!core_interfaces__action__MoveTo_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__action__MoveTo_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

core_interfaces__action__MoveTo_GetResult_Event *
core_interfaces__action__MoveTo_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Event * msg = (core_interfaces__action__MoveTo_GetResult_Event *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_GetResult_Event));
  bool success = core_interfaces__action__MoveTo_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_GetResult_Event__destroy(core_interfaces__action__MoveTo_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_GetResult_Event__Sequence__init(core_interfaces__action__MoveTo_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Event * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_GetResult_Event *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_GetResult_Event__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_GetResult_Event__Sequence__fini(core_interfaces__action__MoveTo_GetResult_Event__Sequence * array)
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
      core_interfaces__action__MoveTo_GetResult_Event__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_GetResult_Event__Sequence *
core_interfaces__action__MoveTo_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_GetResult_Event__Sequence * array = (core_interfaces__action__MoveTo_GetResult_Event__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_GetResult_Event__Sequence__destroy(core_interfaces__action__MoveTo_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_GetResult_Event__Sequence__are_equal(const core_interfaces__action__MoveTo_GetResult_Event__Sequence * lhs, const core_interfaces__action__MoveTo_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_GetResult_Event__Sequence__copy(
  const core_interfaces__action__MoveTo_GetResult_Event__Sequence * input,
  core_interfaces__action__MoveTo_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_GetResult_Event * data =
      (core_interfaces__action__MoveTo_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_GetResult_Event__copy(
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
// #include "core_interfaces/action/detail/move_to__functions.h"

bool
core_interfaces__action__MoveTo_FeedbackMessage__init(core_interfaces__action__MoveTo_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    core_interfaces__action__MoveTo_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!core_interfaces__action__MoveTo_Feedback__init(&msg->feedback)) {
    core_interfaces__action__MoveTo_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__action__MoveTo_FeedbackMessage__fini(core_interfaces__action__MoveTo_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  core_interfaces__action__MoveTo_Feedback__fini(&msg->feedback);
}

bool
core_interfaces__action__MoveTo_FeedbackMessage__are_equal(const core_interfaces__action__MoveTo_FeedbackMessage * lhs, const core_interfaces__action__MoveTo_FeedbackMessage * rhs)
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
  if (!core_interfaces__action__MoveTo_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__action__MoveTo_FeedbackMessage__copy(
  const core_interfaces__action__MoveTo_FeedbackMessage * input,
  core_interfaces__action__MoveTo_FeedbackMessage * output)
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
  if (!core_interfaces__action__MoveTo_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

core_interfaces__action__MoveTo_FeedbackMessage *
core_interfaces__action__MoveTo_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_FeedbackMessage * msg = (core_interfaces__action__MoveTo_FeedbackMessage *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__action__MoveTo_FeedbackMessage));
  bool success = core_interfaces__action__MoveTo_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__action__MoveTo_FeedbackMessage__destroy(core_interfaces__action__MoveTo_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__action__MoveTo_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__init(core_interfaces__action__MoveTo_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_FeedbackMessage * data = NULL;

  if (size) {
    data = (core_interfaces__action__MoveTo_FeedbackMessage *)allocator.zero_allocate(size, sizeof(core_interfaces__action__MoveTo_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__action__MoveTo_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__action__MoveTo_FeedbackMessage__fini(&data[i - 1]);
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
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__fini(core_interfaces__action__MoveTo_FeedbackMessage__Sequence * array)
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
      core_interfaces__action__MoveTo_FeedbackMessage__fini(&array->data[i]);
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

core_interfaces__action__MoveTo_FeedbackMessage__Sequence *
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__action__MoveTo_FeedbackMessage__Sequence * array = (core_interfaces__action__MoveTo_FeedbackMessage__Sequence *)allocator.allocate(sizeof(core_interfaces__action__MoveTo_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__action__MoveTo_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__destroy(core_interfaces__action__MoveTo_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__action__MoveTo_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__are_equal(const core_interfaces__action__MoveTo_FeedbackMessage__Sequence * lhs, const core_interfaces__action__MoveTo_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__action__MoveTo_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__action__MoveTo_FeedbackMessage__Sequence__copy(
  const core_interfaces__action__MoveTo_FeedbackMessage__Sequence * input,
  core_interfaces__action__MoveTo_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__action__MoveTo_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__action__MoveTo_FeedbackMessage * data =
      (core_interfaces__action__MoveTo_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__action__MoveTo_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__action__MoveTo_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__action__MoveTo_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
