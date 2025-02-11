// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice
#include "core_interfaces/msg/detail/detected_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
// Member `boxes`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
core_interfaces__msg__DetectedMsg__init(core_interfaces__msg__DetectedMsg * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->objects, 0)) {
    core_interfaces__msg__DetectedMsg__fini(msg);
    return false;
  }
  // boxes
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->boxes, 0)) {
    core_interfaces__msg__DetectedMsg__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__msg__DetectedMsg__fini(core_interfaces__msg__DetectedMsg * msg)
{
  if (!msg) {
    return;
  }
  // objects
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->objects);
  // boxes
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->boxes);
}

bool
core_interfaces__msg__DetectedMsg__are_equal(const core_interfaces__msg__DetectedMsg * lhs, const core_interfaces__msg__DetectedMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  // boxes
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->boxes), &(rhs->boxes)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__msg__DetectedMsg__copy(
  const core_interfaces__msg__DetectedMsg * input,
  core_interfaces__msg__DetectedMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  // boxes
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->boxes), &(output->boxes)))
  {
    return false;
  }
  return true;
}

core_interfaces__msg__DetectedMsg *
core_interfaces__msg__DetectedMsg__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__msg__DetectedMsg * msg = (core_interfaces__msg__DetectedMsg *)allocator.allocate(sizeof(core_interfaces__msg__DetectedMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__msg__DetectedMsg));
  bool success = core_interfaces__msg__DetectedMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__msg__DetectedMsg__destroy(core_interfaces__msg__DetectedMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__msg__DetectedMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__msg__DetectedMsg__Sequence__init(core_interfaces__msg__DetectedMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__msg__DetectedMsg * data = NULL;

  if (size) {
    data = (core_interfaces__msg__DetectedMsg *)allocator.zero_allocate(size, sizeof(core_interfaces__msg__DetectedMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__msg__DetectedMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__msg__DetectedMsg__fini(&data[i - 1]);
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
core_interfaces__msg__DetectedMsg__Sequence__fini(core_interfaces__msg__DetectedMsg__Sequence * array)
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
      core_interfaces__msg__DetectedMsg__fini(&array->data[i]);
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

core_interfaces__msg__DetectedMsg__Sequence *
core_interfaces__msg__DetectedMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__msg__DetectedMsg__Sequence * array = (core_interfaces__msg__DetectedMsg__Sequence *)allocator.allocate(sizeof(core_interfaces__msg__DetectedMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__msg__DetectedMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__msg__DetectedMsg__Sequence__destroy(core_interfaces__msg__DetectedMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__msg__DetectedMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__msg__DetectedMsg__Sequence__are_equal(const core_interfaces__msg__DetectedMsg__Sequence * lhs, const core_interfaces__msg__DetectedMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__msg__DetectedMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__msg__DetectedMsg__Sequence__copy(
  const core_interfaces__msg__DetectedMsg__Sequence * input,
  core_interfaces__msg__DetectedMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__msg__DetectedMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__msg__DetectedMsg * data =
      (core_interfaces__msg__DetectedMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__msg__DetectedMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__msg__DetectedMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__msg__DetectedMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
