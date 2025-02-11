// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from core_interfaces:srv/GeofenceCompliance.idl
// generated code does not contain a copyright notice
#include "core_interfaces/srv/detail/geofence_compliance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
core_interfaces__srv__GeofenceCompliance_Request__init(core_interfaces__srv__GeofenceCompliance_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    core_interfaces__srv__GeofenceCompliance_Request__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__srv__GeofenceCompliance_Request__fini(core_interfaces__srv__GeofenceCompliance_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
}

bool
core_interfaces__srv__GeofenceCompliance_Request__are_equal(const core_interfaces__srv__GeofenceCompliance_Request * lhs, const core_interfaces__srv__GeofenceCompliance_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__srv__GeofenceCompliance_Request__copy(
  const core_interfaces__srv__GeofenceCompliance_Request * input,
  core_interfaces__srv__GeofenceCompliance_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

core_interfaces__srv__GeofenceCompliance_Request *
core_interfaces__srv__GeofenceCompliance_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Request * msg = (core_interfaces__srv__GeofenceCompliance_Request *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__srv__GeofenceCompliance_Request));
  bool success = core_interfaces__srv__GeofenceCompliance_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__srv__GeofenceCompliance_Request__destroy(core_interfaces__srv__GeofenceCompliance_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__srv__GeofenceCompliance_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__srv__GeofenceCompliance_Request__Sequence__init(core_interfaces__srv__GeofenceCompliance_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Request * data = NULL;

  if (size) {
    data = (core_interfaces__srv__GeofenceCompliance_Request *)allocator.zero_allocate(size, sizeof(core_interfaces__srv__GeofenceCompliance_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__srv__GeofenceCompliance_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__srv__GeofenceCompliance_Request__fini(&data[i - 1]);
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
core_interfaces__srv__GeofenceCompliance_Request__Sequence__fini(core_interfaces__srv__GeofenceCompliance_Request__Sequence * array)
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
      core_interfaces__srv__GeofenceCompliance_Request__fini(&array->data[i]);
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

core_interfaces__srv__GeofenceCompliance_Request__Sequence *
core_interfaces__srv__GeofenceCompliance_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Request__Sequence * array = (core_interfaces__srv__GeofenceCompliance_Request__Sequence *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__srv__GeofenceCompliance_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__srv__GeofenceCompliance_Request__Sequence__destroy(core_interfaces__srv__GeofenceCompliance_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__srv__GeofenceCompliance_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__srv__GeofenceCompliance_Request__Sequence__are_equal(const core_interfaces__srv__GeofenceCompliance_Request__Sequence * lhs, const core_interfaces__srv__GeofenceCompliance_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__srv__GeofenceCompliance_Request__Sequence__copy(
  const core_interfaces__srv__GeofenceCompliance_Request__Sequence * input,
  core_interfaces__srv__GeofenceCompliance_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__srv__GeofenceCompliance_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__srv__GeofenceCompliance_Request * data =
      (core_interfaces__srv__GeofenceCompliance_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__srv__GeofenceCompliance_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__srv__GeofenceCompliance_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
core_interfaces__srv__GeofenceCompliance_Response__init(core_interfaces__srv__GeofenceCompliance_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
core_interfaces__srv__GeofenceCompliance_Response__fini(core_interfaces__srv__GeofenceCompliance_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
core_interfaces__srv__GeofenceCompliance_Response__are_equal(const core_interfaces__srv__GeofenceCompliance_Response * lhs, const core_interfaces__srv__GeofenceCompliance_Response * rhs)
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
core_interfaces__srv__GeofenceCompliance_Response__copy(
  const core_interfaces__srv__GeofenceCompliance_Response * input,
  core_interfaces__srv__GeofenceCompliance_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

core_interfaces__srv__GeofenceCompliance_Response *
core_interfaces__srv__GeofenceCompliance_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Response * msg = (core_interfaces__srv__GeofenceCompliance_Response *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__srv__GeofenceCompliance_Response));
  bool success = core_interfaces__srv__GeofenceCompliance_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__srv__GeofenceCompliance_Response__destroy(core_interfaces__srv__GeofenceCompliance_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__srv__GeofenceCompliance_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__srv__GeofenceCompliance_Response__Sequence__init(core_interfaces__srv__GeofenceCompliance_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Response * data = NULL;

  if (size) {
    data = (core_interfaces__srv__GeofenceCompliance_Response *)allocator.zero_allocate(size, sizeof(core_interfaces__srv__GeofenceCompliance_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__srv__GeofenceCompliance_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__srv__GeofenceCompliance_Response__fini(&data[i - 1]);
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
core_interfaces__srv__GeofenceCompliance_Response__Sequence__fini(core_interfaces__srv__GeofenceCompliance_Response__Sequence * array)
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
      core_interfaces__srv__GeofenceCompliance_Response__fini(&array->data[i]);
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

core_interfaces__srv__GeofenceCompliance_Response__Sequence *
core_interfaces__srv__GeofenceCompliance_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Response__Sequence * array = (core_interfaces__srv__GeofenceCompliance_Response__Sequence *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__srv__GeofenceCompliance_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__srv__GeofenceCompliance_Response__Sequence__destroy(core_interfaces__srv__GeofenceCompliance_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__srv__GeofenceCompliance_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__srv__GeofenceCompliance_Response__Sequence__are_equal(const core_interfaces__srv__GeofenceCompliance_Response__Sequence * lhs, const core_interfaces__srv__GeofenceCompliance_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__srv__GeofenceCompliance_Response__Sequence__copy(
  const core_interfaces__srv__GeofenceCompliance_Response__Sequence * input,
  core_interfaces__srv__GeofenceCompliance_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__srv__GeofenceCompliance_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__srv__GeofenceCompliance_Response * data =
      (core_interfaces__srv__GeofenceCompliance_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__srv__GeofenceCompliance_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__srv__GeofenceCompliance_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Response__copy(
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
// #include "core_interfaces/srv/detail/geofence_compliance__functions.h"

bool
core_interfaces__srv__GeofenceCompliance_Event__init(core_interfaces__srv__GeofenceCompliance_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    core_interfaces__srv__GeofenceCompliance_Event__fini(msg);
    return false;
  }
  // request
  if (!core_interfaces__srv__GeofenceCompliance_Request__Sequence__init(&msg->request, 0)) {
    core_interfaces__srv__GeofenceCompliance_Event__fini(msg);
    return false;
  }
  // response
  if (!core_interfaces__srv__GeofenceCompliance_Response__Sequence__init(&msg->response, 0)) {
    core_interfaces__srv__GeofenceCompliance_Event__fini(msg);
    return false;
  }
  return true;
}

void
core_interfaces__srv__GeofenceCompliance_Event__fini(core_interfaces__srv__GeofenceCompliance_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  core_interfaces__srv__GeofenceCompliance_Request__Sequence__fini(&msg->request);
  // response
  core_interfaces__srv__GeofenceCompliance_Response__Sequence__fini(&msg->response);
}

bool
core_interfaces__srv__GeofenceCompliance_Event__are_equal(const core_interfaces__srv__GeofenceCompliance_Event * lhs, const core_interfaces__srv__GeofenceCompliance_Event * rhs)
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
  if (!core_interfaces__srv__GeofenceCompliance_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__srv__GeofenceCompliance_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
core_interfaces__srv__GeofenceCompliance_Event__copy(
  const core_interfaces__srv__GeofenceCompliance_Event * input,
  core_interfaces__srv__GeofenceCompliance_Event * output)
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
  if (!core_interfaces__srv__GeofenceCompliance_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!core_interfaces__srv__GeofenceCompliance_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

core_interfaces__srv__GeofenceCompliance_Event *
core_interfaces__srv__GeofenceCompliance_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Event * msg = (core_interfaces__srv__GeofenceCompliance_Event *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core_interfaces__srv__GeofenceCompliance_Event));
  bool success = core_interfaces__srv__GeofenceCompliance_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core_interfaces__srv__GeofenceCompliance_Event__destroy(core_interfaces__srv__GeofenceCompliance_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core_interfaces__srv__GeofenceCompliance_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core_interfaces__srv__GeofenceCompliance_Event__Sequence__init(core_interfaces__srv__GeofenceCompliance_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Event * data = NULL;

  if (size) {
    data = (core_interfaces__srv__GeofenceCompliance_Event *)allocator.zero_allocate(size, sizeof(core_interfaces__srv__GeofenceCompliance_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core_interfaces__srv__GeofenceCompliance_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core_interfaces__srv__GeofenceCompliance_Event__fini(&data[i - 1]);
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
core_interfaces__srv__GeofenceCompliance_Event__Sequence__fini(core_interfaces__srv__GeofenceCompliance_Event__Sequence * array)
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
      core_interfaces__srv__GeofenceCompliance_Event__fini(&array->data[i]);
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

core_interfaces__srv__GeofenceCompliance_Event__Sequence *
core_interfaces__srv__GeofenceCompliance_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core_interfaces__srv__GeofenceCompliance_Event__Sequence * array = (core_interfaces__srv__GeofenceCompliance_Event__Sequence *)allocator.allocate(sizeof(core_interfaces__srv__GeofenceCompliance_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core_interfaces__srv__GeofenceCompliance_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core_interfaces__srv__GeofenceCompliance_Event__Sequence__destroy(core_interfaces__srv__GeofenceCompliance_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core_interfaces__srv__GeofenceCompliance_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core_interfaces__srv__GeofenceCompliance_Event__Sequence__are_equal(const core_interfaces__srv__GeofenceCompliance_Event__Sequence * lhs, const core_interfaces__srv__GeofenceCompliance_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core_interfaces__srv__GeofenceCompliance_Event__Sequence__copy(
  const core_interfaces__srv__GeofenceCompliance_Event__Sequence * input,
  core_interfaces__srv__GeofenceCompliance_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core_interfaces__srv__GeofenceCompliance_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core_interfaces__srv__GeofenceCompliance_Event * data =
      (core_interfaces__srv__GeofenceCompliance_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core_interfaces__srv__GeofenceCompliance_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core_interfaces__srv__GeofenceCompliance_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core_interfaces__srv__GeofenceCompliance_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
