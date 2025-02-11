// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/msg/detected_msg.h"


#ifndef CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_H_
#define CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'objects'
// Member 'boxes'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/DetectedMsg in the package core_interfaces.
typedef struct core_interfaces__msg__DetectedMsg
{
  geometry_msgs__msg__PoseStamped__Sequence objects;
  geometry_msgs__msg__PoseStamped__Sequence boxes;
} core_interfaces__msg__DetectedMsg;

// Struct for a sequence of core_interfaces__msg__DetectedMsg.
typedef struct core_interfaces__msg__DetectedMsg__Sequence
{
  core_interfaces__msg__DetectedMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__msg__DetectedMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_H_
