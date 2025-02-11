// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from core_interfaces:srv/GeofenceCompliance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/srv/geofence_compliance.h"


#ifndef CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__STRUCT_H_
#define CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/GeofenceCompliance in the package core_interfaces.
typedef struct core_interfaces__srv__GeofenceCompliance_Request
{
  geometry_msgs__msg__PoseStamped pose;
} core_interfaces__srv__GeofenceCompliance_Request;

// Struct for a sequence of core_interfaces__srv__GeofenceCompliance_Request.
typedef struct core_interfaces__srv__GeofenceCompliance_Request__Sequence
{
  core_interfaces__srv__GeofenceCompliance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__srv__GeofenceCompliance_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/GeofenceCompliance in the package core_interfaces.
typedef struct core_interfaces__srv__GeofenceCompliance_Response
{
  bool success;
} core_interfaces__srv__GeofenceCompliance_Response;

// Struct for a sequence of core_interfaces__srv__GeofenceCompliance_Response.
typedef struct core_interfaces__srv__GeofenceCompliance_Response__Sequence
{
  core_interfaces__srv__GeofenceCompliance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__srv__GeofenceCompliance_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  core_interfaces__srv__GeofenceCompliance_Event__request__MAX_SIZE = 1
};
// response
enum
{
  core_interfaces__srv__GeofenceCompliance_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GeofenceCompliance in the package core_interfaces.
typedef struct core_interfaces__srv__GeofenceCompliance_Event
{
  service_msgs__msg__ServiceEventInfo info;
  core_interfaces__srv__GeofenceCompliance_Request__Sequence request;
  core_interfaces__srv__GeofenceCompliance_Response__Sequence response;
} core_interfaces__srv__GeofenceCompliance_Event;

// Struct for a sequence of core_interfaces__srv__GeofenceCompliance_Event.
typedef struct core_interfaces__srv__GeofenceCompliance_Event__Sequence
{
  core_interfaces__srv__GeofenceCompliance_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__srv__GeofenceCompliance_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__STRUCT_H_
