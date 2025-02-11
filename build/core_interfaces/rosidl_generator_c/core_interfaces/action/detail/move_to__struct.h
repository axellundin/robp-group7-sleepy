// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from core_interfaces:action/MoveTo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/action/move_to.h"


#ifndef CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_H_
#define CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'way_point'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_Goal
{
  geometry_msgs__msg__PoseStamped way_point;
  bool enforce_orientation;
  bool stop_at_goal;
} core_interfaces__action__MoveTo_Goal;

// Struct for a sequence of core_interfaces__action__MoveTo_Goal.
typedef struct core_interfaces__action__MoveTo_Goal__Sequence
{
  core_interfaces__action__MoveTo_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_Result
{
  bool success;
} core_interfaces__action__MoveTo_Result;

// Struct for a sequence of core_interfaces__action__MoveTo_Result.
typedef struct core_interfaces__action__MoveTo_Result__Sequence
{
  core_interfaces__action__MoveTo_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_Feedback
{
  float distance_to_goal;
} core_interfaces__action__MoveTo_Feedback;

// Struct for a sequence of core_interfaces__action__MoveTo_Feedback.
typedef struct core_interfaces__action__MoveTo_Feedback__Sequence
{
  core_interfaces__action__MoveTo_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "core_interfaces/action/detail/move_to__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  core_interfaces__action__MoveTo_Goal goal;
} core_interfaces__action__MoveTo_SendGoal_Request;

// Struct for a sequence of core_interfaces__action__MoveTo_SendGoal_Request.
typedef struct core_interfaces__action__MoveTo_SendGoal_Request__Sequence
{
  core_interfaces__action__MoveTo_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} core_interfaces__action__MoveTo_SendGoal_Response;

// Struct for a sequence of core_interfaces__action__MoveTo_SendGoal_Response.
typedef struct core_interfaces__action__MoveTo_SendGoal_Response__Sequence
{
  core_interfaces__action__MoveTo_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  core_interfaces__action__MoveTo_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  core_interfaces__action__MoveTo_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  core_interfaces__action__MoveTo_SendGoal_Request__Sequence request;
  core_interfaces__action__MoveTo_SendGoal_Response__Sequence response;
} core_interfaces__action__MoveTo_SendGoal_Event;

// Struct for a sequence of core_interfaces__action__MoveTo_SendGoal_Event.
typedef struct core_interfaces__action__MoveTo_SendGoal_Event__Sequence
{
  core_interfaces__action__MoveTo_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} core_interfaces__action__MoveTo_GetResult_Request;

// Struct for a sequence of core_interfaces__action__MoveTo_GetResult_Request.
typedef struct core_interfaces__action__MoveTo_GetResult_Request__Sequence
{
  core_interfaces__action__MoveTo_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "core_interfaces/action/detail/move_to__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_GetResult_Response
{
  int8_t status;
  core_interfaces__action__MoveTo_Result result;
} core_interfaces__action__MoveTo_GetResult_Response;

// Struct for a sequence of core_interfaces__action__MoveTo_GetResult_Response.
typedef struct core_interfaces__action__MoveTo_GetResult_Response__Sequence
{
  core_interfaces__action__MoveTo_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  core_interfaces__action__MoveTo_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  core_interfaces__action__MoveTo_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  core_interfaces__action__MoveTo_GetResult_Request__Sequence request;
  core_interfaces__action__MoveTo_GetResult_Response__Sequence response;
} core_interfaces__action__MoveTo_GetResult_Event;

// Struct for a sequence of core_interfaces__action__MoveTo_GetResult_Event.
typedef struct core_interfaces__action__MoveTo_GetResult_Event__Sequence
{
  core_interfaces__action__MoveTo_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "core_interfaces/action/detail/move_to__struct.h"

/// Struct defined in action/MoveTo in the package core_interfaces.
typedef struct core_interfaces__action__MoveTo_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  core_interfaces__action__MoveTo_Feedback feedback;
} core_interfaces__action__MoveTo_FeedbackMessage;

// Struct for a sequence of core_interfaces__action__MoveTo_FeedbackMessage.
typedef struct core_interfaces__action__MoveTo_FeedbackMessage__Sequence
{
  core_interfaces__action__MoveTo_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core_interfaces__action__MoveTo_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_H_
