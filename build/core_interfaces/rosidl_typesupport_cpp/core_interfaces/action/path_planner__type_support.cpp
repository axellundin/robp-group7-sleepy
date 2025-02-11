// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from core_interfaces:action/PathPlanner.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "core_interfaces/action/detail/path_planner__functions.h"
#include "core_interfaces/action/detail/path_planner__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_Goal_type_support_ids_t;

static const _PathPlanner_Goal_type_support_ids_t _PathPlanner_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_Goal_type_support_symbol_names_t _PathPlanner_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_Goal)),
  }
};

typedef struct _PathPlanner_Goal_type_support_data_t
{
  void * data[2];
} _PathPlanner_Goal_type_support_data_t;

static _PathPlanner_Goal_type_support_data_t _PathPlanner_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_Goal_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_Goal__get_type_hash,
  &core_interfaces__action__PathPlanner_Goal__get_type_description,
  &core_interfaces__action__PathPlanner_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_Goal>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_Goal)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_Result_type_support_ids_t;

static const _PathPlanner_Result_type_support_ids_t _PathPlanner_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_Result_type_support_symbol_names_t _PathPlanner_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_Result)),
  }
};

typedef struct _PathPlanner_Result_type_support_data_t
{
  void * data[2];
} _PathPlanner_Result_type_support_data_t;

static _PathPlanner_Result_type_support_data_t _PathPlanner_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_Result_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_Result_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_Result_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_Result__get_type_hash,
  &core_interfaces__action__PathPlanner_Result__get_type_description,
  &core_interfaces__action__PathPlanner_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_Result>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_Result)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_Feedback_type_support_ids_t;

static const _PathPlanner_Feedback_type_support_ids_t _PathPlanner_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_Feedback_type_support_symbol_names_t _PathPlanner_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_Feedback)),
  }
};

typedef struct _PathPlanner_Feedback_type_support_data_t
{
  void * data[2];
} _PathPlanner_Feedback_type_support_data_t;

static _PathPlanner_Feedback_type_support_data_t _PathPlanner_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_Feedback_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_Feedback__get_type_hash,
  &core_interfaces__action__PathPlanner_Feedback__get_type_description,
  &core_interfaces__action__PathPlanner_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_Feedback>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_Feedback)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_SendGoal_Request_type_support_ids_t;

static const _PathPlanner_SendGoal_Request_type_support_ids_t _PathPlanner_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_SendGoal_Request_type_support_symbol_names_t _PathPlanner_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_SendGoal_Request)),
  }
};

typedef struct _PathPlanner_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _PathPlanner_SendGoal_Request_type_support_data_t;

static _PathPlanner_SendGoal_Request_type_support_data_t _PathPlanner_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_SendGoal_Request_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_SendGoal_Request__get_type_hash,
  &core_interfaces__action__PathPlanner_SendGoal_Request__get_type_description,
  &core_interfaces__action__PathPlanner_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Request>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_SendGoal_Request)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_SendGoal_Response_type_support_ids_t;

static const _PathPlanner_SendGoal_Response_type_support_ids_t _PathPlanner_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_SendGoal_Response_type_support_symbol_names_t _PathPlanner_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_SendGoal_Response)),
  }
};

typedef struct _PathPlanner_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _PathPlanner_SendGoal_Response_type_support_data_t;

static _PathPlanner_SendGoal_Response_type_support_data_t _PathPlanner_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_SendGoal_Response_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_SendGoal_Response__get_type_hash,
  &core_interfaces__action__PathPlanner_SendGoal_Response__get_type_description,
  &core_interfaces__action__PathPlanner_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Response>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_SendGoal_Response)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_SendGoal_Event_type_support_ids_t;

static const _PathPlanner_SendGoal_Event_type_support_ids_t _PathPlanner_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_SendGoal_Event_type_support_symbol_names_t _PathPlanner_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_SendGoal_Event)),
  }
};

typedef struct _PathPlanner_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _PathPlanner_SendGoal_Event_type_support_data_t;

static _PathPlanner_SendGoal_Event_type_support_data_t _PathPlanner_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_SendGoal_Event_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_SendGoal_Event__get_type_hash,
  &core_interfaces__action__PathPlanner_SendGoal_Event__get_type_description,
  &core_interfaces__action__PathPlanner_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Event>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_SendGoal_Event)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_SendGoal_type_support_ids_t;

static const _PathPlanner_SendGoal_type_support_ids_t _PathPlanner_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_SendGoal_type_support_symbol_names_t _PathPlanner_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_SendGoal)),
  }
};

typedef struct _PathPlanner_SendGoal_type_support_data_t
{
  void * data[2];
} _PathPlanner_SendGoal_type_support_data_t;

static _PathPlanner_SendGoal_type_support_data_t _PathPlanner_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_SendGoal_service_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PathPlanner_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<core_interfaces::action::PathPlanner_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<core_interfaces::action::PathPlanner_SendGoal>,
  &core_interfaces__action__PathPlanner_SendGoal__get_type_hash,
  &core_interfaces__action__PathPlanner_SendGoal__get_type_description,
  &core_interfaces__action__PathPlanner_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<core_interfaces::action::PathPlanner_SendGoal>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<core_interfaces::action::PathPlanner_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_GetResult_Request_type_support_ids_t;

static const _PathPlanner_GetResult_Request_type_support_ids_t _PathPlanner_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_GetResult_Request_type_support_symbol_names_t _PathPlanner_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_GetResult_Request)),
  }
};

typedef struct _PathPlanner_GetResult_Request_type_support_data_t
{
  void * data[2];
} _PathPlanner_GetResult_Request_type_support_data_t;

static _PathPlanner_GetResult_Request_type_support_data_t _PathPlanner_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_GetResult_Request_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_GetResult_Request__get_type_hash,
  &core_interfaces__action__PathPlanner_GetResult_Request__get_type_description,
  &core_interfaces__action__PathPlanner_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Request>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_GetResult_Request)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_GetResult_Response_type_support_ids_t;

static const _PathPlanner_GetResult_Response_type_support_ids_t _PathPlanner_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_GetResult_Response_type_support_symbol_names_t _PathPlanner_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_GetResult_Response)),
  }
};

typedef struct _PathPlanner_GetResult_Response_type_support_data_t
{
  void * data[2];
} _PathPlanner_GetResult_Response_type_support_data_t;

static _PathPlanner_GetResult_Response_type_support_data_t _PathPlanner_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_GetResult_Response_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_GetResult_Response__get_type_hash,
  &core_interfaces__action__PathPlanner_GetResult_Response__get_type_description,
  &core_interfaces__action__PathPlanner_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Response>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_GetResult_Response)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_GetResult_Event_type_support_ids_t;

static const _PathPlanner_GetResult_Event_type_support_ids_t _PathPlanner_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_GetResult_Event_type_support_symbol_names_t _PathPlanner_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_GetResult_Event)),
  }
};

typedef struct _PathPlanner_GetResult_Event_type_support_data_t
{
  void * data[2];
} _PathPlanner_GetResult_Event_type_support_data_t;

static _PathPlanner_GetResult_Event_type_support_data_t _PathPlanner_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_GetResult_Event_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_GetResult_Event__get_type_hash,
  &core_interfaces__action__PathPlanner_GetResult_Event__get_type_description,
  &core_interfaces__action__PathPlanner_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Event>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_GetResult_Event)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_GetResult_type_support_ids_t;

static const _PathPlanner_GetResult_type_support_ids_t _PathPlanner_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_GetResult_type_support_symbol_names_t _PathPlanner_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_GetResult)),
  }
};

typedef struct _PathPlanner_GetResult_type_support_data_t
{
  void * data[2];
} _PathPlanner_GetResult_type_support_data_t;

static _PathPlanner_GetResult_type_support_data_t _PathPlanner_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_GetResult_service_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PathPlanner_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::PathPlanner_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<core_interfaces::action::PathPlanner_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<core_interfaces::action::PathPlanner_GetResult>,
  &core_interfaces__action__PathPlanner_GetResult__get_type_hash,
  &core_interfaces__action__PathPlanner_GetResult__get_type_description,
  &core_interfaces__action__PathPlanner_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<core_interfaces::action::PathPlanner_GetResult>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<core_interfaces::action::PathPlanner_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__functions.h"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PathPlanner_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPlanner_FeedbackMessage_type_support_ids_t;

static const _PathPlanner_FeedbackMessage_type_support_ids_t _PathPlanner_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PathPlanner_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPlanner_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPlanner_FeedbackMessage_type_support_symbol_names_t _PathPlanner_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, PathPlanner_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, PathPlanner_FeedbackMessage)),
  }
};

typedef struct _PathPlanner_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _PathPlanner_FeedbackMessage_type_support_data_t;

static _PathPlanner_FeedbackMessage_type_support_data_t _PathPlanner_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPlanner_FeedbackMessage_message_typesupport_map = {
  2,
  "core_interfaces",
  &_PathPlanner_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_PathPlanner_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_PathPlanner_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPlanner_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPlanner_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__PathPlanner_FeedbackMessage__get_type_hash,
  &core_interfaces__action__PathPlanner_FeedbackMessage__get_type_description,
  &core_interfaces__action__PathPlanner_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::PathPlanner_FeedbackMessage>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner_FeedbackMessage)() {
  return get_message_type_support_handle<core_interfaces::action::PathPlanner_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "core_interfaces/action/detail/path_planner__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace core_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t PathPlanner_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &core_interfaces__action__PathPlanner__get_type_hash,
  &core_interfaces__action__PathPlanner__get_type_description,
  &core_interfaces__action__PathPlanner__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<core_interfaces::action::PathPlanner>()
{
  using ::core_interfaces::action::rosidl_typesupport_cpp::PathPlanner_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  PathPlanner_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::core_interfaces::action::PathPlanner::Impl::SendGoalService>();
  PathPlanner_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::core_interfaces::action::PathPlanner::Impl::GetResultService>();
  PathPlanner_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::core_interfaces::action::PathPlanner::Impl::CancelGoalService>();
  PathPlanner_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::core_interfaces::action::PathPlanner::Impl::FeedbackMessage>();
  PathPlanner_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::core_interfaces::action::PathPlanner::Impl::GoalStatusMessage>();
  return &PathPlanner_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, PathPlanner)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<core_interfaces::action::PathPlanner>();
}

#ifdef __cplusplus
}
#endif
