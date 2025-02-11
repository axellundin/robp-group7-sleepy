// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from core_interfaces:action/MoveTo.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "core_interfaces/action/detail/move_to__functions.h"
#include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_Goal_type_support_ids_t;

static const _MoveTo_Goal_type_support_ids_t _MoveTo_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_Goal_type_support_symbol_names_t _MoveTo_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_Goal)),
  }
};

typedef struct _MoveTo_Goal_type_support_data_t
{
  void * data[2];
} _MoveTo_Goal_type_support_data_t;

static _MoveTo_Goal_type_support_data_t _MoveTo_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_Goal_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_Goal__get_type_hash,
  &core_interfaces__action__MoveTo_Goal__get_type_description,
  &core_interfaces__action__MoveTo_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_Goal>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_Goal)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_Goal>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_Result_type_support_ids_t;

static const _MoveTo_Result_type_support_ids_t _MoveTo_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_Result_type_support_symbol_names_t _MoveTo_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_Result)),
  }
};

typedef struct _MoveTo_Result_type_support_data_t
{
  void * data[2];
} _MoveTo_Result_type_support_data_t;

static _MoveTo_Result_type_support_data_t _MoveTo_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_Result_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_Result_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_Result_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_Result__get_type_hash,
  &core_interfaces__action__MoveTo_Result__get_type_description,
  &core_interfaces__action__MoveTo_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_Result>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_Result)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_Result>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_Feedback_type_support_ids_t;

static const _MoveTo_Feedback_type_support_ids_t _MoveTo_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_Feedback_type_support_symbol_names_t _MoveTo_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_Feedback)),
  }
};

typedef struct _MoveTo_Feedback_type_support_data_t
{
  void * data[2];
} _MoveTo_Feedback_type_support_data_t;

static _MoveTo_Feedback_type_support_data_t _MoveTo_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_Feedback_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_Feedback__get_type_hash,
  &core_interfaces__action__MoveTo_Feedback__get_type_description,
  &core_interfaces__action__MoveTo_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_Feedback>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_Feedback)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_Feedback>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_SendGoal_Request_type_support_ids_t;

static const _MoveTo_SendGoal_Request_type_support_ids_t _MoveTo_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_SendGoal_Request_type_support_symbol_names_t _MoveTo_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_SendGoal_Request)),
  }
};

typedef struct _MoveTo_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _MoveTo_SendGoal_Request_type_support_data_t;

static _MoveTo_SendGoal_Request_type_support_data_t _MoveTo_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_SendGoal_Request_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_SendGoal_Request__get_type_hash,
  &core_interfaces__action__MoveTo_SendGoal_Request__get_type_description,
  &core_interfaces__action__MoveTo_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Request>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_SendGoal_Request)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Request>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_SendGoal_Response_type_support_ids_t;

static const _MoveTo_SendGoal_Response_type_support_ids_t _MoveTo_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_SendGoal_Response_type_support_symbol_names_t _MoveTo_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_SendGoal_Response)),
  }
};

typedef struct _MoveTo_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _MoveTo_SendGoal_Response_type_support_data_t;

static _MoveTo_SendGoal_Response_type_support_data_t _MoveTo_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_SendGoal_Response_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_SendGoal_Response__get_type_hash,
  &core_interfaces__action__MoveTo_SendGoal_Response__get_type_description,
  &core_interfaces__action__MoveTo_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Response>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_SendGoal_Response)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Response>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_SendGoal_Event_type_support_ids_t;

static const _MoveTo_SendGoal_Event_type_support_ids_t _MoveTo_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_SendGoal_Event_type_support_symbol_names_t _MoveTo_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_SendGoal_Event)),
  }
};

typedef struct _MoveTo_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _MoveTo_SendGoal_Event_type_support_data_t;

static _MoveTo_SendGoal_Event_type_support_data_t _MoveTo_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_SendGoal_Event_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_SendGoal_Event__get_type_hash,
  &core_interfaces__action__MoveTo_SendGoal_Event__get_type_description,
  &core_interfaces__action__MoveTo_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Event>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_SendGoal_Event)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Event>();
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
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_SendGoal_type_support_ids_t;

static const _MoveTo_SendGoal_type_support_ids_t _MoveTo_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_SendGoal_type_support_symbol_names_t _MoveTo_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_SendGoal)),
  }
};

typedef struct _MoveTo_SendGoal_type_support_data_t
{
  void * data[2];
} _MoveTo_SendGoal_type_support_data_t;

static _MoveTo_SendGoal_type_support_data_t _MoveTo_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_SendGoal_service_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveTo_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<core_interfaces::action::MoveTo_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<core_interfaces::action::MoveTo_SendGoal>,
  &core_interfaces__action__MoveTo_SendGoal__get_type_hash,
  &core_interfaces__action__MoveTo_SendGoal__get_type_description,
  &core_interfaces__action__MoveTo_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<core_interfaces::action::MoveTo_SendGoal>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<core_interfaces::action::MoveTo_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_GetResult_Request_type_support_ids_t;

static const _MoveTo_GetResult_Request_type_support_ids_t _MoveTo_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_GetResult_Request_type_support_symbol_names_t _MoveTo_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_GetResult_Request)),
  }
};

typedef struct _MoveTo_GetResult_Request_type_support_data_t
{
  void * data[2];
} _MoveTo_GetResult_Request_type_support_data_t;

static _MoveTo_GetResult_Request_type_support_data_t _MoveTo_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_GetResult_Request_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_GetResult_Request__get_type_hash,
  &core_interfaces__action__MoveTo_GetResult_Request__get_type_description,
  &core_interfaces__action__MoveTo_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Request>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_GetResult_Request)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Request>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_GetResult_Response_type_support_ids_t;

static const _MoveTo_GetResult_Response_type_support_ids_t _MoveTo_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_GetResult_Response_type_support_symbol_names_t _MoveTo_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_GetResult_Response)),
  }
};

typedef struct _MoveTo_GetResult_Response_type_support_data_t
{
  void * data[2];
} _MoveTo_GetResult_Response_type_support_data_t;

static _MoveTo_GetResult_Response_type_support_data_t _MoveTo_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_GetResult_Response_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_GetResult_Response__get_type_hash,
  &core_interfaces__action__MoveTo_GetResult_Response__get_type_description,
  &core_interfaces__action__MoveTo_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Response>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_GetResult_Response)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Response>();
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
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_GetResult_Event_type_support_ids_t;

static const _MoveTo_GetResult_Event_type_support_ids_t _MoveTo_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_GetResult_Event_type_support_symbol_names_t _MoveTo_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_GetResult_Event)),
  }
};

typedef struct _MoveTo_GetResult_Event_type_support_data_t
{
  void * data[2];
} _MoveTo_GetResult_Event_type_support_data_t;

static _MoveTo_GetResult_Event_type_support_data_t _MoveTo_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_GetResult_Event_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_GetResult_Event__get_type_hash,
  &core_interfaces__action__MoveTo_GetResult_Event__get_type_description,
  &core_interfaces__action__MoveTo_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Event>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_GetResult_Event)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Event>();
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
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_GetResult_type_support_ids_t;

static const _MoveTo_GetResult_type_support_ids_t _MoveTo_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_GetResult_type_support_symbol_names_t _MoveTo_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_GetResult)),
  }
};

typedef struct _MoveTo_GetResult_type_support_data_t
{
  void * data[2];
} _MoveTo_GetResult_type_support_data_t;

static _MoveTo_GetResult_type_support_data_t _MoveTo_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_GetResult_service_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveTo_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::action::MoveTo_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<core_interfaces::action::MoveTo_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<core_interfaces::action::MoveTo_GetResult>,
  &core_interfaces__action__MoveTo_GetResult__get_type_hash,
  &core_interfaces__action__MoveTo_GetResult__get_type_description,
  &core_interfaces__action__MoveTo_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<core_interfaces::action::MoveTo_GetResult>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<core_interfaces::action::MoveTo_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "core_interfaces/action/detail/move_to__functions.h"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

typedef struct _MoveTo_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveTo_FeedbackMessage_type_support_ids_t;

static const _MoveTo_FeedbackMessage_type_support_ids_t _MoveTo_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveTo_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveTo_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveTo_FeedbackMessage_type_support_symbol_names_t _MoveTo_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, action, MoveTo_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, action, MoveTo_FeedbackMessage)),
  }
};

typedef struct _MoveTo_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _MoveTo_FeedbackMessage_type_support_data_t;

static _MoveTo_FeedbackMessage_type_support_data_t _MoveTo_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveTo_FeedbackMessage_message_typesupport_map = {
  2,
  "core_interfaces",
  &_MoveTo_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_MoveTo_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_MoveTo_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveTo_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveTo_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__action__MoveTo_FeedbackMessage__get_type_hash,
  &core_interfaces__action__MoveTo_FeedbackMessage__get_type_description,
  &core_interfaces__action__MoveTo_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::action::MoveTo_FeedbackMessage>()
{
  return &::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo_FeedbackMessage)() {
  return get_message_type_support_handle<core_interfaces::action::MoveTo_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"
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

static rosidl_action_type_support_t MoveTo_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &core_interfaces__action__MoveTo__get_type_hash,
  &core_interfaces__action__MoveTo__get_type_description,
  &core_interfaces__action__MoveTo__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<core_interfaces::action::MoveTo>()
{
  using ::core_interfaces::action::rosidl_typesupport_cpp::MoveTo_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  MoveTo_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::core_interfaces::action::MoveTo::Impl::SendGoalService>();
  MoveTo_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::core_interfaces::action::MoveTo::Impl::GetResultService>();
  MoveTo_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::core_interfaces::action::MoveTo::Impl::CancelGoalService>();
  MoveTo_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::core_interfaces::action::MoveTo::Impl::FeedbackMessage>();
  MoveTo_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::core_interfaces::action::MoveTo::Impl::GoalStatusMessage>();
  return &MoveTo_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, action, MoveTo)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<core_interfaces::action::MoveTo>();
}

#ifdef __cplusplus
}
#endif
