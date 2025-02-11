// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from core_interfaces:srv/GeofenceCompliance.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "core_interfaces/srv/detail/geofence_compliance__functions.h"
#include "core_interfaces/srv/detail/geofence_compliance__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GeofenceCompliance_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GeofenceCompliance_Request_type_support_ids_t;

static const _GeofenceCompliance_Request_type_support_ids_t _GeofenceCompliance_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GeofenceCompliance_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GeofenceCompliance_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GeofenceCompliance_Request_type_support_symbol_names_t _GeofenceCompliance_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, srv, GeofenceCompliance_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, srv, GeofenceCompliance_Request)),
  }
};

typedef struct _GeofenceCompliance_Request_type_support_data_t
{
  void * data[2];
} _GeofenceCompliance_Request_type_support_data_t;

static _GeofenceCompliance_Request_type_support_data_t _GeofenceCompliance_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GeofenceCompliance_Request_message_typesupport_map = {
  2,
  "core_interfaces",
  &_GeofenceCompliance_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GeofenceCompliance_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GeofenceCompliance_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GeofenceCompliance_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GeofenceCompliance_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__srv__GeofenceCompliance_Request__get_type_hash,
  &core_interfaces__srv__GeofenceCompliance_Request__get_type_description,
  &core_interfaces__srv__GeofenceCompliance_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Request>()
{
  return &::core_interfaces::srv::rosidl_typesupport_cpp::GeofenceCompliance_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, srv, GeofenceCompliance_Request)() {
  return get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Request>();
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
// #include "core_interfaces/srv/detail/geofence_compliance__functions.h"
// already included above
// #include "core_interfaces/srv/detail/geofence_compliance__struct.hpp"
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

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GeofenceCompliance_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GeofenceCompliance_Response_type_support_ids_t;

static const _GeofenceCompliance_Response_type_support_ids_t _GeofenceCompliance_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GeofenceCompliance_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GeofenceCompliance_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GeofenceCompliance_Response_type_support_symbol_names_t _GeofenceCompliance_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, srv, GeofenceCompliance_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, srv, GeofenceCompliance_Response)),
  }
};

typedef struct _GeofenceCompliance_Response_type_support_data_t
{
  void * data[2];
} _GeofenceCompliance_Response_type_support_data_t;

static _GeofenceCompliance_Response_type_support_data_t _GeofenceCompliance_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GeofenceCompliance_Response_message_typesupport_map = {
  2,
  "core_interfaces",
  &_GeofenceCompliance_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GeofenceCompliance_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GeofenceCompliance_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GeofenceCompliance_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GeofenceCompliance_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__srv__GeofenceCompliance_Response__get_type_hash,
  &core_interfaces__srv__GeofenceCompliance_Response__get_type_description,
  &core_interfaces__srv__GeofenceCompliance_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Response>()
{
  return &::core_interfaces::srv::rosidl_typesupport_cpp::GeofenceCompliance_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, srv, GeofenceCompliance_Response)() {
  return get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Response>();
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
// #include "core_interfaces/srv/detail/geofence_compliance__functions.h"
// already included above
// #include "core_interfaces/srv/detail/geofence_compliance__struct.hpp"
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

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GeofenceCompliance_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GeofenceCompliance_Event_type_support_ids_t;

static const _GeofenceCompliance_Event_type_support_ids_t _GeofenceCompliance_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GeofenceCompliance_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GeofenceCompliance_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GeofenceCompliance_Event_type_support_symbol_names_t _GeofenceCompliance_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, srv, GeofenceCompliance_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, srv, GeofenceCompliance_Event)),
  }
};

typedef struct _GeofenceCompliance_Event_type_support_data_t
{
  void * data[2];
} _GeofenceCompliance_Event_type_support_data_t;

static _GeofenceCompliance_Event_type_support_data_t _GeofenceCompliance_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GeofenceCompliance_Event_message_typesupport_map = {
  2,
  "core_interfaces",
  &_GeofenceCompliance_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GeofenceCompliance_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GeofenceCompliance_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GeofenceCompliance_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GeofenceCompliance_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &core_interfaces__srv__GeofenceCompliance_Event__get_type_hash,
  &core_interfaces__srv__GeofenceCompliance_Event__get_type_description,
  &core_interfaces__srv__GeofenceCompliance_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Event>()
{
  return &::core_interfaces::srv::rosidl_typesupport_cpp::GeofenceCompliance_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, srv, GeofenceCompliance_Event)() {
  return get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Event>();
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
// #include "core_interfaces/srv/detail/geofence_compliance__struct.hpp"
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

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GeofenceCompliance_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GeofenceCompliance_type_support_ids_t;

static const _GeofenceCompliance_type_support_ids_t _GeofenceCompliance_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GeofenceCompliance_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GeofenceCompliance_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GeofenceCompliance_type_support_symbol_names_t _GeofenceCompliance_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core_interfaces, srv, GeofenceCompliance)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, srv, GeofenceCompliance)),
  }
};

typedef struct _GeofenceCompliance_type_support_data_t
{
  void * data[2];
} _GeofenceCompliance_type_support_data_t;

static _GeofenceCompliance_type_support_data_t _GeofenceCompliance_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GeofenceCompliance_service_typesupport_map = {
  2,
  "core_interfaces",
  &_GeofenceCompliance_service_typesupport_ids.typesupport_identifier[0],
  &_GeofenceCompliance_service_typesupport_symbol_names.symbol_name[0],
  &_GeofenceCompliance_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GeofenceCompliance_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GeofenceCompliance_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<core_interfaces::srv::GeofenceCompliance_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<core_interfaces::srv::GeofenceCompliance>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<core_interfaces::srv::GeofenceCompliance>,
  &core_interfaces__srv__GeofenceCompliance__get_type_hash,
  &core_interfaces__srv__GeofenceCompliance__get_type_description,
  &core_interfaces__srv__GeofenceCompliance__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace core_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<core_interfaces::srv::GeofenceCompliance>()
{
  return &::core_interfaces::srv::rosidl_typesupport_cpp::GeofenceCompliance_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, core_interfaces, srv, GeofenceCompliance)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<core_interfaces::srv::GeofenceCompliance>();
}

#ifdef __cplusplus
}
#endif
