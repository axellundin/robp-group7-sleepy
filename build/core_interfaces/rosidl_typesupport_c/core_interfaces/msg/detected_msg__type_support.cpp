// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "core_interfaces/msg/detail/detected_msg__struct.h"
#include "core_interfaces/msg/detail/detected_msg__type_support.h"
#include "core_interfaces/msg/detail/detected_msg__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace core_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _DetectedMsg_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DetectedMsg_type_support_ids_t;

static const _DetectedMsg_type_support_ids_t _DetectedMsg_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _DetectedMsg_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DetectedMsg_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DetectedMsg_type_support_symbol_names_t _DetectedMsg_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, core_interfaces, msg, DetectedMsg)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, core_interfaces, msg, DetectedMsg)),
  }
};

typedef struct _DetectedMsg_type_support_data_t
{
  void * data[2];
} _DetectedMsg_type_support_data_t;

static _DetectedMsg_type_support_data_t _DetectedMsg_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DetectedMsg_message_typesupport_map = {
  2,
  "core_interfaces",
  &_DetectedMsg_message_typesupport_ids.typesupport_identifier[0],
  &_DetectedMsg_message_typesupport_symbol_names.symbol_name[0],
  &_DetectedMsg_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t DetectedMsg_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DetectedMsg_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &core_interfaces__msg__DetectedMsg__get_type_hash,
  &core_interfaces__msg__DetectedMsg__get_type_description,
  &core_interfaces__msg__DetectedMsg__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace core_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, core_interfaces, msg, DetectedMsg)() {
  return &::core_interfaces::msg::rosidl_typesupport_c::DetectedMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
