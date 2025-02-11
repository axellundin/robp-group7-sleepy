// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice
#ifndef CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "core_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "core_interfaces/msg/detail/detected_msg__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
bool cdr_serialize_core_interfaces__msg__DetectedMsg(
  const core_interfaces__msg__DetectedMsg * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
bool cdr_deserialize_core_interfaces__msg__DetectedMsg(
  eprosima::fastcdr::Cdr &,
  core_interfaces__msg__DetectedMsg * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
size_t get_serialized_size_core_interfaces__msg__DetectedMsg(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
size_t max_serialized_size_core_interfaces__msg__DetectedMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
bool cdr_serialize_key_core_interfaces__msg__DetectedMsg(
  const core_interfaces__msg__DetectedMsg * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
size_t get_serialized_size_key_core_interfaces__msg__DetectedMsg(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
size_t max_serialized_size_key_core_interfaces__msg__DetectedMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_core_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, core_interfaces, msg, DetectedMsg)();

#ifdef __cplusplus
}
#endif

#endif  // CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
